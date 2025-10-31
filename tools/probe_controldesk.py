# -*- coding: utf-8 -*-
"""ControlDesk variable probe (clean version).

Purpose:
- Connect to ControlDesk via COM
- Ensure online calibration/measurement is running
- Enumerate top-level and nested platforms
- For each nested platform, report whether ActiveVariableDescription.Variables exists
- Optionally scan for keywords (e.g. throttle/steer/brake) and print current values

Usage (PowerShell):
  venv\Scripts\python.exe Scenic\tools\probe_controldesk.py --keywords throttle steer brake --max 50

Notes:
- Read values using ValueConverted when available
- This script is read-focused; it does not write variables
"""

from __future__ import annotations

import argparse
import sys
from typing import Iterable, List, Optional, Sequence, Tuple


# ----------------------------- Logging -------------------------------------

def log(level: str, msg: str):
    sys.stdout.write(f"[{level}] {msg}\n")
    sys.stdout.flush()


def info(msg: str):
    log("INFO", msg)


def warn(msg: str):
    log("WARN", msg)


def err(msg: str):
    log("ERROR", msg)


# ----------------------------- COM Helpers ---------------------------------

def com_connect(prog_id: str = "ControlDeskNG.Application"):
    import pythoncom
    from win32com.client import Dispatch

    pythoncom.CoInitialize()
    return Dispatch(prog_id)


def safe_get(obj, name: str, default=None):
    try:
        return getattr(obj, name)
    except Exception:
        return default


def get_prop(obj, name: str, default=None):
    try:
        val = getattr(obj, name)
    except Exception:
        return default
    try:
        if callable(val):
            return val()
    except Exception:
        return val
    return val


def coll_to_list(coll) -> List:
    try:
        count = safe_get(coll, "Count", None)
        if isinstance(count, int) and count > 0:
            return [coll.Item(i) for i in range(count)]
    except Exception:
        pass
    try:
        return list(coll)
    except Exception:
        return []


# ----------------------------- Core Logic ----------------------------------

def list_platforms(app) -> Tuple[List, List[Tuple[object, str]]]:
    """Return (top_level_platforms, nested_entries[(plat_obj, disp_name)])."""
    exp = get_prop(app, "ActiveExperiment", None)
    if exp is None:
        err("No ActiveExperiment. Open one in ControlDesk.")
        return [], []
    plats = get_prop(exp, "Platforms", None)
    if plats is None:
        err("ActiveExperiment has no Platforms.")
        return [], []

    top = coll_to_list(plats)
    top_names = [safe_get(p, "Name", "<Platform>") for p in top]
    info("Top-level Platforms: " + ", ".join(top_names) if top_names else "Top-level Platforms: <none>")

    nested: List[Tuple[object, str]] = []
    for p in top:
        pname = safe_get(p, "Name", "<Platform>")
        inner_plats = safe_get(p, "Platforms", None)
        if inner_plats is None:
            continue
        inner = coll_to_list(inner_plats)
        inner_names = [safe_get(ip, "Name", "<Inner>") for ip in inner]
        if inner_names:
            info(f"Nested Platforms under '{pname}': {', '.join(inner_names)}")
        for ip in inner:
            iname = safe_get(ip, "Name", "<Inner>")
            nested.append((ip, f"{pname}/{iname}"))
    return top, nested


def report_variables_availability(nested_entries: Sequence[Tuple[object, str]]):
    for plat_obj, disp in nested_entries:
        vdesc = get_prop(plat_obj, "ActiveVariableDescription", None)
        if vdesc is None:
            warn(f"{disp}: no ActiveVariableDescription")
            continue
        # Try to ensure content is fresh
        try:
            if hasattr(vdesc, "Reload"):
                vdesc.Reload()
        except Exception:
            pass
        try:
            if hasattr(vdesc, "CheckSourceForChanges"):
                vdesc.CheckSourceForChanges()
        except Exception:
            pass

        vars_obj = get_prop(vdesc, "Variables", None)
        if vars_obj is None:
            info(f"{disp}: Variables = None")
            continue

        count = safe_get(vars_obj, "Count", None)
        count_s = str(count) if isinstance(count, int) else "<unknown>"
        info(f"{disp}: Variables available (Count={count_s})")


def scan_keywords(nested_entries: Sequence[Tuple[object, str]], keywords: Sequence[str], max_items: int):
    if not keywords:
        return
    kw_lower = [k.lower() for k in keywords]
    for plat_obj, disp in nested_entries:
        vdesc = get_prop(plat_obj, "ActiveVariableDescription", None)
        vars_obj = get_prop(vdesc, "Variables", None) if vdesc is not None else None
        if vars_obj is None:
            continue

        info(f"Scanning {disp} for keywords: {', '.join(keywords)}")
        hits = 0
        try:
            count = safe_get(vars_obj, "Count", None)
            iterator: Iterable[Tuple[int, object]]
            if isinstance(count, int) and count > 0:
                iterator = ((i, vars_obj.Item(i)) for i in range(count))
            else:
                iterator = enumerate(vars_obj)

            for i, item in iterator:
                name = safe_get(item, "Name", None)
                if not isinstance(name, str):
                    continue
                low = name.lower()
                if any(k in low for k in kw_lower):
                    value = safe_get(item, "ValueConverted", safe_get(item, "Value", None))
                    info(f"  [hit] {disp} idx={i} name={name} value={value}")
                    hits += 1
                    if hits >= max_items:
                        break
        except Exception as e:
            warn(f"{disp}: scan error: {e}")


def start_online_and_measurement(app):
    try:
        app.CalibrationManagement.StartOnlineCalibration()
        app.MeasurementDataManagement.Start()
        info("Online calibration + measurement started")
    except Exception as e:
        warn(f"Could not start online/measurement: {e}")


# ------------------------- RootGroup Traversal ------------------------------

def _enum_collection(coll) -> Iterable[object]:
    try:
        count = safe_get(coll, 'Count', None)
        if isinstance(count, int) and count > 0:
            for i in range(count):
                try:
                    yield coll.Item(i)
                except Exception:
                    continue
            return
    except Exception:
        pass
    try:
        for it in coll:
            yield it
    except Exception:
        return


def traverse_rootgroup_for_keywords(nested_entries: Sequence[Tuple[object, str]], keywords: Sequence[str], max_depth: int, max_hits: int):
    if not keywords:
        return
    kw_lower = [k.lower() for k in keywords]

    def visit(node, disp_path: str, depth: int, hits_left: List[int]):
        if hits_left[0] <= 0 or depth < 0:
            return
        name = safe_get(node, 'Name', '<group>')
        here = f"{disp_path}/{name}" if disp_path else str(name)

        # Variables under this node (common containers: Variables, Items, Children)
        for cont_name in ("Variables", "Items", "Children"):
            cont = safe_get(node, cont_name, None)
            if cont is None:
                continue
            for it in _enum_collection(cont):
                var_name = safe_get(it, 'Name', None)
                if not isinstance(var_name, str):
                    continue
                low = var_name.lower()
                if any(k in low for k in kw_lower):
                    full_path = f"{here}/{var_name}" if here else var_name
                    val = safe_get(it, 'ValueConverted', safe_get(it, 'Value', None))
                    info(f"  [rootgroup-hit] {full_path} = {val}")
                    hits_left[0] -= 1
                    if hits_left[0] <= 0:
                        return

        # Recurse into groups
        for grp_name in ("Groups", "Children"):
            groups = safe_get(node, grp_name, None)
            if groups is None:
                continue
            for child in _enum_collection(groups):
                if hits_left[0] <= 0:
                    return
                visit(child, here, depth - 1, hits_left)

    for plat_obj, disp in nested_entries:
        vdesc = get_prop(plat_obj, 'ActiveVariableDescription', None)
        root = get_prop(vdesc, 'RootGroup', None) if vdesc is not None else None
        if root is None:
            continue
        info(f"Traversing RootGroup on {disp} for keywords: {', '.join(keywords)}")
        visit(root, disp_path=disp, depth=max_depth, hits_left=[max_hits])


def _find_group_by_path(root, segments: Sequence[str]):
    node = root
    for seg in segments:
        if not seg:
            continue
        next_node = None
        # Try through 'Groups' then 'Children'
        for coll_name in ("Groups", "Children"):
            coll = safe_get(node, coll_name, None)
            if coll is None:
                continue
            for child in _enum_collection(coll):
                name = safe_get(child, 'Name', None)
                if name == seg:
                    next_node = child
                    break
            if next_node is not None:
                break
        if next_node is None:
            return None
        node = next_node
    return node


def list_subtree_external_userdata(plat_obj, disp: str, max_items: int, probe_writable: bool):
    vdesc = get_prop(plat_obj, 'ActiveVariableDescription', None)
    root = get_prop(vdesc, 'RootGroup', None) if vdesc is not None else None
    if root is None:
        return
    # Path under Model Root
    path_segs = [
        'Model Root',
        'Environment',
        'Maneuver',
        'PlantModel',
        'ExternalUserData',
    ]
    grp = _find_group_by_path(root, path_segs)
    if grp is None:
        warn(f"{disp}: ExternalUserData path not found")
        return
    info(f"Listing {disp}/{'/'.join(path_segs)} (up to {max_items})")

    def visit(node, prefix: str, remaining: List[int]):
        if remaining[0] <= 0:
            return
        # Variables / Items at this node
        for cont_name in ("Variables", "Items", "Children"):
            cont = safe_get(node, cont_name, None)
            if cont is None:
                continue
            for it in _enum_collection(cont):
                if remaining[0] <= 0:
                    return
                name = safe_get(it, 'Name', None)
                if not isinstance(name, str):
                    continue
                # If this is a variable-like item, print value
                val = safe_get(it, 'ValueConverted', safe_get(it, 'Value', None))
                if val is not None or cont_name != 'Children':
                    extra = ""
                    if probe_writable and isinstance(val, (int, float)):
                        try:
                            if hasattr(it, 'ValueConverted'):
                                it.ValueConverted = val
                            elif hasattr(it, 'Value'):
                                it.Value = val
                            extra = " (writable)"
                        except Exception:
                            extra = " (read-only)"
                    info(f"  {prefix}{name} = {val}{extra}")
                    remaining[0] -= 1
                # Recurse into children nodes
                child_groups = safe_get(it, 'Groups', None)
                if child_groups is not None and remaining[0] > 0:
                    visit(it, prefix + name + '/', remaining)

        # Also explicit 'Groups' on current node
        groups = safe_get(node, 'Groups', None)
        if groups is not None and remaining[0] > 0:
            for child in _enum_collection(groups):
                if remaining[0] <= 0:
                    return
                cname = safe_get(child, 'Name', '<group>')
                visit(child, prefix + cname + '/', remaining)

    visit(grp, prefix='', remaining=[max_items])


def list_subtree_vesi_interface(plat_obj, disp: str, max_items: int, probe_writable: bool):
    vdesc = get_prop(plat_obj, 'ActiveVariableDescription', None)
    root = get_prop(vdesc, 'RootGroup', None) if vdesc is not None else None
    if root is None:
        return
    # Path to VesiInterface/VESIResultData_Manual/vehicle_inputs
    path_segs = [
        'Model Root',
        'VesiInterface',
        'VESIResultData_Manual',
        'vehicle_inputs',
    ]
    grp = _find_group_by_path(root, path_segs)
    if grp is None:
        warn(f"{disp}: VesiInterface/VESIResultData_Manual/vehicle_inputs path not found")
        return
    info(f"Listing {disp}/{'/'.join(path_segs)} (up to {max_items})")

    def visit(node, prefix: str, remaining: List[int]):
        if remaining[0] <= 0:
            return
        # Variables / Items at this node
        for cont_name in ("Variables", "Items", "Children"):
            cont = safe_get(node, cont_name, None)
            if cont is None:
                continue
            for it in _enum_collection(cont):
                if remaining[0] <= 0:
                    return
                name = safe_get(it, 'Name', None)
                if not isinstance(name, str):
                    continue
                # If this is a variable-like item, print value
                val = safe_get(it, 'ValueConverted', safe_get(it, 'Value', None))
                if val is not None or cont_name != 'Children':
                    extra = ""
                    if probe_writable and isinstance(val, (int, float)):
                        try:
                            if hasattr(it, 'ValueConverted'):
                                it.ValueConverted = val
                            elif hasattr(it, 'Value'):
                                it.Value = val
                            extra = " (writable)"
                        except Exception:
                            extra = " (read-only)"
                    info(f"  {prefix}{name} = {val}{extra}")
                    remaining[0] -= 1
                # Recurse into children nodes
                child_groups = safe_get(it, 'Groups', None)
                if child_groups is not None and remaining[0] > 0:
                    visit(it, prefix + name + '/', remaining)

        # Also explicit 'Groups' on current node
        groups = safe_get(node, 'Groups', None)
        if groups is not None and remaining[0] > 0:
            for child in _enum_collection(groups):
                if remaining[0] <= 0:
                    return
                cname = safe_get(child, 'Name', '<group>')
                visit(child, prefix + cname + '/', remaining)

    visit(grp, prefix='', remaining=[max_items])


# ----------------------------- Write Helpers --------------------------------

def try_set_pos_acc_pedal_maneuver(app, platform_inner_name: str, new_value: float) -> bool:
    """Try to set one of several accelerator pedal variables to new_value.

    Strategy:
      1) Look for exact candidate Names in priority order and attempt write
      2) If none succeed, try fuzzy matches containing 'AccPedal' and 'Pos' or 'Request' or 'Ctrl'
    """
    try:
        exp = get_prop(app, "ActiveExperiment", None)
        plats = get_prop(exp, "Platforms", None)
        # First outer platform by index
        count = safe_get(plats, 'Count', None)
        outer = plats.Item(0) if isinstance(count, int) and count > 0 else list(plats)[0]
        inner_plats = get_prop(outer, 'Platforms', None)
        inner = inner_plats.Item(platform_inner_name)
        vdesc = get_prop(inner, 'ActiveVariableDescription', None)
        vars_obj = get_prop(vdesc, 'Variables', None)
        if vars_obj is None:
            warn(f"{platform_inner_name}: Variables is None; cannot write")
            return False

        candidates_priority = [
            "Pos_AccPedal_Maneuver[%]",
            "Pos_AccPedal_Request[%]",
            "Pos_AccPedal_Ctrl[%]",
            "Pos_AccPedal_Driver_Des[%]",
            "Pos_AccPedal_Driver[%]",
            "Pos_AccPedal[%]",
        ]

        # Build an index of Name -> (idx, item)
        name_to_item = {}
        count = safe_get(vars_obj, 'Count', None)
        iterator = ((i, vars_obj.Item(i)) for i in range(count)) if isinstance(count, int) and count > 0 else enumerate(vars_obj)
        for i, item in iterator:
            nm = safe_get(item, 'Name', None)
            if isinstance(nm, str):
                # Keep first occurrence only
                name_to_item.setdefault(nm, (i, item))

        # Step 1: exact candidates
        for cand in candidates_priority:
            if cand in name_to_item:
                i, item = name_to_item[cand]
                info(f"Attempt write (exact): idx={i} name={cand} -> {new_value}")
                try:
                    if hasattr(item, 'ValueConverted'):
                        item.ValueConverted = new_value
                    elif hasattr(item, 'Value'):
                        item.Value = new_value
                    readback = safe_get(item, 'ValueConverted', safe_get(item, 'Value', None))
                    info(f"WRITE OK idx={i} name={cand} -> {readback}")
                    return True
                except Exception as e:
                    warn(f"WRITE FAIL idx={i} name={cand}: {e}")

        # Step 2: fuzzy matches
        info("Trying fuzzy matches for accelerator pedal variables...")
        fuzz_candidates: List[Tuple[str, Tuple[int, object]]] = []
        for nm, pair in name_to_item.items():
            l = nm.lower()
            if "accpedal" in l and ("pos" in l or "request" in l or "ctrl" in l):
                fuzz_candidates.append((nm, pair))

        for nm, (i, item) in fuzz_candidates:
            info(f"Attempt write (fuzzy): idx={i} name={nm} -> {new_value}")
            try:
                if hasattr(item, 'ValueConverted'):
                    item.ValueConverted = new_value
                elif hasattr(item, 'Value'):
                    item.Value = new_value
                readback = safe_get(item, 'ValueConverted', safe_get(item, 'Value', None))
                info(f"WRITE OK idx={i} name={nm} -> {readback}")
                return True
            except Exception as e:
                warn(f"WRITE FAIL idx={i} name={nm}: {e}")

        warn("No writable accelerator pedal variable found")
        return False
    except Exception as e:
        err(f"Write helper error: {e}")
        return False


def coerce_value(text: str):
    t = text.strip().lower()
    # First try to parse as number (important for numeric variables)
    try:
        if "." in text.strip():  # Check original string, not lowercased
            return float(text.strip())
        return int(text.strip())
    except Exception:
        pass
    # Then check for boolean strings
    if t in ("true", "t", "yes", "y"):
        return True
    if t in ("false", "f", "no", "n"):
        return False
    return text


def read_write_by_key_path(app, platform_inner_name: Optional[str], key_path: str, set_value_text: Optional[str]) -> bool:
    """Try to access key_path on specified platform, or all platforms if platform_inner_name is None."""
    try:
        exp = get_prop(app, "ActiveExperiment", None)
        plats = get_prop(exp, "Platforms", None)
        count = safe_get(plats, 'Count', None)
        outer = plats.Item(0) if isinstance(count, int) and count > 0 else list(plats)[0]
        inner_plats = get_prop(outer, 'Platforms', None)
        
        # Build list of platforms to try
        platforms_to_try = []
        if platform_inner_name:
            platforms_to_try = [(platform_inner_name, inner_plats.Item(platform_inner_name))]
        else:
            # Try all nested platforms
            inner_list = coll_to_list(inner_plats)
            for inner in inner_list:
                name = safe_get(inner, 'Name', None)
                if isinstance(name, str):
                    platforms_to_try.append((name, inner))
        
        for plat_name, inner in platforms_to_try:
            vdesc = get_prop(inner, 'ActiveVariableDescription', None)
            vars_obj = get_prop(vdesc, 'Variables', None)
            if vars_obj is None:
                warn(f"{plat_name}: Variables is None; skipping")
                continue

            info(f"Trying {plat_name}: Access by key: {key_path}")
            try:
                item = vars_obj[key_path]
            except Exception as e:
                info(f"{plat_name}: Key lookup failed: {e}")
                continue

            current = safe_get(item, 'ValueConverted', safe_get(item, 'Value', None))
            info(f"{plat_name}: READ OK: {key_path} -> {current}")

            if set_value_text is None:
                return True

            new_val = coerce_value(set_value_text)
            info(f"{plat_name}: Attempt write: {key_path} -> {new_val}")
            try:
                if hasattr(item, 'ValueConverted'):
                    item.ValueConverted = new_val
                elif hasattr(item, 'Value'):
                    item.Value = new_val
                readback = safe_get(item, 'ValueConverted', safe_get(item, 'Value', None))
                info(f"{plat_name}: WRITE OK: {key_path} -> {readback}")
                return True
            except Exception as e:
                err(f"{plat_name}: WRITE FAIL: {e}")
                continue
        
        err("Key-path access failed on all platforms")
        return False
    except Exception as e:
        err(f"Key-path helper error: {e}")
        return False


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="ControlDesk variable probe (clean)")
    parser.add_argument("--prog-id", default="ControlDeskNG.Application", help="COM ProgID for ControlDesk")
    parser.add_argument("--max", type=int, default=50, help="Max hits to print per platform when scanning keywords")
    parser.add_argument(
        "--keywords",
        nargs="*",
        default=[
            "brake",
            "braking",
            "front",
            "rear",
            "pedal",
        ],
        help="Keywords to scan for (brake-related by default)",
    )
    parser.add_argument("--set-acc-pedal", type=float, default=None, help="If provided, set Pos_AccPedal_Maneuver[%] on Platform_2 to this value")
    parser.add_argument("--key-path", type=str, default=None, help="Variables[] key path to read/write (e.g. Platform()://.../Value)")
    parser.add_argument("--set-by-key", type=str, default=None, help="Value to set for --key-path (bools as true/false, numbers ok)")
    parser.add_argument("--focus-external-userdata", action="store_true", help="List variables under Maneuver/PlantModel/ExternalUserData on Platform_2")
    parser.add_argument("--focus-vesi-interface", action="store_true", help="List variables under VesiInterface/VESIResultData_Manual/vehicle_inputs on Platform_2")
    parser.add_argument("--focus-racecontrol", action="store_true", help="List variables under RaceControl/race_control on Platform_2")
    parser.add_argument("--probe-writable", action="store_true", help="Attempt no-op write to detect writable variables when listing")
    parser.add_argument("--max-list", type=int, default=200, help="Max variables to list when focusing a subtree")
    args = parser.parse_args(argv)

    try:
        app = com_connect(args.prog_id)
        info("Connected to ControlDesk application")

        exp = get_prop(app, "ActiveExperiment", None)
        if exp is None:
            err("No ActiveExperiment; please open one in ControlDesk")
            return 1
        info(f"ActiveExperiment: {safe_get(exp, 'Name', 'Unknown')}")

        start_online_and_measurement(app)

        top, nested = list_platforms(app)
        if not nested:
            warn("No nested platforms found")

        # Commented out unnecessary searches
        # report_variables_availability(nested)
        # scan_keywords(nested, args.keywords, args.max)
        # traverse_rootgroup_for_keywords(nested, args.keywords, max_depth=6, max_hits=args.max)

        # if args.focus_external_userdata:
        #     # Find Platform_2 entry
        #     plat2 = None
        #     for plat_obj, disp in nested:
        #         if disp.endswith('/Platform_2'):
        #             plat2 = (plat_obj, disp)
        #             break
        #     if plat2 is None:
        #         warn("Platform_2 not found among nested platforms")
        #     else:
        #         list_subtree_external_userdata(plat2[0], plat2[1], max_items=args.max_list, probe_writable=args.probe_writable)

        if args.focus_vesi_interface:
            # Check all nested platforms for VesiInterface
            found_any = False
            for plat_obj, disp in nested:
                info(f"Checking {disp} for VesiInterface...")
                vdesc = get_prop(plat_obj, 'ActiveVariableDescription', None)
                root = get_prop(vdesc, 'RootGroup', None) if vdesc is not None else None
                if root is None:
                    continue
                path_segs = ['Model Root', 'VesiInterface', 'VESIResultData_Manual', 'vehicle_inputs']
                grp = _find_group_by_path(root, path_segs)
                if grp is not None:
                    found_any = True
                    list_subtree_vesi_interface(plat_obj, disp, max_items=args.max_list, probe_writable=args.probe_writable)
            if not found_any:
                warn("VesiInterface/VESIResultData_Manual/vehicle_inputs not found on any platform")

        # Optional write step: accelerator pedal
        # if args.set_acc_pedal is not None and args.set_acc_pedal >= 0:
        #     success = try_set_pos_acc_pedal_maneuver(app, platform_inner_name="Platform_2", new_value=float(args.set_acc_pedal))
        #     if not success:
        #         warn("Setting Pos_AccPedal_Maneuver[%] did not succeed")

        # Optional write step: direct key-path access
        # Try all platforms if key_path doesn't specify, otherwise try specified platform
        if args.key_path is not None:
            # If key_path contains "Platform()://", try all platforms
            if "Platform()://" in args.key_path:
                read_write_by_key_path(app, platform_inner_name=None, key_path=args.key_path, set_value_text=args.set_by_key)
            else:
                read_write_by_key_path(app, platform_inner_name="Platform_2", key_path=args.key_path, set_value_text=args.set_by_key)
        return 0
    except Exception as e:
        err(f"Probe error: {e}")
        return 2


if __name__ == "__main__":
    sys.exit(main())


