# -*- coding: utf-8 -*-
"""Disposable script to probe ControlDesk COM and enumerate variables.

Run with your venv's Python while ControlDesk is connected to VEOS.
"""

import sys
import traceback


def log(msg: str):
    sys.stdout.write(msg + "\n")
    sys.stdout.flush()


def try_connect(prog_id: str = "ControlDeskNG.Application"):
    import pythoncom
    from win32com.client import Dispatch
    pythoncom.CoInitialize()
    return Dispatch(prog_id)


def safe_getattr(obj, name, default=None):
    try:
        return getattr(obj, name)
    except Exception:
        return default


def get_prop(obj, name, default=None):
    """Get COM property; if callable, attempt to invoke without args."""
    try:
        val = getattr(obj, name)
    except Exception:
        return default
    # If the property is a zero-arg callable, try calling it
    try:
        if callable(val):
            return val()
    except Exception:
        pass
    return val


def dump_com_object(title: str, obj, max_items: int = 200):
    log(f"[introspect] {title} type={type(obj)}")
    # Try COM property map
    prop_map = safe_getattr(obj, "_prop_map_get_", None)
    if isinstance(prop_map, dict) and prop_map:
        names = list(prop_map.keys())[:max_items]
        log(f"[introspect] {title} _prop_map_get_ keys ({len(prop_map)}):")
        for n in names:
            log(f"  - {n}")
    # Fallback: dir()
    try:
        names = [n for n in dir(obj) if not n.startswith('_')]
        names = names[:max_items]
        if names:
            log(f"[introspect] {title} dir() sample:")
            for n in names:
                log(f"  - {n}")
    except Exception:
        pass


def enumerate_variables(variables_obj, max_items=500):
    """Try multiple enumeration strategies; yield (index, name, obj_or_value)."""
    yielded = 0

    # Strategy 1: Use Count + Item(i)
    count = safe_getattr(variables_obj, "Count", None)
    if isinstance(count, int) and count > 0:
        log(f"[enum] Using Count/Item for {count} variables")
        for i in range(count):
            if yielded >= max_items:
                return
            try:
                item = variables_obj.Item(i)
            except Exception:
                continue
            name = safe_getattr(item, "Name", None)
            if name is None:
                try:
                    name = str(item)
                except Exception:
                    name = f"<item_{i}>"
            yield (i, name, item)
            yielded += 1
        return

    # Strategy 2: Direct iteration
    try:
        log("[enum] Attempting direct iteration over Variables")
        for i, item in enumerate(variables_obj):
            if yielded >= max_items:
                return
            name = safe_getattr(item, "Name", None)
            if name is None:
                try:
                    name = str(item)
                except Exception:
                    name = f"<item_{i}>"
            yield (i, name, item)
            yielded += 1
        return
    except Exception:
        pass

    # Strategy 3: Keys/Items dict-like
    keys = safe_getattr(variables_obj, "Keys", None)
    if keys is not None:
        log("[enum] Attempting Keys-based enumeration")
        try:
            for i, k in enumerate(keys):
                if yielded >= max_items:
                    return
                try:
                    item = variables_obj[k]
                except Exception:
                    item = None
                yield (i, str(k), item)
                yielded += 1
            return
        except Exception:
            pass

    log("[enum] Could not enumerate variables with known strategies")


def _enum_collection(obj, label: str, max_items=200):
    """Yield items from a COM collection using multiple strategies."""
    yielded = 0
    count = safe_getattr(obj, 'Count', None)
    if isinstance(count, int) and count > 0:
        log(f"[enum] {label}: Count={count}")
        for i in range(count):
            if yielded >= max_items:
                return
            try:
                it = obj.Item(i)
            except Exception:
                continue
            yield i, it
            yielded += 1
        return
    try:
        for i, it in enumerate(obj):
            if yielded >= max_items:
                return
            yield i, it
            yielded += 1
    except Exception:
        pass


def traverse_root_group(root, max_depth=3, max_print=500):
    """Traverse RootGroup hierarchy to find variables with ValueConverted."""
    log("[cd] Traversing RootGroup...")
    printed = 0

    def visit(node, depth):
        nonlocal printed
        if depth > max_depth or printed >= max_print:
            return
        name = safe_getattr(node, 'Name', '<group>')
        log(f"  [grp] {'  '*depth}{name}")

        # Try child variables
        for cont_name in ('Variables', 'Items', 'Children'):
            cont = safe_getattr(node, cont_name, None)
            if cont is None:
                continue
            for i, it in _enum_collection(cont, f"{name}.{cont_name}"):
                it_name = safe_getattr(it, 'Name', f'<{cont_name}_{i}>')
                val = safe_getattr(it, 'ValueConverted', None)
                log(f"    [var] {'  '*depth}{it_name} = {val}")
                printed += 1
                if printed >= max_print:
                    return

        # Try sub-groups
        for grp_name in ('Groups', 'Children'):
            groups = safe_getattr(node, grp_name, None)
            if groups is None:
                continue
            for i, child in _enum_collection(groups, f"{name}.{grp_name}"):
                visit(child, depth + 1)

    visit(root, 0)


def main():
    try:
        app = try_connect()
        log("[cd] Connected to ControlDesk application")

        exp = app.ActiveExperiment
        if not exp:
            log("[cd] No ActiveExperiment; please open one in ControlDesk")
            return 1
        log(f"[cd] ActiveExperiment: {safe_getattr(exp, 'Name', 'Unknown')}")

        # Try to go online and start measurement to ensure variables are populated
        try:
            app.CalibrationManagement.StartOnlineCalibration()
            app.MeasurementDataManagement.Start()
            log("[cd] Online calibration + measurement started")
        except Exception as e:
            log(f"[cd] Could not start online/measurement: {e}")

        plats = safe_getattr(exp, "Platforms", None)
        if plats is None:
            log("[cd] No Platforms collection available")
            return 1

        # Iterate platforms until one yields variables
        def try_platform(plat):
            plat_name = safe_getattr(plat, "Name", "<Platform>")
            log(f"[cd] Using platform: {plat_name}")

            vdesc = get_prop(plat, "ActiveVariableDescription", None)
            vars_obj = get_prop(vdesc, "Variables", None) if vdesc is not None else None

            # If no active variables, try to select a variable description
            if vars_obj is None:
                vdescs = safe_getattr(plat, "VariableDescriptions", None)
                if vdescs is None:
                    log("[cd] No VariableDescriptions available; cannot enumerate variables")
                    dump_com_object("Platform object", plat)
                    return None, None, plat

                # Try Count/Item to pick one
                picked = None
                count = safe_getattr(vdescs, "Count", None)
                if isinstance(count, int) and count > 0:
                    for i in range(count):
                        try:
                            cand = vdescs.Item(i)
                            name = safe_getattr(cand, "Name", f"VD_{i}")
                            log(f"[cd] Found VariableDescription[{i}]: {name}")
                            if picked is None:
                                picked = cand
                        except Exception:
                            continue
                else:
                    try:
                        for i, cand in enumerate(vdescs):
                            name = safe_getattr(cand, "Name", f"VD_{i}")
                            log(f"[cd] Found VariableDescription[{i}]: {name}")
                            if picked is None:
                                picked = cand
                    except Exception:
                        pass

                if picked is None:
                    log("[cd] Could not pick a VariableDescription")
                    return None, None, plat

                # Activate it (try multiple APIs)
                activated = False
                last_err = None
                try:
                    if hasattr(vdescs, 'Activate'):
                        vdescs.Activate(picked)
                        activated = True
                except Exception as e:
                    last_err = e
                if not activated:
                    try:
                        if hasattr(picked, 'Activate'):
                            picked.Activate()
                            activated = True
                    except Exception as e:
                        last_err = e
                if not activated:
                    try:
                        plat.ActiveVariableDescription = picked
                        activated = True
                    except Exception as e:
                        last_err = e

                if not activated:
                    log(f"[cd] Failed to activate VariableDescription via known methods: {last_err}")
                    return None, None, plat

                vdesc = get_prop(plat, "ActiveVariableDescription", picked)
                # Try to force-load content
                try:
                    if hasattr(vdesc, 'Reload'):
                        vdesc.Reload()
                except Exception:
                    pass
                try:
                    if hasattr(vdesc, 'CheckSourceForChanges'):
                        vdesc.CheckSourceForChanges()
                except Exception:
                    pass
                vars_obj = get_prop(vdesc, "Variables", None)
                log(f"[cd] Activated VariableDescription: {safe_getattr(vdesc, 'Name', 'Unknown')}")
                if vars_obj is None:
                    dump_com_object("ActiveVariableDescription", vdesc)

            return vdesc, vars_obj, plat

        # Try each platform
        plat_list = []
        try:
            count = safe_getattr(plats, 'Count', None)
            if isinstance(count, int) and count > 0:
                plat_list = [plats.Item(i) for i in range(count)]
            else:
                plat_list = list(plats)
        except Exception:
            try:
                plat_list = [plats[0]]
            except Exception:
                plat_list = []

        vdesc = None
        vars_obj = None
        plat_used = None
        for plat in plat_list:
            vdesc, vars_obj, plat_used = try_platform(plat)
            if vars_obj is not None:
                break

        if vars_obj is None:
            log("[cd] Could not access any variable container on available platforms")
            # Try path-based access on the last platform examined
            if plat_used is None and plat_list:
                plat_used = plat_list[0]
            if plat_used is None:
                return 1
            vdesc_try = get_prop(plat_used, "ActiveVariableDescription", None)
            vars_try = get_prop(vdesc_try, "Variables", None)
            if vars_try is None:
                # Try RootGroup traversal
                root = get_prop(vdesc_try, 'RootGroup', None) if vdesc_try else None
                if root is not None:
                    traverse_root_group(root)
                else:
                    log("[cd] No RootGroup available to traverse")
                dump_com_object("Platform object (failure path)", plat_used)
                if vdesc_try:
                    dump_com_object("ActiveVariableDescription (failure path)", vdesc_try)
                log("[cd] Variables container unavailable; cannot test path-based access")
                return 1
            any_ok = False
            test_paths = [
                "Platform()://Model Root/BatteryVoltage[V]/Value",
                "Platform()://Model Root/Vehicles/Ego/Throttle/Value",
                "Platform()://Model Root/Vehicles/F1/Throttle/Value",
                "Platform()://Model Root/Vehicles/F1/Steering/Value",
                "Platform()://Model Root/Vehicles/F1/Brake/Value",
            ]
            for p in test_paths:
                try:
                    val = vars_try[p].ValueConverted
                    log(f"[cd] READ OK: {p} = {val}")
                    any_ok = True
                except Exception as e:
                    log(f"[cd] READ FAIL: {p}  ({e})")
            return 0 if any_ok else 1
            if vdescs is None:
                log("[cd] No VariableDescriptions available; cannot enumerate variables")
                return 1

            # Try Count/Item to pick one
            picked = None
            count = safe_getattr(vdescs, "Count", None)
            if isinstance(count, int) and count > 0:
                for i in range(count):
                    try:
                        cand = vdescs.Item(i)
                        name = safe_getattr(cand, "Name", f"VD_{i}")
                        log(f"[cd] Found VariableDescription[{i}]: {name}")
                        if picked is None:
                            picked = cand
                    except Exception:
                        continue
            else:
                try:
                    for i, cand in enumerate(vdescs):
                        name = safe_getattr(cand, "Name", f"VD_{i}")
                        log(f"[cd] Found VariableDescription[{i}]: {name}")
                        if picked is None:
                            picked = cand
                except Exception:
                    pass

            if picked is None:
                log("[cd] Could not pick a VariableDescription")
                return 1

            # Activate it (try multiple APIs)
            activated = False
            last_err = None
            try:
                # Some APIs expose 'Activate' on the collection
                if hasattr(vdescs, 'Activate'):
                    vdescs.Activate(picked)
                    activated = True
            except Exception as e:
                last_err = e
            if not activated:
                try:
                    # Some expose 'Activate' on the item itself
                    if hasattr(picked, 'Activate'):
                        picked.Activate()
                        activated = True
                except Exception as e:
                    last_err = e
            if not activated:
                try:
                    # Fallback: assign if property is settable
                    plat0.ActiveVariableDescription = picked
                    activated = True
                except Exception as e:
                    last_err = e

            if not activated:
                log(f"[cd] Failed to activate VariableDescription via known methods: {last_err}")
                return 1

            vdesc = safe_getattr(plat0, "ActiveVariableDescription", picked)
            vars_obj = safe_getattr(vdesc, "Variables", None)
            log(f"[cd] Activated VariableDescription: {safe_getattr(vdesc, 'Name', 'Unknown')}")

        if vars_obj is None:
            log("[cd] vdesc.Variables still not available after activation; probing alternates...")
            candidates = [
                "Variables", "Variables2", "Channels", "Measures", "Parameters",
                "Signals", "Items", "Children", "Groups", "Measurements"
            ]
            for name in candidates:
                obj = safe_getattr(vdesc, name, None)
                if obj is not None:
                    log(f"[cd] Trying container '{name}'")
                    vars_obj = obj
                    break
        if vars_obj is None:
            log("[cd] No variable container found on ActiveVariableDescription")
            # As a fallback, try direct path-based access on Platforms[0].ActiveVariableDescription.Variables
            test_paths = [
                "Platform()://Model Root/BatteryVoltage[V]/Value",
                "Platform()://Model Root/Vehicles/Ego/Throttle/Value",
                "Platform()://Model Root/Vehicles/F1/Throttle/Value",
                "Platform()://Model Root/Vehicles/F1/Steering/Value",
                "Platform()://Model Root/Vehicles/F1/Brake/Value",
            ]
            vdesc_try = safe_getattr(plat0, "ActiveVariableDescription", None)
            vars_try = safe_getattr(vdesc_try, "Variables", None)
            if vars_try is None:
                log("[cd] Variables container unavailable; cannot test path-based access")
                return 1
            any_ok = False
            for p in test_paths:
                try:
                    val = vars_try[p].ValueConverted
                    log(f"[cd] READ OK: {p} = {val}")
                    any_ok = True
                except Exception as e:
                    log(f"[cd] READ FAIL: {p}  ({e})")
            return 0 if any_ok else 1

        log("[cd] Enumerating variables (up to 500 items)...")
        found = []
        for idx, name, item in enumerate_variables(vars_obj, max_items=500):
            # Try to read a converted value if available and item supports it
            value_preview = None
            try:
                value_preview = safe_getattr(item, "ValueConverted", None)
            except Exception:
                value_preview = None
            log(f"  [{idx}] {name}  value={value_preview}")
            found.append(name)

        if not found:
            log("[cd] No variables enumerated. Try switching the Variable Description or different ProgID.")
        else:
            # Heuristic: filter likely vehicle-related signals
            wanted = [n for n in found if any(k in n.lower() for k in ("throttle", "steer", "brake", "velocity", "speed"))]
            if wanted:
                log("[cd] Candidate control variable names:")
                for n in wanted:
                    log(f"    - {n}")

        return 0

    except Exception as e:
        log(f"[cd] Probe error: {e}")
        traceback.print_exc()
        return 2


if __name__ == "__main__":
    sys.exit(main())


