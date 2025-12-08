# -*- coding: utf-8 -*-
"""Explore ControlDesk variable structure via COM connection.

This script connects to ControlDesk and displays the complete variable
structure hierarchy, helping you understand how variables are organized
and what paths to use for accessing them.

Usage:
  python tools/explore_controldesk_structure.py
  python tools/explore_controldesk_structure.py --platform Platform_2
  python tools/explore_controldesk_structure.py --max-depth 5 --max-vars 100
  python tools/explore_controldesk_structure.py --search throttle brake steering
  python tools/explore_controldesk_structure.py --show-types
"""

from __future__ import annotations

import argparse
import sys
from typing import List, Optional, Sequence, Set


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
    """Connect to ControlDesk via COM."""
    import pythoncom
    from win32com.client import Dispatch
    
    pythoncom.CoInitialize()
    return Dispatch(prog_id)


def safe_get(obj, name: str, default=None):
    """Safely get an attribute from a COM object."""
    try:
        return getattr(obj, name)
    except Exception:
        return default


def get_prop(obj, name: str, default=None):
    """Get a property from a COM object, handling callable properties."""
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
    """Convert a COM collection to a Python list."""
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


# ----------------------------- Structure Exploration -----------------------

def explore_variable_structure(
    plat_obj,
    plat_name: str,
    max_depth: int = 4,
    max_vars_per_level: int = 50,
    search_terms: Optional[List[str]] = None,
    show_types: bool = False,
    visited: Optional[Set[str]] = None,
    show_paths: bool = False
):
    """Explore and display the variable structure hierarchy.
    
    Args:
        plat_obj: Platform COM object
        plat_name: Platform display name
        max_depth: Maximum depth to traverse
        max_vars_per_level: Maximum variables to show per level
        search_terms: Optional list of terms to search for
        show_types: Whether to show variable types
        visited: Set of already visited paths (for cycle detection)
    """
    if visited is None:
        visited = set()
    
    info(f"\n{'='*70}")
    info(f"Exploring Variable Structure: {plat_name}")
    info(f"{'='*70}")
    
    # Get ActiveVariableDescription
    vdesc = get_prop(plat_obj, "ActiveVariableDescription", None)
    if vdesc is None:
        warn(f"{plat_name}: No ActiveVariableDescription")
        return
    
    # Try to refresh variable description
    try:
        if hasattr(vdesc, "Reload"):
            vdesc.Reload()
        if hasattr(vdesc, "CheckSourceForChanges"):
            vdesc.CheckSourceForChanges()
    except Exception:
        pass
    
    # Get Variables collection
    vars_obj = get_prop(vdesc, "Variables", None)
    if vars_obj is None:
        warn(f"{plat_name}: Variables collection is None")
        info("This may mean the platform is not online or variables are not loaded")
        return
    
    count = safe_get(vars_obj, "Count", None)
    info(f"Variables collection found: {count if isinstance(count, int) else 'unknown'} items")
    
    # Get RootGroup for hierarchical exploration
    root = get_prop(vdesc, "RootGroup", None)
    if root is None:
        warn(f"{plat_name}: No RootGroup available")
        info("Will try to list variables directly from Variables collection")
        list_variables_flat(vars_obj, max_vars_per_level, search_terms, show_types)
        return
    
    info("\nTraversing RootGroup hierarchy...")
    traverse_group(
        root,
        prefix="",
        depth=max_depth,
        max_vars=max_vars_per_level,
        search_terms=[t.lower() for t in search_terms] if search_terms else None,
        show_types=show_types,
        visited=visited,
        show_paths=show_paths
    )


def traverse_group(
    node,
    prefix: str,
    depth: int,
    max_vars: int,
    search_terms: Optional[List[str]],
    show_types: bool,
    visited: Set[str],
    vars_shown: Optional[List[int]] = None,
    show_paths: bool = False
):
    """Recursively traverse a group in the variable hierarchy.
    
    Args:
        node: Current group node
        prefix: Display prefix for indentation
        depth: Remaining depth to traverse
        max_vars: Maximum variables to display
        search_terms: Optional search filter terms
        show_types: Whether to show variable types
        visited: Set of visited paths
        vars_shown: Counter for displayed variables [count]
    """
    if vars_shown is None:
        vars_shown = [0]
    
    if depth < 0 or vars_shown[0] >= max_vars:
        return
    
    node_name = safe_get(node, "Name", "<unnamed>")
    node_path = f"{prefix}/{node_name}"
    
    # Cycle detection
    if node_path in visited:
        return
    visited.add(node_path)
    
    # Print group header
    indent = "  " * (len(prefix.split('/')) - 1 if prefix else 0)
    print(f"\n{indent}📁 {node_name}/")
    
    # Check for Variables/Items/Children at this level
    for container_name in ("Variables", "Items", "Children"):
        container = safe_get(node, container_name, None)
        if container is None:
            continue
        
        items = []
        try:
            cnt = safe_get(container, "Count", None)
            if isinstance(cnt, int) and cnt > 0:
                items = [container.Item(i) for i in range(min(cnt, max_vars - vars_shown[0]))]
            else:
                for i, item in enumerate(container):
                    if i >= max_vars - vars_shown[0]:
                        break
                    items.append(item)
        except Exception:
            pass
        
        # Display variables at this level
        for item in items:
            if vars_shown[0] >= max_vars:
                break
            
            var_name = safe_get(item, "Name", None)
            if not isinstance(var_name, str):
                continue
            
            # Apply search filter if specified
            if search_terms and not any(term in var_name.lower() for term in search_terms):
                continue
            
            # Get variable value
            value = safe_get(item, "ValueConverted", safe_get(item, "Value", "<no value>"))
            
            # Get variable type if requested
            type_info = ""
            if show_types:
                var_type = safe_get(item, "Type", None)
                if var_type:
                    type_name = safe_get(var_type, "Name", "unknown")
                    type_info = f" [{type_name}]"
            
            # Check if writable
            writable = ""
            try:
                # Try to detect write capability
                if hasattr(item, "ValueConverted") or hasattr(item, "Value"):
                    writable = " (writable)"
            except Exception:
                pass
            
            # Display the variable
            var_indent = indent + "  "
            
            # Build full path if requested
            full_path = ""
            if show_paths:
                # Try to get the actual path from the variable object
                path_str = None
                for path_attr in ["Path", "FullPath", "QualifiedName", "Name", "Key"]:
                    try:
                        path_val = safe_get(item, path_attr, None)
                        if path_val and isinstance(path_val, str) and "Platform()://" in path_val:
                            path_str = path_val
                            break
                    except Exception:
                        pass
                
                # If we didn't find a direct path property, construct it
                if not path_str:
                    path_parts = [node_path, var_name] if node_path else [var_name]
                    path_parts = [p for p in path_parts if p and p != "/"]
                    if path_parts:
                        # Construct a ControlDesk-style path
                        path_str = f"Platform()://ASM_Traffic/{'/'.join(path_parts)}"
                
                if path_str:
                    full_path = f"  Path: {path_str}"
            
            print(f"{var_indent}🔹 {var_name}{type_info} = {value}{writable}")
            if full_path:
                print(f"{var_indent}   {full_path}")
            vars_shown[0] += 1
            
            # Check if this item has child groups
            child_groups = safe_get(item, "Groups", None)
            if child_groups is not None and depth > 0:
                for child in coll_to_list(child_groups):
                    traverse_group(
                        child,
                        prefix=node_path,
                        depth=depth - 1,
                        max_vars=max_vars,
                        search_terms=search_terms,
                        show_types=show_types,
                        visited=visited,
                        vars_shown=vars_shown,
                        show_paths=show_paths
                    )
    
    # Traverse child groups
    for group_name in ("Groups", "Children"):
        groups = safe_get(node, group_name, None)
        if groups is None or depth <= 0:
            continue
        
        for child in coll_to_list(groups):
            if vars_shown[0] >= max_vars:
                break
            traverse_group(
                child,
                prefix=node_path,
                depth=depth - 1,
                max_vars=max_vars,
                search_terms=search_terms,
                show_types=show_types,
                visited=visited,
                vars_shown=vars_shown,
                show_paths=show_paths
            )


def list_variables_flat(
    vars_obj,
    max_vars: int,
    search_terms: Optional[List[str]],
    show_types: bool
):
    """List variables from Variables collection (flat, no hierarchy).
    
    Args:
        vars_obj: Variables COM collection
        max_vars: Maximum variables to display
        search_terms: Optional search filter terms
        show_types: Whether to show variable types
    """
    info("\nListing variables (flat structure):")
    
    search_lower = [t.lower() for t in search_terms] if search_terms else None
    shown = 0
    
    try:
        count = safe_get(vars_obj, "Count", None)
        iterator = (vars_obj.Item(i) for i in range(count)) if isinstance(count, int) else iter(vars_obj)
        
        for item in iterator:
            if shown >= max_vars:
                info(f"\n... (showing first {max_vars} variables, use --max-vars to see more)")
                break
            
            var_name = safe_get(item, "Name", None)
            if not isinstance(var_name, str):
                continue
            
            # Apply search filter
            if search_lower and not any(term in var_name.lower() for term in search_lower):
                continue
            
            value = safe_get(item, "ValueConverted", safe_get(item, "Value", "<no value>"))
            
            type_info = ""
            if show_types:
                var_type = safe_get(item, "Type", None)
                if var_type:
                    type_name = safe_get(var_type, "Name", "unknown")
                    type_info = f" [{type_name}]"
            
            print(f"  {var_name}{type_info} = {value}")
            shown += 1
            
    except Exception as e:
        err(f"Error listing variables: {e}")


def list_platforms(app):
    """List all available platforms.
    
    Returns:
        Tuple of (top_level_platforms, nested_platforms)
    """
    exp = get_prop(app, "ActiveExperiment", None)
    if exp is None:
        err("No ActiveExperiment. Open an experiment in ControlDesk.")
        return [], []
    
    plats = get_prop(exp, "Platforms", None)
    if plats is None:
        err("ActiveExperiment has no Platforms.")
        return [], []
    
    top = coll_to_list(plats)
    top_names = [safe_get(p, "Name", "<Platform>") for p in top]
    info(f"Top-level Platforms: {', '.join(top_names) if top_names else '<none>'}")
    
    nested = []
    for p in top:
        pname = safe_get(p, "Name", "<Platform>")
        inner_plats = safe_get(p, "Platforms", None)
        if inner_plats is None:
            continue
        inner = coll_to_list(inner_plats)
        inner_names = [safe_get(ip, "Name", "<Inner>") for ip in inner]
        if inner_names:
            info(f"Nested platforms under '{pname}': {', '.join(inner_names)}")
        for ip in inner:
            iname = safe_get(ip, "Name", "<Inner>")
            nested.append((ip, f"{pname}/{iname}"))
    
    return top, nested


def start_online_calibration(app):
    """Start online calibration and measurement."""
    try:
        info("Starting online calibration...")
        app.CalibrationManagement.StartOnlineCalibration()
        info("Starting measurement...")
        app.MeasurementDataManagement.Start()
        info("✓ Online calibration and measurement started")
        return True
    except Exception as e:
        warn(f"Could not start online calibration/measurement: {e}")
        return False


# ----------------------------- Main ----------------------------------------

def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Explore ControlDesk variable structure via COM",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Explore all platforms with default settings
  python tools/explore_controldesk_structure.py
  
  # Explore specific platform
  python tools/explore_controldesk_structure.py --platform Platform_2
  
  # Search for specific variables
  python tools/explore_controldesk_structure.py --search throttle brake steering
  
  # Show more depth and variables
  python tools/explore_controldesk_structure.py --max-depth 6 --max-vars 200
  
  # Show variable types
  python tools/explore_controldesk_structure.py --show-types
        """
    )
    
    parser.add_argument(
        "--prog-id",
        default="ControlDeskNG.Application",
        help="COM ProgID for ControlDesk (default: ControlDeskNG.Application)"
    )
    parser.add_argument(
        "--platform",
        type=str,
        default=None,
        help="Specific platform name to explore (e.g., Platform_2). If not specified, explores all."
    )
    parser.add_argument(
        "--max-depth",
        type=int,
        default=4,
        help="Maximum depth to traverse in hierarchy (default: 4)"
    )
    parser.add_argument(
        "--max-vars",
        type=int,
        default=100,
        help="Maximum variables to display (default: 100)"
    )
    parser.add_argument(
        "--search",
        nargs="*",
        default=None,
        help="Filter variables containing these terms (case-insensitive)"
    )
    parser.add_argument(
        "--show-types",
        action="store_true",
        help="Show variable types in output"
    )
    parser.add_argument(
        "--no-start-online",
        action="store_true",
        help="Don't attempt to start online calibration (useful if already running)"
    )
    parser.add_argument(
        "--ego-state",
        action="store_true",
        help="Search for ego vehicle state variables (position, orientation, velocity)"
    )
    parser.add_argument(
        "--show-paths",
        action="store_true",
        help="Show full variable paths (Platform()://... format)"
    )
    
    args = parser.parse_args(argv)
    
    # If --ego-state is specified, add ego-related search terms
    if args.ego_state:
        ego_terms = ["ego", "position", "orientation", "velocity", "x", "y", "z", 
                     "yaw", "pitch", "roll", "heading", "pose", "pos", "vel", 
                     "linvel", "angvel", "location", "rotation", "quaternion"]
        if args.search:
            args.search.extend(ego_terms)
        else:
            args.search = ego_terms
        print("[INFO] Ego state mode: searching for position, orientation, and velocity variables", flush=True)
        info("Ego state mode: searching for position, orientation, and velocity variables")
    
    try:
        # Connect to ControlDesk
        print(f"[INFO] Connecting to ControlDesk (ProgID: {args.prog_id})...", flush=True)
        info(f"Connecting to ControlDesk (ProgID: {args.prog_id})...")
        app = com_connect(args.prog_id)
        print("[INFO] ✓ Connected to ControlDesk", flush=True)
        info("✓ Connected to ControlDesk")
        
        # Check for active experiment
        exp = get_prop(app, "ActiveExperiment", None)
        if exp is None:
            err("No ActiveExperiment found")
            err("Please open an experiment in ControlDesk and try again")
            return 1
        
        exp_name = safe_get(exp, "Name", "Unknown")
        info(f"✓ Active experiment: {exp_name}")
        
        # Start online calibration if needed
        if not args.no_start_online:
            start_online_calibration(app)
        
        # List platforms
        top, nested = list_platforms(app)
        
        if not nested:
            warn("No nested platforms found")
            info("This may indicate the experiment is not fully loaded")
            return 1
        
        # Determine which platforms to explore
        platforms_to_explore = []
        if args.platform:
            # Find specific platform
            for plat_obj, disp_name in nested:
                if args.platform in disp_name or disp_name.endswith(f"/{args.platform}"):
                    platforms_to_explore.append((plat_obj, disp_name))
            if not platforms_to_explore:
                err(f"Platform '{args.platform}' not found")
                err(f"Available platforms: {', '.join([d for _, d in nested])}")
                return 1
        else:
            # Explore all platforms
            platforms_to_explore = nested
        
        # Explore each platform
        for plat_obj, disp_name in platforms_to_explore:
            explore_variable_structure(
                plat_obj,
                disp_name,
                max_depth=args.max_depth,
                max_vars_per_level=args.max_vars,
                search_terms=args.search,
                show_types=args.show_types,
                show_paths=args.show_paths
            )
        
        info("\n" + "="*70)
        info("Exploration complete!")
        info("="*70)
        
        return 0
        
    except Exception as e:
        err(f"Fatal error: {e}")
        import traceback
        traceback.print_exc()
        return 2


if __name__ == "__main__":
    sys.exit(main())



