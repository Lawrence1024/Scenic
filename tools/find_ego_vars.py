#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Find ego position, orientation, and velocity variables in ControlDesk."""

import sys
import pythoncom
from win32com.client import Dispatch

def safe_get(obj, name, default=None):
    try:
        return getattr(obj, name)
    except Exception:
        return default

def coll_to_list(coll):
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

print("Connecting to ControlDesk...", flush=True)
pythoncom.CoInitialize()
app = Dispatch("ControlDeskNG.Application")
print("Connected!", flush=True)

exp = app.ActiveExperiment
if not exp:
    print("ERROR: No ActiveExperiment", flush=True)
    sys.exit(1)

print(f"Experiment: {exp.Name}", flush=True)

plats = exp.Platforms
top = coll_to_list(plats)
print(f"Top platforms: {[safe_get(p, 'Name', '?') for p in top]}", flush=True)

nested = []
for p in top:
    inner_plats = safe_get(p, "Platforms", None)
    if inner_plats:
        inner = coll_to_list(inner_plats)
        for ip in inner:
            nested.append((ip, f"{safe_get(p, 'Name', '?')}/{safe_get(ip, 'Name', '?')}"))

print(f"Found {len(nested)} nested platforms", flush=True)

# Search terms for ego state
search_terms = ["ego", "position", "orientation", "velocity", "x", "y", "z", 
                "yaw", "pitch", "roll", "heading", "pose", "pos", "vel", 
                "linvel", "angvel", "location", "rotation"]

for plat_obj, disp_name in nested:
    print(f"\n{'='*70}", flush=True)
    print(f"Platform: {disp_name}", flush=True)
    print(f"{'='*70}", flush=True)
    
    vdesc = safe_get(plat_obj, "ActiveVariableDescription", None)
    if not vdesc:
        print("  No ActiveVariableDescription", flush=True)
        continue
    
    vars_obj = safe_get(vdesc, "Variables", None)
    if not vars_obj:
        print("  No Variables collection", flush=True)
        continue
    
    count = safe_get(vars_obj, "Count", None)
    print(f"  Variables count: {count}", flush=True)
    
    # Try to get RootGroup
    root = safe_get(vdesc, "RootGroup", None)
    if not root:
        print("  No RootGroup - searching flat list", flush=True)
        # Search flat
        try:
            cnt = safe_get(vars_obj, "Count", None)
            iterator = (vars_obj.Item(i) for i in range(min(cnt, 100))) if isinstance(cnt, int) else iter(vars_obj)
            found = 0
            for item in iterator:
                if found >= 50:
                    break
                name = safe_get(item, "Name", None)
                if not name:
                    continue
                name_lower = name.lower()
                if any(term in name_lower for term in search_terms):
                    value = safe_get(item, "ValueConverted", safe_get(item, "Value", "?"))
                    # Try to get path
                    path = safe_get(item, "Path", safe_get(item, "FullPath", safe_get(item, "QualifiedName", name)))
                    print(f"    {name} = {value}", flush=True)
                    if path and path != name:
                        print(f"      Path: {path}", flush=True)
                    found += 1
        except Exception as e:
            print(f"  Error searching: {e}", flush=True)
        continue
    
    # Search in hierarchy
    def search_node(node, path="", depth=0, max_depth=6):
        if depth > max_depth:
            return 0
        found = 0
        
        node_name = safe_get(node, "Name", "<unnamed>")
        current_path = f"{path}/{node_name}" if path else node_name
        
        # Check variables at this node
        for cont_name in ("Variables", "Items", "Children"):
            cont = safe_get(node, cont_name, None)
            if not cont:
                continue
            items = []
            try:
                cnt = safe_get(cont, "Count", None)
                if isinstance(cnt, int) and cnt > 0:
                    items = [cont.Item(i) for i in range(min(cnt, 50))]
            except Exception:
                pass
            
            for item in items:
                if found >= 50:
                    return found
                name = safe_get(item, "Name", None)
                if not name:
                    continue
                name_lower = name.lower()
                if any(term in name_lower for term in search_terms):
                    value = safe_get(item, "ValueConverted", safe_get(item, "Value", "?"))
                    path_str = safe_get(item, "Path", safe_get(item, "FullPath", safe_get(item, "QualifiedName", f"{current_path}/{name}")))
                    indent = "  " * (depth + 1)
                    print(f"{indent}📁 {current_path}/", flush=True)
                    print(f"{indent}  🔹 {name} = {value}", flush=True)
                    print(f"{indent}     Path: {path_str}", flush=True)
                    found += 1
        
        # Recurse into groups
        for grp_name in ("Groups", "Children"):
            groups = safe_get(node, grp_name, None)
            if groups:
                for child in coll_to_list(groups):
                    found += search_node(child, current_path, depth + 1, max_depth)
                    if found >= 50:
                        return found
        
        return found
    
    print("  Searching hierarchy...", flush=True)
    found_count = search_node(root, max_depth=6)
    print(f"  Found {found_count} matching variables", flush=True)

print("\nDone!", flush=True)


