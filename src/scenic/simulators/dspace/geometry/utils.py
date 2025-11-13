"""COM helper utilities for dSPACE ModelDesk."""

def _count_any(coll):
    try:
        return int(getattr(coll, "Count", len(coll)))
    except Exception:
        return 0

def clear_collection(coll):
    n = _count_any(coll)
    for i in reversed(range(n)):
        for m in ("Remove", "Delete", "RemoveAt"):
            if hasattr(coll, m):
                try:
                    getattr(coll, m)(i)
                    break
                except Exception:
                    pass

def ensure_two_segments(sequence):
    segs = sequence.Segments
    while _count_any(segs) < 2:
        if hasattr(segs, "Add"):
            segs.Add()
        else:
            raise RuntimeError("Segments.Add() missing; please pre-create 2 segs in UI.")
    return segs

def activate_type(typed_obj, element_name: str) -> bool:
    try:
        typed_obj.Activate(element_name); return True
    except Exception:
        pass
    avail = getattr(typed_obj, "AvailableElements", None)
    if avail:
        for el in avail:
            if str(el).lower() == element_name.lower():
                typed_obj.Activate(el); return True
    return False

def set_activity_constant(typed_obj, value: float):
    # typed_obj → ActiveElement → SourceType → ActiveElement → Constant
    tgt = typed_obj.ActiveElement
    tgt = tgt.SourceType
    tgt = tgt.ActiveElement
    tgt.Constant = float(value)

def make_endless_transition(segs):
    try:
        conds = segs[1].Transition.Conditions
        for i in reversed(range(_count_any(conds))):
            try: conds.Remove(i)
            except Exception: pass
        conds.Add("Endless")
    except Exception:
        pass

def make_endless_transition_segment(segment):
    """Set up endless transition for a single segment."""
    try:
        tr = segment.Transition
        if hasattr(tr, 'Conditions'):
            conds = tr.Conditions
            # Clear existing conditions
            while conds.Count > 0:
                try:
                    conds.Remove(0)
                except:
                    break
            # Add Endless condition
            if hasattr(conds, 'Add'):
                conds.Add("Endless")
    except Exception as e:
        print(f"    Warning: Could not set up endless transition: {e}")

def configure_seg0_absolute_pose(segs, *, s: float, t: float):
    # Longitudinal: "Position" (absolute along reference line)
    lt0 = segs[0].Activity.LongitudinalType
    if not activate_type(lt0, "Position"):
        # some setups name it "DistancePosition" or similar; try fallback
        if not activate_type(lt0, "DistanceMeter"):
            raise RuntimeError("seg0.LongitudinalType 'Position' not available.")
    set_activity_constant(lt0, s)

    # Lateral: "Deviation" with DependencyType "Absolute" (not Relative)
    lat0 = segs[0].Activity.LateralType
    activate_type(lat0, "Deviation")
    dep = getattr(lat0.ActiveElement, "DependencyType", None)
    if dep is not None:
        # Prefer Absolute; fall back to 'Road' if that's how your MD version names it
        if not activate_type(dep, "Absolute"):
            activate_type(dep, "Road")
    set_activity_constant(lat0, t)

def configure_seg1_motion(segs, *, v: float, t: float):
    """Configure segment 1 with Velocity (longitudinal) and Lateral deviation (lateral),
    both set to SourceType='Extern' for external control.
    
    This enables ControlDesk External Signals to control fellow vehicles at runtime.
    The v and t parameters are kept for backward compatibility but are ignored when
    Type='Extern' (external control takes precedence).
    
    Args:
        segs: Segments collection (segs[1] will be configured)
        v: Initial velocity value (ignored when Type='Extern', kept for compatibility)
        t: Initial lateral deviation value (ignored when Type='Extern', kept for compatibility)
    """
    # Longitudinal: Velocity with SourceType='Extern'
    lt1 = segs[1].Activity.LongitudinalType
    if not activate_type(lt1, "Velocity"):
        activate_type(lt1, "Speed")
    
    # Set SourceType to "Extern" for longitudinal (this sets Type='Extern' in the UI)
    long_elem = lt1.ActiveElement
    if hasattr(long_elem, "SourceType"):
        source_type = long_elem.SourceType
        if hasattr(source_type, "Activate"):
            if not activate_type(source_type, "Extern"):
                activate_type(source_type, "External")  # Try alternative spelling
    
    # Lateral: Lateral deviation (or Deviation) with SourceType='Extern'
    lat1 = segs[1].Activity.LateralType
    lateral_activated = False
    if activate_type(lat1, "Lateral deviation"):
        lateral_activated = True
    elif activate_type(lat1, "Deviation"):
        lateral_activated = True
    
    if not lateral_activated:
        # Fallback to Continue if Deviation not available (but can't set Extern on Continue)
        activate_type(lat1, "Continue")
        return  # Early return - Continue doesn't support external control
    
    # Set SourceType to "Extern" for lateral (this sets Type='Extern' in the UI)
    lat_elem = lat1.ActiveElement
    if hasattr(lat_elem, "SourceType"):
        source_type = lat_elem.SourceType
        if hasattr(source_type, "Activate"):
            if not activate_type(source_type, "Extern"):
                activate_type(source_type, "External")  # Try alternative spelling


# Main road names for Laguna Seca (consistent across XODR and RD paths)
MAIN_ROAD_NAMES = ['The Corkscrew1', 'Pit Lane1_2', 'Andretti Hairpin1_3']

