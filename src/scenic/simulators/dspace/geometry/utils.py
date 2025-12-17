"""COM helper utilities for dSPACE ModelDesk."""

# Import projection functions for use by route_mapping
from .projection import find_road_id_for_position

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


def get_road_name_for_id(road_index, road_id):
    """Get road name from road ID in road_index.
    
    Args:
        road_index: Road index dict with 'roads' key
        road_id: Road ID (RD ID, typically 0, 1, 2)
        
    Returns:
        Road name string or None if not found
    """
    try:
        if not road_index:
            return None
        roads = road_index.get('roads', {})
        for road_name, road_data in roads.items():
            if road_data.get('id') == road_id:
                return road_name
        return None
    except Exception:
        return None


def map_rd_to_xodr_road_id(road_index, rd_road_id):
    """Map RD road ID to XODR road ID.
    
    Maps RD road IDs (0, 1, 2) to XODR road IDs based on road names.
    Known mapping for Laguna Seca:
    - RD ID 0 (The Corkscrew1) -> XODR ID "2117817291"
    - RD ID 1 (Pit Lane1_2) -> XODR ID "1545702203"
    - RD ID 2 (Andretti Hairpin1_3) -> XODR ID "1776499453"
    
    Args:
        road_index: Road index dict
        rd_road_id: RD road ID (typically 0, 1, 2)
        
    Returns:
        XODR road ID string or None if mapping not available
    """
    # Known mapping for Laguna Seca
    RD_TO_XODR_MAPPING = {
        0: '2117817291',  # The Corkscrew1
        1: '1545702203',  # Pit Lane1_2
        2: '1776499453',  # Andretti Hairpin1_3
    }
    
    if isinstance(rd_road_id, int) and rd_road_id in RD_TO_XODR_MAPPING:
        return RD_TO_XODR_MAPPING[rd_road_id]
    
    # Fallback: Try to get road name and map it
    try:
        road_name = get_road_name_for_id(road_index, rd_road_id)
        if road_name:
            # Map road name to XODR ID
            NAME_TO_XODR_MAPPING = {
                'The Corkscrew1': '2117817291',
                'Pit Lane1_2': '1545702203',
                'Andretti Hairpin1_3': '1776499453',
            }
            if road_name in NAME_TO_XODR_MAPPING:
                return NAME_TO_XODR_MAPPING[road_name]
    except Exception:
        pass
    
    return None

