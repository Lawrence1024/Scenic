"""Vehicle placement in ModelDesk scenario."""

from ..geometry import (
    project_world_to_st,
    configure_seg0_absolute_pose,
    configure_seg1_motion,
    make_endless_transition,
    clear_collection,
    ensure_two_segments,
)


def project_scenic_to_st(obj, road_index, coordinate_transform=None):
    """Project Scenic object position to road (s,t) coordinates.
    coordinate_transform is ignored (map is single source).
    """
    if not getattr(obj, "position", None):
        return 0.0, 0.0
    work_x, work_y = obj.position.x, obj.position.y
    # Use road index for proper geometric projection
    if road_index:
        s_val, t_val = project_world_to_st(road_index, (work_x, work_y))
        print(f"  World coordinates ({work_x:.3f}, {work_y:.3f}) -> Road coordinates (s={s_val:.1f}, t={t_val:.3f})")
        return s_val, t_val
    else:
        return 0.0, 0.0


def create_fellow_vehicle(ts, obj, road_index, coordinate_transform, fellow_storage):
    """Create a Fellow vehicle in ModelDesk.
    
    Args:
        ts: TrafficScenario object
        obj: Scenic object
        road_index: Road geometry index
        coordinate_transform: Optional coordinate transformation
        fellow_storage: Dictionary to store fellow references
        
    Returns:
        Fellow object
    """
    # 1) Project position (map frame = Scenic)
    work_x, work_y = obj.position.x, obj.position.y
    if road_index:
        s_val, t_val = project_world_to_st(road_index, (work_x, work_y))
        print(f"World coordinates ({work_x:.3f}, {work_y:.3f}) -> Road coordinates (s={s_val:.1f}, t={t_val:.3f})")
    else:
        s_val, t_val = 0.0, 0.0
    
    # 2) Create Fellow
    F = ts.Fellows.Add()
    try:
        if getattr(obj, "name", None):
            F.Name = str(obj.name)
        else:
            F.Name = f"Fellow_{ts.Fellows.Count}"
        print(f"    Created Fellow with name: {F.Name}")
    except Exception as e:
        F.Name = f"Fellow_{ts.Fellows.Count}"
        print(f"    Created Fellow with fallback name: {F.Name} (error: {e})")
    
    # 3) Configure segments
    seqs = F.Sequences
    clear_collection(seqs)
    S1 = seqs.Add() if hasattr(seqs, "Add") else seqs.Item(0)
    segs = ensure_two_segments(S1)
    
    configure_seg0_absolute_pose(segs, s=float(s_val), t=float(t_val))
    
    base_v = 0.0  # Force velocity to 0 for all vehicles
    configure_seg1_motion(segs, v=float(base_v), t=float(t_val))
    make_endless_transition(segs)
    
    # 4) Store reference
    fellow_storage[F.Name] = {
        'fellow_object': F,
        'sequence': S1,
        'segments': segs,
        'scenic_object': obj
    }
    
    return F


def create_ego_vehicle(ego_maneuver, obj, road_index, coordinate_transform):
    """Configure ego vehicle using Maneuver API.
    
    Args:
        ego_maneuver: Ego maneuver object
        obj: Scenic ego object
        road_index: Road geometry index
        coordinate_transform: Optional coordinate transformation
        
    Returns:
        Maneuver object
    """
    print(f"  Configuring ego vehicle (Maneuver)")
    
    # Project position (map frame = Scenic)
    if not getattr(obj, "position", None):
        s_val, t_val = 0.0, 0.0
        print("  Warning: No position available, using default coordinates (s=0, t=0)")
    else:
        work_x, work_y = obj.position.x, obj.position.y
        if road_index:
            s_val, t_val = project_world_to_st(road_index, (work_x, work_y))
            print(f"  World coordinates ({work_x:.3f}, {work_y:.3f}) -> Road coordinates (s={s_val:.1f}, t={t_val:.3f})")
        else:
            s_val, t_val = 0.0, 0.0
    
    # Configure sequence
    sequences = ego_maneuver.Sequences
    if sequences.Count == 0:
        print("  Warning: No sequences in ego maneuver - cannot configure")
        return None
    
    seq = sequences.Item(0)
    
    print(f"  Setting ego position: s={s_val:.1f}, t={t_val:.3f}, velocity=0.0")
    seq.StartPosition = float(s_val)
    seq.InitialLongitudinalVelocity = 0.0
    seq.VehicleOrientation = 0.0
    print(f"  Set orientation: 0.0 degrees (aligned with road)")
    
    # Set lateral position if needed
    if abs(t_val) > 0.1:
        try:
            segments = seq.Segments
            if segments.Count > 0:
                seg0 = segments.Item(0)
                lat0 = seg0.Activity.LateralType
                from ..geometry import activate_type, set_activity_constant
                activate_type(lat0, "Deviation")
                dep = getattr(lat0.ActiveElement, "DependencyType", None)
                if dep is not None:
                    activate_type(dep, "Absolute")
                set_activity_constant(lat0, t_val)
                print(f"  Set lateral deviation: {t_val:.3f}m")
        except Exception as e:
            print(f"  Warning: Could not set lateral position: {e}")
    
    return ego_maneuver

