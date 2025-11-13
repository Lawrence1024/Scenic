from ..utils import legacy as dutils

def place_ego(sim, obj):
    """Create/configure the ego vehicle using the Maneuver API."""
    # 1) Project Scenic (x,y) → (s,t)
    if getattr(obj, "position", None) is not None:
        scenic_x, scenic_y = obj.position.x, obj.position.y
        # Apply coordinate transformation if available
        if sim._coordinate_transform is not None:
            from ..geometry.coordinate_transform import apply_coordinate_transform
            transformed_x, transformed_y = apply_coordinate_transform(
                sim._coordinate_transform, (scenic_x, scenic_y)
            )
            print(f"  Scenic coords ({scenic_x:.3f}, {scenic_y:.3f}) -> RD coords ({transformed_x:.3f}, {transformed_y:.3f})")
            work_x, work_y = transformed_x, transformed_y
        else:
            work_x, work_y = scenic_x, scenic_y
        # Use road index for proper geometric projection
        if sim._road_index:
            s_val, t_val = dutils.project_world_to_st(sim._road_index, (work_x, work_y))
            print(f"  World coordinates ({work_x:.3f}, {work_y:.3f}) -> Road coordinates (s={s_val:.1f}, t={t_val:.3f})")
        else:
            s_val, t_val = dutils.project_world_to_st(sim._road_index, (work_x, work_y))
            print(f"  World coordinates ({work_x:.3f}, {work_y:.3f}) -> Fallback coordinates (s={s_val:.1f}, t={t_val:.3f})")
    else:
        s_val, t_val = 0.0, 0.0
        print("  Warning: No position available, using default coordinates (s=0, t=0)")

    # 2) Get velocity - always set to 0 for static scenarios
    base_v = 0.0

    # 3) Access the ego maneuver (Maneuver is a collection, use Item(0))
    try:
        maneuver_collection = sim.ts.Maneuver
        if maneuver_collection.Count == 0:
            print("  Warning: No ego maneuver found in scenario - cannot configure ego")
            return None

        ego_maneuver = maneuver_collection.Item(0)
        print(f"  Accessed ego maneuver: {ego_maneuver.Name if hasattr(ego_maneuver, 'Name') else 'Ego'}")

        # Access sequences
        sequences = ego_maneuver.Sequences
        if sequences.Count == 0:
            print("  Warning: No sequences in ego maneuver - cannot configure")
            return None
        seq = sequences.Item(0)

        # 4) Configure ego vehicle position and properties
        print(f"  Setting ego position: s={s_val:.1f}, t={t_val:.3f}, velocity={base_v:.1f}")
        seq.StartPosition = float(s_val)
        seq.InitialLongitudinalVelocity = float(base_v)

        # Orientation conversion: dSPACE_orientation = scenic_heading - π/2
        if hasattr(obj, 'heading'):
            import math
            dspace_orientation = obj.heading - math.pi / 2
            seq.VehicleOrientation = dspace_orientation
            print(f"  Set orientation: {math.degrees(dspace_orientation):.1f} degrees (from Scenic heading {math.degrees(obj.heading):.1f})")
        else:
            seq.VehicleOrientation = 0.0
            print(f"  Set orientation: 0.0 degrees (aligned with road)")

        # Optional lateral position through segments if t != 0
        if abs(t_val) > 0.1:
            try:
                segments = seq.Segments
                if segments.Count > 0:
                    seg0 = segments.Item(0)
                    lat0 = seg0.Activity.LateralType
                    dutils.activate_type(lat0, "Deviation")
                    dep = getattr(lat0.ActiveElement, "DependencyType", None)
                    if dep is not None:
                        dutils.activate_type(dep, "Absolute")
                    dutils.set_activity_constant(lat0, t_val)
                    print(f"  Set lateral deviation: {t_val:.3f}m")
            except Exception as e:
                print(f"  Warning: Could not set lateral position: {e}")

        sim._ego_created = True

        # 5) Set route for ego via same helper as fellows
        try:
            sim._set_fellow_route_via_sequence(seq, obj)
        except Exception as e:
            print(f"  [Route] Could not set route for ego: {e}")

        return ego_maneuver
    except Exception as e:
        print(f"  Error configuring ego vehicle: {e}")
        import traceback
        traceback.print_exc()
        return None


def place_fellow(sim, obj):
    """Create a Fellow vehicle (non-ego) using the Fellows API."""
    # 1) Project Scenic (x,y) → (s,t). If no map, use zeros.
    if getattr(obj, "position", None) is not None:
        scenic_x, scenic_y = obj.position.x, obj.position.y
        if sim._coordinate_transform is not None:
            from ..geometry.coordinate_transform import apply_coordinate_transform
            transformed_x, transformed_y = apply_coordinate_transform(
                sim._coordinate_transform, (scenic_x, scenic_y)
            )
            print(f"Scenic coords ({scenic_x:.3f}, {scenic_y:.3f}) -> RD coords ({transformed_x:.3f}, {transformed_y:.3f})")
            work_x, work_y = transformed_x, transformed_y
        else:
            work_x, work_y = scenic_x, scenic_y
        if sim._road_index:
            s_val, t_val = dutils.project_world_to_st(sim._road_index, (work_x, work_y))
            route_pref = sim._detect_route_from_road_segment(obj)
            if route_pref == "Pit":
                if s_val > 1000:
                    s_val = s_val % 883.4
            print(f"World coordinates ({work_x:.3f}, {work_y:.3f}) -> Road coordinates (s={s_val:.1f}, t={t_val:.3f})")
        else:
            s_val, t_val = dutils.project_world_to_st(sim._road_index, (work_x, work_y))
            print(f"World coordinates ({work_x:.3f}, {work_y:.3f}) -> Fallback coordinates (s={s_val:.1f}, t={t_val:.3f})")
    else:
        s_val, t_val = 0.0, 0.0
        print("Warning: No position available, using default coordinates (s=0, t=0)")

    # 3) Create Fellow with one Sequence and two Segments
    F = sim.ts.Fellows.Add()

    # Set a unique name
    try:
        if getattr(obj, "name", None):
            F.Name = str(obj.name)
        else:
            F.Name = f"Fellow_{sim.ts.Fellows.Count}"
        print(f"    Created Fellow with name: {F.Name}")
    except Exception as e:
        F.Name = f"Fellow_{sim.ts.Fellows.Count}"
        print(f"    Created Fellow with fallback name: {F.Name} (error: {e})")

    seqs = F.Sequences
    dutils.clear_collection(seqs)
    S1 = seqs.Add() if hasattr(seqs, "Add") else seqs.Item(0)
    segs = dutils.ensure_two_segments(S1)

    # seg0 = ABSOLUTE pose: Position = s, Deviation(Absolute) = t
    dutils.configure_seg0_absolute_pose(segs, s=float(s_val), t=float(t_val))

    # seg1 = Longitudinal Velocity (Extern), Lateral deviation (Extern); make segment endless
    # Both are configured with SourceType='Extern' to enable external control via ControlDesk
    try:
        dutils.configure_seg1_motion(segs, v=0.0, t=0.0)
    except Exception as e:
        print(f"    Warning: could not set seg1 Velocity(Extern)/Lateral deviation(Extern): {e}")
    try:
        dutils.make_endless_transition(segs)
    except Exception:
        pass

    # Orientation (if supported)
    if hasattr(obj, 'heading'):
        try:
            import math
            dspace_orientation = obj.heading - math.pi / 2
            if hasattr(S1, 'VehicleOrientation'):
                S1.VehicleOrientation = dspace_orientation
                print(f"    Set orientation: {math.degrees(dspace_orientation):.1f} degrees (from Scenic heading {math.degrees(obj.heading):.1f})")
        except Exception as e:
            print(f"    Note: Cannot set orientation for Fellow (not supported or error: {e})")

    # Route via sequence.Route
    sim._set_fellow_route_via_sequence(S1, obj)

    # Store fellow reference for dynamic control
    sim._fellow_vehicles[F.Name] = {
        'fellow_object': F,
        'sequence': S1,
        'segments': segs,
        'scenic_object': obj,
        'index': sim.ts.Fellows.Count - 1
    }

    return F


