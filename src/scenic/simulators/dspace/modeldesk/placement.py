from ..utils import legacy as dutils

def place_ego(sim, obj):
    """Create/configure the ego vehicle using the Maneuver API."""
    # 1) Transform Scenic XODR → RD
    if getattr(obj, "position", None) is not None:
        scenic_x, scenic_y = obj.position.x, obj.position.y
        # Apply coordinate transformation if available
        if sim._coordinate_transform is not None:
            from ..geometry.coordinate_transform import apply_coordinate_transform
            transformed_x, transformed_y = apply_coordinate_transform(
                sim._coordinate_transform, (scenic_x, scenic_y)
            )
            work_x, work_y = transformed_x, transformed_y
        else:
            work_x, work_y = scenic_x, scenic_y
        
        # 2) Determine route FIRST (before projection) using RD coordinates
        route_pref = None
        try:
            # Use the transformed RD coordinates for route detection
            # (road_index is built from RD file, so it expects RD coordinates)
            position_xy = (work_x, work_y)
            track_segment = sim.detectTrackSegment(position_xy)
            if track_segment:
                route_pref = sim.assignRoute(obj, track_segment)
        except Exception as e:
            print(f"  [Route] Could not detect route from RD coordinates: {e}")
        
        # Fallback: try original method (uses XODR coordinates)
        if not route_pref:
            try:
                route_pref = sim._detect_route_from_road_segment(obj)
            except Exception:
                pass
        
        # Default to 'Lap' if route detection fails
        if not route_pref:
            route_pref = 'Lap'
        
        # 3) Project RD → (s,t) using route-specific road index
        if sim._road_index:
            from ..geometry.route_projection import project_world_to_st_route_specific
            s_val, t_val = project_world_to_st_route_specific(
                sim._road_index,
                (work_x, work_y),
                route_preference=route_pref
            )
        else:
            s_val, t_val = dutils.project_world_to_st(sim._road_index, (work_x, work_y))
    else:
        s_val, t_val = 0.0, 0.0
        route_pref = 'Lap'  # Default route

    # 2) Get velocity
    base_v = 0.0

    # 3) Access the ego maneuver
    try:
        maneuver_collection = sim.ts.Maneuver
        if maneuver_collection.Count == 0:
            print("[Ego] Warning: No ego maneuver found in scenario - cannot configure ego")
            return None

        ego_maneuver = maneuver_collection.Item(0)

        # Ensure Maneuver is enabled/active if such a property exists
        try:
            ego_maneuver.Enabled = True
        except:
            pass

        # Access sequences
        sequences = ego_maneuver.Sequences
        if sequences.Count == 0:
            print("[Ego] Warning: No sequences in ego maneuver - cannot configure")
            return None
        seq = sequences.Item(0)

        # 4) Set route FIRST (before setting position) to ensure s-coordinate is interpreted correctly
        # Map route preference to ModelDesk route names (same as fellows)
        route_name_map = {
            'Pit': 'R1',
            'Lap': 'R2'
        }
        modeldesk_route = route_name_map.get(route_pref, 'R2')
        
        try:
            route_sel = seq.Route
            route_sel.UseExternal = False
            route_sel.Direction = 0  # Direct
            route_sel.Activate(modeldesk_route)
            print(f"[Ego] Route set to: {modeldesk_route} (from preference: {route_pref})")
        except Exception as e:
            # Fallback to helper method if direct activation fails
            print(f"[Ego] Direct route activation failed, trying fallback: {e}")
            try:
                sim._set_fellow_route_via_sequence(seq, obj)
            except Exception:
                pass

        # 5) Configure ego vehicle position and properties
        # Store transformation info for later comparison
        if getattr(obj, "position", None) is not None:
            obj._scenic_xodr = (scenic_x, scenic_y)
            obj._expected_rd = (work_x, work_y)
            obj._route_s_t = (s_val, t_val)
            obj._route = route_pref
            print(f"[Ego] XODR: ({scenic_x:.6f}, {scenic_y:.6f}) -> RD: ({work_x:.6f}, {work_y:.6f}) -> Route {route_pref} (s={s_val:.2f}, t={t_val:.6f})")
        
        # STRATEGY: Set StartPosition on Sequence AND Segments to force update
        seq.StartPosition = float(s_val)
        seq.InitialLongitudinalVelocity = float(base_v)

        # Iterate through segments to ensure they don't override the start pos
        for i in range(seq.Segments.Count):
            seg = seq.Segments.Item(i)
            # Some versions allow setting start S on segment 0 explicitly
            if i == 0 and hasattr(seg, 'StartPosition'):
                try:
                    seg.StartPosition = float(s_val)
                except:
                    pass

        # Orientation conversion
        # Transform from Scenic ENU (North=0°) to dSPACE RD (East=0°)
        # Use orientation.yaw directly for clarity (equivalent to heading for most cases)
        import math
        try:
            if hasattr(obj, 'orientation') and hasattr(obj.orientation, 'yaw'):
                scenic_yaw = obj.orientation.yaw
                dspace_orientation = scenic_yaw - math.pi / 2
                seq.VehicleOrientation = dspace_orientation
            elif hasattr(obj, 'heading'):
                # Fallback to heading if orientation.yaw not available
                dspace_orientation = obj.heading - math.pi / 2
                seq.VehicleOrientation = dspace_orientation
            else:
                seq.VehicleOrientation = 0.0
        except Exception:
            seq.VehicleOrientation = 0.0

        # Optional lateral position (Fix for 'Constant' error)
        # NOTE: Known limitation - dSPACE ModelDesk may ignore t-coordinate (lateral deviation)
        # for both ego and fellow vehicles. Testing shows "Could not activate Deviation mode"
        # warning and vehicles are placed on centerline regardless of t value.
        # This is a dSPACE ModelDesk configuration issue, not a bug in our code.
        # See debug_ego_cord/README.md and debug_route_code/README.md for details.
        if abs(t_val) > 0.01: # Lower threshold to catch small offsets
            try:
                segments = seq.Segments
                if segments.Count > 0:
                    seg0 = segments.Item(0)
                    lat0 = seg0.Activity.LateralType
                    dutils.activate_type(lat0, "Deviation")
                    
                    dep = getattr(lat0.ActiveElement, "DependencyType", None)
                    if dep is not None:
                        dutils.activate_type(dep, "Absolute")
                    
                    # RETRY LOGIC for Lateral Property
                    # Try 'Constant', then 'Value', then 'Offset'
                    success_lat = False
                    for prop_name in ['Constant', 'Value', 'Offset', 'LateralOffset']:
                        try:
                            # Try setting property directly on ActiveElement
                            if hasattr(lat0.ActiveElement, prop_name):
                                setattr(lat0.ActiveElement, prop_name, float(t_val))
                                success_lat = True
                                break
                            # Try dutils helper
                            dutils.set_activity_constant(lat0, t_val)
                            success_lat = True
                            break
                        except:
                            continue
                    
                    if not success_lat:
                        pass  # Known limitation - t-coordinate may be ignored by dSPACE

            except Exception:
                pass  # Known limitation - t-coordinate may be ignored by dSPACE

        sim._ego_created = True

        return ego_maneuver
    except Exception as e:
        print(f"[Ego] Error configuring: {e}")
        return None


def place_fellow(sim, obj):
    """Create a Fellow vehicle (non-ego) using the Fellows API."""
    # Get vehicle name for logging
    vehicle_name = getattr(obj, "name", f"Fellow_{sim.ts.Fellows.Count}")
    
    # 1) Transform Scenic XODR → RD
    if getattr(obj, "position", None) is not None:
        scenic_x, scenic_y = obj.position.x, obj.position.y
        if sim._coordinate_transform is not None:
            from ..geometry.coordinate_transform import apply_coordinate_transform
            transformed_x, transformed_y = apply_coordinate_transform(
                sim._coordinate_transform, (scenic_x, scenic_y)
            )
            work_x, work_y = transformed_x, transformed_y
        else:
            work_x, work_y = scenic_x, scenic_y
        
        # 2) Determine route FIRST (before projection) using RD coordinates
        route_pref = None
        try:
            position_xy = (work_x, work_y)
            track_segment = sim.detectTrackSegment(position_xy)
            if track_segment:
                route_pref = sim.assignRoute(obj, track_segment)
        except Exception as e:
            pass
        
        # Fallback: try original method (uses XODR coordinates)
        if not route_pref:
            try:
                route_pref = sim._detect_route_from_road_segment(obj)
            except Exception:
                pass
        
        # Default to 'Lap' if route detection fails
        if not route_pref:
            route_pref = 'Lap'
        
        # 3) Project RD → (s,t) using route-specific road index
        if sim._road_index:
            from ..geometry.route_projection import project_world_to_st_route_specific
            s_val, t_val = project_world_to_st_route_specific(
                sim._road_index, 
                (work_x, work_y),
                route_preference=route_pref
            )
        else:
            s_val, t_val = dutils.project_world_to_st(sim._road_index, (work_x, work_y))
        
        # Store transformation info for later comparison
        obj._scenic_xodr = (scenic_x, scenic_y)
        obj._expected_rd = (work_x, work_y)
        obj._route_s_t = (s_val, t_val)
        obj._route = route_pref
        
        # Log transformation chain
        print(f"[{vehicle_name}] XODR: ({scenic_x:.6f}, {scenic_y:.6f}) -> RD: ({work_x:.6f}, {work_y:.6f}) -> Route {route_pref} (s={s_val:.2f}, t={t_val:.6f})")
    else:
        s_val, t_val = 0.0, 0.0
        route_pref = 'Lap'
        print(f"[{vehicle_name}] Warning: No position available, using default (s=0, t=0)")

    # 3) Create Fellow
    F = sim.ts.Fellows.Add()

    try:
        if getattr(obj, "name", None):
            F.Name = str(obj.name)
        else:
            F.Name = f"Fellow_{sim.ts.Fellows.Count}"
    except Exception as e:
        F.Name = f"Fellow_{sim.ts.Fellows.Count}"

    seqs = F.Sequences
    dutils.clear_collection(seqs)
    S1 = seqs.Add() if hasattr(seqs, "Add") else seqs.Item(0)
    segs = dutils.ensure_two_segments(S1)

    # seg0 = ABSOLUTE pose: Position = s, Deviation(Absolute) = t
    # NOTE: Known limitation - dSPACE ModelDesk may ignore t-coordinate (lateral deviation)
    # for fellow vehicles. Testing shows vehicles are placed on centerline regardless of t value.
    # This is a dSPACE ModelDesk configuration issue, not a bug in our code.
    # See debug_ego_cord/README.md and debug_route_code/README.md for details.
    dutils.configure_seg0_absolute_pose(segs, s=float(s_val), t=float(t_val))

    # seg1 = Longitudinal Velocity (Extern), Lateral deviation (Extern)
    try:
        dutils.configure_seg1_motion(segs, v=0.0, t=0.0)
    except Exception as e:
        print(f"    Warning: could not set seg1 Velocity(Extern)/Lateral deviation(Extern): {e}")
    try:
        dutils.make_endless_transition(segs)
    except Exception:
        pass

    # Orientation
    # Transform from Scenic ENU (North=0°) to dSPACE RD (East=0°)
    # Use orientation.yaw directly for clarity (equivalent to heading for most cases)
    try:
        import math
        if hasattr(obj, 'orientation') and hasattr(obj.orientation, 'yaw'):
            scenic_yaw = obj.orientation.yaw
            dspace_orientation = scenic_yaw - math.pi / 2
            if hasattr(S1, 'VehicleOrientation'):
                S1.VehicleOrientation = dspace_orientation
        elif hasattr(obj, 'heading'):
            # Fallback to heading if orientation.yaw not available
            dspace_orientation = obj.heading - math.pi / 2
            if hasattr(S1, 'VehicleOrientation'):
                S1.VehicleOrientation = dspace_orientation
    except Exception:
        pass

    # Route (already determined above, but set it explicitly to ensure consistency)
    # The route was determined before projection, so we know it's correct
    try:
        route_sel = S1.Route
        route_sel.UseExternal = False
        route_sel.Direction = 0  # Direct
        
        # Map route preference to ModelDesk route names
        route_name_map = {
            'Pit': 'R1',
            'Lap': 'R2'
        }
        modeldesk_route = route_name_map.get(route_pref, 'R2')
        
        try:
            route_sel.Activate(modeldesk_route)
        except Exception as e:
            # Fallback to original method if direct activation fails
            sim._set_fellow_route_via_sequence(S1, obj)
    except Exception as e:
        # Fallback to original method
        sim._set_fellow_route_via_sequence(S1, obj)

    # Store reference
    sim._fellow_vehicles[F.Name] = {
        'fellow_object': F,
        'sequence': S1,
        'segments': segs,
        'scenic_object': obj,
        'index': sim.ts.Fellows.Count - 1
    }

    return F