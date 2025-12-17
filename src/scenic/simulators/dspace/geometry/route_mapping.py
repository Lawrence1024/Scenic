def detect_track_segment(position_xy, road_index, params, utils_module):
    """Return 'pitLane', 'mainRacing', or None based on projection and params."""
    try:
        pit_lane_ids = params.get('pitLaneRoadIds', [])
        main_racing_ids = params.get('mainRacingRoadIds', [])
        
        # Debug logging
        print(f"  [RouteDetection] Position: ({position_xy[0]:.6f}, {position_xy[1]:.6f})")
        print(f"  [RouteDetection] pitLaneRoadIds: {pit_lane_ids}")
        print(f"  [RouteDetection] mainRacingRoadIds: {main_racing_ids}")
        
        if not pit_lane_ids and not main_racing_ids:
            print(f"  [RouteDetection] No road IDs available - returning None")
            return None
        if not road_index:
            print(f"  [RouteDetection] No road index available - returning None")
            return None
        
        obj_x, obj_y = float(position_xy[0]), float(position_xy[1])
        projected_road_id = utils_module.find_road_id_for_position(road_index, obj_x, obj_y)
        print(f"  [RouteDetection] Projected road ID (raw): {projected_road_id} (type: {type(projected_road_id).__name__})")
        
        # If the projection returns an internal RD id, try mapping to XODR id
        original_road_id = projected_road_id
        try:
            if projected_road_id is not None and hasattr(utils_module, 'map_rd_to_xodr_road_id'):
                mapped = utils_module.map_rd_to_xodr_road_id(road_index, projected_road_id)
                if mapped is not None:
                    projected_road_id = mapped
                    print(f"  [RouteDetection] Mapped RD ID {original_road_id} -> XODR ID {projected_road_id}")
        except Exception as e:
            print(f"  [RouteDetection] Mapping failed: {e}")
        
        if projected_road_id is None:
            print(f"  [RouteDetection] Projected road ID is None - returning None")
            return None
        
        # Try direct ID matching
        if str(projected_road_id) in pit_lane_ids:
            print(f"  [RouteDetection] Matched pitLaneRoadIds -> 'pitLane'")
            return 'pitLane'
        if str(projected_road_id) in main_racing_ids:
            print(f"  [RouteDetection] Matched mainRacingRoadIds -> 'mainRacing'")
            return 'mainRacing'
        
        # Fallback by name
        try:
            if hasattr(utils_module, 'get_road_name_for_id'):
                rname = utils_module.get_road_name_for_id(road_index, projected_road_id)
            else:
                rname = None
            if rname:
                lname = str(rname).lower()
                print(f"  [RouteDetection] Road name: '{rname}' (lowercase: '{lname}')")
                if 'pit' in lname:
                    print(f"  [RouteDetection] Name contains 'pit' -> 'pitLane'")
                    return 'pitLane'
                else:
                    print(f"  [RouteDetection] Name does not contain 'pit' -> 'mainRacing'")
                    return 'mainRacing'
        except Exception as e:
            print(f"  [RouteDetection] Name lookup failed: {e}")
        
        # Last heuristic: RD ids 0/1/2 → assume 1 is pit
        try:
            if isinstance(projected_road_id, int) and projected_road_id in (0, 1, 2):
                result = 'pitLane' if projected_road_id == 1 else 'mainRacing'
                print(f"  [RouteDetection] Using heuristic: RD ID {projected_road_id} -> '{result}'")
                return result
        except Exception as e:
            print(f"  [RouteDetection] Heuristic failed: {e}")
        
        print(f"  [RouteDetection] No match found - returning None")
        return None
    except Exception as e:
        print(f"  [RouteDetection] Exception: {e}")
        return None


def assign_route_for_segment(track_segment):
    """Map track segment to dSPACE route preference string."""
    if track_segment == 'pitLane':
        return 'Pit'
    if track_segment == 'mainRacing':
        return 'Lap'
    return None


