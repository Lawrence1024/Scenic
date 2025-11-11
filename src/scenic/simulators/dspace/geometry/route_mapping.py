def detect_track_segment(position_xy, road_index, params, utils_module):
    """Return 'pitLane', 'mainRacing', or None based on projection and params."""
    try:
        pit_lane_ids = params.get('pitLaneRoadIds', [])
        main_racing_ids = params.get('mainRacingRoadIds', [])
        if not pit_lane_ids and not main_racing_ids:
            return None
        if not road_index:
            return None
        obj_x, obj_y = float(position_xy[0]), float(position_xy[1])
        projected_road_id = utils_module.find_road_id_for_position(road_index, obj_x, obj_y)
        # If the projection returns an internal RD id, try mapping to XODR id
        try:
            if projected_road_id is not None and hasattr(utils_module, 'map_rd_to_xodr_road_id'):
                mapped = utils_module.map_rd_to_xodr_road_id(road_index, projected_road_id)
                if mapped is not None:
                    projected_road_id = mapped
        except Exception:
            pass
        if projected_road_id is None:
            return None
        if str(projected_road_id) in pit_lane_ids:
            return 'pitLane'
        if str(projected_road_id) in main_racing_ids:
            return 'mainRacing'
        # Fallback by name
        try:
            if hasattr(utils_module, 'get_road_name_for_id'):
                rname = utils_module.get_road_name_for_id(road_index, projected_road_id)
            else:
                rname = None
            if rname:
                lname = str(rname).lower()
                if 'pit' in lname:
                    return 'pitLane'
                else:
                    return 'mainRacing'
        except Exception:
            pass
        # Last heuristic: RD ids 0/1/2 → assume 1 is pit
        try:
            if isinstance(projected_road_id, int) and projected_road_id in (0, 1, 2):
                return 'pitLane' if projected_road_id == 1 else 'mainRacing'
        except Exception:
            pass
        return None
    except Exception:
        return None


def assign_route_for_segment(track_segment):
    """Map track segment to dSPACE route preference string."""
    if track_segment == 'pitLane':
        return 'Pit'
    if track_segment == 'mainRacing':
        return 'Lap'
    return None


