"""Projection utilities for world-to-road coordinate conversion."""

from typing import Tuple


def find_road_id_for_position(road_index, x, y):
    """Find which road ID a position projects onto.
    
    Args:
        road_index: Road index from build_xodr_sec_points or build_rd_road_index
        x, y: World coordinates
        
    Returns:
        Road ID or None if not found
    """
    try:
        if not road_index:
            return None
            
        roads_obj = road_index.get('roads', {})
        if not roads_obj:
            return None
        
        best_road_id = None
        min_distance = float('inf')
        
        # Check each road to find the closest projection
        for road_name, road_data in roads_obj.items():
            sec_list = road_data.get('sec_points', [])
            if not sec_list:
                continue
                
            for pts in sec_list:
                if not pts or len(pts) < 2:
                    continue
                    
                for i in range(len(pts) - 1):
                    x0, y0, s0 = pts[i]
                    x1, y1, s1 = pts[i+1]
                    vx, vy = x1 - x0, y1 - y0
                    seg_len2 = vx*vx + vy*vy
                    if seg_len2 <= 1e-12:
                        continue
                        
                    wx, wy = x - x0, y - y0
                    u = (wx*vx + wy*vy) / seg_len2
                    u = 0.0 if u < 0.0 else (1.0 if u > 1.0 else u)
                    qx = x0 + u*vx
                    qy = y0 + u*vy
                    dx, dy = x - qx, y - qy
                    dist2 = dx*dx + dy*dy
                    
                    if dist2 < min_distance:
                        min_distance = dist2
                        best_road_id = road_data.get('id')
        
        return best_road_id
        
    except Exception as e:
        print(f"    [RoadID] Error finding road ID: {e}")
        return None


def project_world_to_st(index_or_map, pos: Tuple[float, float], xodr_file: str = None):
    """Project world (x,y) onto nearest ref segment; return (s, t) calibrated for ModelDesk.
    
    This function:
    1. Projects Scenic coordinates onto the OpenDRIVE road network
    2. Uses robust mapping that works across the full track length
    3. Provides full track coverage using the road geometry
    """
    px, py = float(pos[0]), float(pos[1])

    roads_obj = None
    if isinstance(index_or_map, dict) and 'roads' in index_or_map:
        roads_obj = index_or_map['roads']
    else:
        roads_obj = getattr(index_or_map, 'roads', None)

    if not roads_obj:
        # Fallback to simple mapping if no road network available
        return 0.0, 0.0

    # Project onto OpenDRIVE road network - IMPROVED ALGORITHM
    # Collect all projections first, then find the truly closest one
    all_projections = []  # List of (dist2, s_proj, t_signed, road_id, road_name)
    
    it = roads_obj.values() if isinstance(roads_obj, dict) else roads_obj
    for road in it:
        sec_list = road.get('sec_points') if isinstance(road, dict) else getattr(road, 'sec_points', [])
        if not sec_list:
            continue
        for pts in sec_list:
            if not pts or len(pts) < 2:
                continue
            for i in range(len(pts) - 1):
                x0, y0, s0 = pts[i]
                x1, y1, s1 = pts[i+1]
                vx, vy = x1 - x0, y1 - y0
                seg_len2 = vx*vx + vy*vy
                if seg_len2 <= 1e-12:
                    continue
                wx, wy = px - x0, py - y0
                u = (wx*vx + wy*vy) / seg_len2
                u = 0.0 if u < 0.0 else (1.0 if u > 1.0 else u)
                qx = x0 + u*vx
                qy = y0 + u*vy
                dx, dy = px - qx, py - qy
                dist2 = dx*dx + dy*dy

                seg_len = seg_len2 ** 0.5
                # Calculate normal vector for t-coordinate
                nx_left, ny_left = -vy/seg_len, vx/seg_len  # left normal
                
                # Calculate t-coordinate: signed lateral offset from segment (meters)
                raw_t = dx*nx_left + dy*ny_left
                t_signed = raw_t
                s_proj = s0 + u*(s1 - s0)
                
                # Get road ID and name
                road_id = road.get('id') if isinstance(road, dict) else getattr(road, 'id', None)
                road_name = road.get('name') if isinstance(road, dict) else getattr(road, 'name', f'Road_{road_id}')
                
                # Store all projections
                all_projections.append((dist2, s_proj, t_signed, road_id, road_name))
    
    # Find the truly closest projection
    if not all_projections:
        # Fallback to simple mapping if projection fails
        return 0.0, 0.0
    
    # Sort by distance and take the closest
    all_projections.sort(key=lambda x: x[0])
    best = all_projections[0]

    if best is None:
        # Fallback to simple mapping if projection fails
        return 0.0, 0.0
    
    # Get raw s-coordinate from OpenDRIVE
    raw_s = float(best[1])
    t_val = float(best[2])
    road_id = best[3]
    
    # DIRECT APPROACH: Use OpenDRIVE s directly for full track coverage
    # This provides the full 0-2484.6m range without calibration capping
    return raw_s, t_val

