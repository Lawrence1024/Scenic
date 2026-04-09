"""Route-specific projection utilities for converting coordinates to route-relative (s,t) values."""

from typing import Tuple, Optional, Dict, Any
import math

# Known route s=0 origins in RD coordinates (from ModelDesk road table)
# These are the RD coordinates that correspond to s=0 on each route
ROUTE_ORIGINS = {
    'R1': (163.54, 48.30),   # Pit route origin = "Pit Lane1_2" start
    'R2': (172.52, 53.55),  # Lap route origin = "Andretti Hairpin1_3" start
}

# Road start positions in RD coordinates (from ModelDesk road table)
ROAD_START_POSITIONS = {
    'Andretti Hairpin1_3': (172.52, 53.55),   # R2 s=0
    'Pit Lane1_2': (163.54, 48.30),           # R1 s=0
    'The Corkscrew1': (-101.92, -457.52),     # Part of both R1 and R2 (at different sequence positions)
}

# Route road sequences (order of roads in each route)
# Based on ModelDesk route definitions
# NOTE: R1 forms a loop - it includes Pit Lane1_2 and continues to The Corkscrew1
# R2 also includes The Corkscrew1, so both routes share this road but at different sequence positions
ROUTE_ROAD_SEQUENCES = {
    'R1': ['Pit Lane1_2', 'The Corkscrew1'],  # R1: Pit lane -> Corkscrew (forms loop)
    'R2': ['Andretti Hairpin1_3', 'The Corkscrew1'],  # R2: Andretti -> Corkscrew
}

# Map route preferences to ModelDesk route names
ROUTE_NAME_MAP = {
    'Pit': 'R1',
    'Lap': 'R2',
}

# Exact transition points from route s to The Corkscrew1 (from calibration testing)
# These are the route s-coordinates where the transition from first road to The Corkscrew1 occurs
ROUTE_TRANSITION_POINTS = {
    'R1': 902.0,   # Transition from Pit Lane1_2 to The Corkscrew1
    'R2': 1006.0,  # Transition from Andretti Hairpin1_3 to The Corkscrew1
}

# Offset corrections for The Corkscrew1 (from offset precision testing)
# These offsets account for the systematic offset when converting route s to road s
# Formula: road_s = (route_s - transition_point) - offset
# Inverse: route_s = road_s + transition_point + offset
# Testing shows:
# - At transition point (road_s=0): offset is 0.00m (perfect alignment)
# - For road_s > 0: consistent ~17.6m offset needed for R1, ~17.9m for R2
# - Offset is constant along The Corkscrew1 (within 0.05m)
ROUTE_CORKSCREW_OFFSETS = {
    'R1': 17.6,   # ~17.6m offset for R1 on The Corkscrew1 (refined from precision testing)
    'R2': 17.9,   # ~17.9m offset for R2 on The Corkscrew1 (refined from precision testing)
}


def build_route_specific_road_index(road_index: Dict[str, Any], route_preference: str) -> Optional[Dict[str, Any]]:
    """Build a route-specific road index by filtering roads based on route.
    
    Args:
        road_index: Full road index containing all roads
        route_preference: Route preference string ('Pit' for R1, 'Lap' for R2)
        
    Returns:
        Filtered road index containing only roads for the specified route, or None if filtering fails
    """
    if not road_index or 'roads' not in road_index:
        return None
    
    # Map route preference to road name patterns
    # Based on RD file structure:
    # - R1 (Pit): "Pit Lane1_2" (id=1)
    # - R2 (Lap): "The Corkscrew1" (id=0) and other main racing roads
    route_road_patterns = {
        'Pit': ['pit', 'lane'],  # R1: pit lane roads
        'Lap': ['corkscrew', 'hairpin', 'main']  # R2: main racing roads (exclude pit)
    }
    
    # Determine which roads to include
    if route_preference == 'Pit':
        include_patterns = route_road_patterns['Pit']
        exclude_patterns = []
    elif route_preference == 'Lap':
        include_patterns = route_road_patterns['Lap']
        exclude_patterns = route_road_patterns['Pit']
    else:
        # Unknown route, return original index
        return road_index
    
    # Filter roads
    filtered_roads = {}
    for road_name, road_data in road_index['roads'].items():
        road_name_lower = str(road_name).lower()
        
        # Check if road matches route
        matches_include = any(pattern in road_name_lower for pattern in include_patterns)
        matches_exclude = any(pattern in road_name_lower for pattern in exclude_patterns)
        
        # Include if matches include patterns and doesn't match exclude patterns
        if matches_include and not matches_exclude:
            filtered_roads[road_name] = road_data
    
    # If no roads matched, fall back to original index (better than nothing)
    if not filtered_roads:
        return road_index
    
    # Build filtered road index
    filtered_index = {
        'roads': filtered_roads
    }
    
    return filtered_index


def find_route_s_from_rd_coordinate(
    rd_coord: Tuple[float, float],
    route_name: str,
    road_index: Dict[str, Any]
) -> Optional[float]:
    """Find the route-relative s coordinate for a given RD coordinate by testing.
    
    This function places a fellow at different s values on the route and finds
    which s gives the closest match to the target RD coordinate.
    
    Args:
        rd_coord: Target RD coordinate (x, y)
        route_name: Route name ('R1' or 'R2')
        road_index: Road index (for projection fallback)
        
    Returns:
        Route-relative s coordinate, or None if not found
    """
    # This is a calibration function - for now, we'll use a simpler approach
    # by finding the distance from route origin and estimating s
    if route_name not in ROUTE_ORIGINS:
        return None
    
    route_origin = ROUTE_ORIGINS[route_name]
    
    # Calculate distance from route origin
    dx = rd_coord[0] - route_origin[0]
    dy = rd_coord[1] - route_origin[1]
    distance = math.sqrt(dx*dx + dy*dy)
    
    # For now, use distance as an approximation of s
    # This is a rough estimate - in reality, we'd need to follow the route geometry
    # But since routes are roughly linear in their early segments, this might work
    return distance


def find_road_s0_rd_coordinate(road_index: Dict[str, Any], road_id: int) -> Optional[Tuple[float, float]]:
    """Find the RD coordinate that corresponds to s=0 on a given road.
    
    Args:
        road_index: Road index
        road_id: Road ID
        
    Returns:
        (x, y) RD coordinate at s=0 of the road, or None if not found
    """
    if not road_index or 'roads' not in road_index:
        return None
    
    for road_name, road_data in road_index['roads'].items():
        if road_data.get('id') == road_id:
            sec_points = road_data.get('sec_points', [])
            if sec_points and len(sec_points) > 0:
                pts = sec_points[0]  # First section
                if pts and len(pts) > 0:
                    # First point is at s=0
                    x, y, s = pts[0]
                    return (x, y)
    return None


def find_road_name_by_id(road_index: Dict[str, Any], road_id: int) -> Optional[str]:
    """Find road name by road ID."""
    if not road_index or 'roads' not in road_index:
        return None
    
    for road_name, road_data in road_index['roads'].items():
        if road_data.get('id') == road_id:
            return road_name
    return None


def calculate_route_s_from_road_sequence(
    road_name: str,
    road_relative_s: float,
    route_name: str,
    road_index: Dict[str, Any]
) -> Optional[float]:
    """Calculate route-relative s from road-relative s using route road sequence.
    
    Uses calibrated transition points and offsets from testing:
    - First roads: route s = road s (direct mapping, perfect alignment)
    - The Corkscrew1: 
      - At transition (road_s ≈ 0): route_s = transition_point (no offset)
      - After transition (road_s > 0): route_s = road_s + transition_point + offset
      - R1: transition_point = 902.0, offset = 9.6m
      - R2: transition_point = 1006.0, offset = 8.9m
    
    Args:
        road_name: Name of the road the coordinate projects onto
        road_relative_s: s coordinate relative to that road (0 to road_length)
        route_name: Route name ('R1' or 'R2')
        road_index: Road index (for getting road lengths)
        
    Returns:
        Route-relative s coordinate, or None if calculation fails
    """
    if route_name not in ROUTE_ROAD_SEQUENCES:
        return None
    
    route_sequence = ROUTE_ROAD_SEQUENCES[route_name]
    
    # Find where this road appears in the route sequence
    try:
        road_index_in_route = route_sequence.index(road_name)
    except ValueError:
        # Road not in route sequence - might be wrong route
        return None
    
    # First road (index 0): Direct mapping - route s = road s
    # From calibration: First roads map perfectly (within 0.01m)
    if road_index_in_route == 0:
        route_s = road_relative_s
        return route_s
    
    # The Corkscrew1 (index 1): Use transition point with conditional offset
    # From offset precision testing:
    # - R1: Transition at route s=902.0, where road_s = 0.00 (perfect match with 9.6m offset)
    # - R2: Transition at route s=1006.0, where road_s = 0.00 (perfect match with 8.9m offset)
    # - After transition (road_s > 0): Need larger offset (~17.6m for R1, ~17.9m for R2)
    # Strategy:
    #   - At transition point (road_s ≈ 0): use transition offset (9.6m for R1, 8.9m for R2)
    #   - After transition (road_s > 0): use larger offset (17.6m for R1, 17.9m for R2)
    # This ensures correct transition point mapping while applying proper offset correction along The Corkscrew1
    if road_name == 'The Corkscrew1' and road_index_in_route == 1:
        if route_name in ROUTE_TRANSITION_POINTS and route_name in ROUTE_CORKSCREW_OFFSETS:
            transition_point = ROUTE_TRANSITION_POINTS[route_name]
            offset = ROUTE_CORKSCREW_OFFSETS[route_name]
            
            # Use transition offset at transition point, larger offset after transition
            # Offset precision testing shows:
            # - At road_s=0: offset=9.6m (R1) or 8.9m (R2) gives perfect alignment
            # - At road_s>0: offset=17.6m (R1) or 17.9m (R2) is needed for perfect alignment
            # Use a threshold to switch between offsets (e.g., road_s < 1.0m uses transition offset)
            TRANSITION_OFFSETS = {
                'R1': 9.6,   # Offset that works at transition point
                'R2': 8.9,   # Offset that works at transition point
            }
            
            if road_relative_s < 1.0:
                # At or very near transition point: use transition offset
                transition_offset = TRANSITION_OFFSETS.get(route_name, offset)
                route_s = road_relative_s + transition_point + transition_offset
            else:
                # After transition: use larger offset for positions along The Corkscrew1
                route_s = road_relative_s + transition_point + offset
            return route_s
    
    # Fallback: cumulative-length calculation (shouldn't be reached for known routes)
    # Calculate cumulative s up to this road
    cumulative_s = 0.0
    
    # Sum lengths of all roads before this one in the sequence
    for i in range(road_index_in_route):
        prev_road_name = route_sequence[i]
        # Find road in index
        if road_index and 'roads' in road_index:
            prev_road_data = road_index['roads'].get(prev_road_name)
            if prev_road_data:
                prev_road_length = prev_road_data.get('length', 0)
                cumulative_s += prev_road_length
    
    # Add the road-relative s to get final route-relative s
    route_s = cumulative_s + road_relative_s
    
    return route_s


def project_world_to_st_route_specific(
    road_index: Dict[str, Any],
    pos: Tuple[float, float],
    route_preference: Optional[str] = None
) -> Tuple[float, float]:
    """Project world coordinates to (s,t) using route-specific road sequence.
    
    Uses calibrated transition points from testing:
    1. Projects coordinate to road-relative (s, t)
    2. Determines which road the coordinate is on
    3. Converts to route-relative s using:
       - First roads: route s = road s (direct mapping)
       - The Corkscrew1: route s = road_s + transition_point
    
    Args:
        road_index: Full road index containing all roads
        pos: World coordinates (x, y) in RD coordinate system
        route_preference: Optional route preference ('Pit' for R1, 'Lap' for R2)
        
    Returns:
        (s, t) tuple relative to the route's coordinate system
    """
    from .projection import project_world_to_st, find_road_id_for_position
    
    # When route is specified, project only onto roads that belong to that route.
    # Otherwise we can project onto the wrong road (e.g. Lap vehicle projected onto pit TTL).
    index_for_projection = road_index
    if route_preference and route_preference in ROUTE_NAME_MAP:
        filtered = build_route_specific_road_index(road_index, route_preference)
        if filtered is not None and filtered.get("roads"):
            index_for_projection = filtered

    # Project to get road-relative (s, t)
    s_road, t_val = project_world_to_st(index_for_projection, pos)

    # If no route specified, return road-relative s (original behavior)
    if not route_preference or route_preference not in ROUTE_NAME_MAP:
        return (s_road, t_val)

    route_name = ROUTE_NAME_MAP[route_preference]
    
    if route_name not in ROUTE_ROAD_SEQUENCES:
        return (s_road, t_val)
    
    # Find which road this projects onto (use same index we projected with)
    road_id = find_road_id_for_position(index_for_projection, pos[0], pos[1])
    
    if road_id is None:
        # Fallback: use distance from route origin
        if route_name in ROUTE_ORIGINS:
            route_origin = ROUTE_ORIGINS[route_name]
            dx = pos[0] - route_origin[0]
            dy = pos[1] - route_origin[1]
            s_route = math.sqrt(dx*dx + dy*dy)
            return (s_route, t_val)
        return (s_road, t_val)
    
    # Find road name
    road_name = find_road_name_by_id(road_index, road_id)
    
    if road_name is None:
        return (s_road, t_val)
    
    # Calculate route-relative s using road sequence
    s_route = calculate_route_s_from_road_sequence(
        road_name, s_road, route_name, road_index
    )
    
    if s_route is not None:
        return (s_route, t_val)
    else:
        # Fallback: use road-relative s
        return (s_road, t_val)


def path_curvature_at_pos_route_aware(
    road_index: Dict[str, Any],
    pos: Tuple[float, float],
    route_preference: Optional[str] = None,
) -> float:
    """Signed curvature (rad/m) at ``pos`` using the same polyline set as route-specific (s,t)."""
    from .projection import path_curvature_signed_at_world_pos

    index_for_projection = road_index
    if route_preference and route_preference in ROUTE_NAME_MAP:
        filtered = build_route_specific_road_index(road_index, route_preference)
        if filtered is not None and filtered.get("roads"):
            index_for_projection = filtered
    return path_curvature_signed_at_world_pos(index_for_projection, pos)
