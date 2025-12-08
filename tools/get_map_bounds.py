#!/usr/bin/env python3
"""
Get the boundary points and bounding box of a map from an XODR file.

This module also provides utilities for finding the closest waypoint when a car
is out of bounds, which is useful for recovery navigation.

Usage:
    python get_map_bounds.py [--xodr <xodr_path>]

Example: Finding forward waypoint (NO BACKTRACKING):
    from get_map_bounds import find_forward_waypoint, find_best_racing_waypoint
    
    # Car's current position (out of bounds)
    car_pos = (100.5, 200.3)
    car_heading = 1.57  # radians (optional, improves selection)
    last_wp_idx = 5  # Last known waypoint index (REQUIRED for forward detection)
    
    # Your waypoints list (from self.waypoints or track.racingLine)
    waypoints = [(x1, y1), (x2, y2), ...]
    
    # RECOMMENDED: Simple forward-only waypoint (guaranteed no backtracking)
    result = find_forward_waypoint(
        car_pos, waypoints,
        last_known_index=last_wp_idx,  # REQUIRED
        car_heading=car_heading,       # Optional but recommended
        max_search_distance=100.0
    )
    
    # ALTERNATIVE: Full control with forward_only=True (default)
    result = find_best_racing_waypoint(
        car_pos, car_heading, waypoints,
        last_known_index=last_wp_idx,
        forward_only=True,  # DEFAULT: Strictly no backtracking
        forward_bias=0.9,   # DEFAULT: Strong forward preference
        max_search_distance=100.0
    )
"""

import sys
import argparse
from pathlib import Path

# Add Scenic to path if needed
scenic_path = Path(__file__).parent.parent / "src"
if scenic_path.exists():
    sys.path.insert(0, str(scenic_path))


def get_map_bounds(xodr_path: str):
    """Extract and return the boundary points and bounding box of a map from an XODR file.
    
    Returns:
        Dictionary with:
        - 'boundary_points': List of (x, y) tuples defining the map boundary
        - 'xmin', 'ymin', 'xmax', 'ymax': Bounding box
        - 'width', 'height', 'center': Additional info
    """
    try:
        from scenic.domains.driving.roads import Network
        
        print(f"[INFO] Loading map from: {xodr_path}")
        network = Network.fromOpenDrive(xodr_path, ref_points=50)
        
        # Get boundary points from the drivable region (most comprehensive)
        if network.drivableRegion:
            polygons = network.drivableRegion.polygons
            boundary_points = []
            
            # Extract exterior coordinates from all polygons
            if hasattr(polygons, 'geoms'):
                # MultiPolygon - iterate through each polygon
                for geom in polygons.geoms:
                    if hasattr(geom, 'exterior'):
                        coords = list(geom.exterior.coords)
                        boundary_points.extend([(float(x), float(y)) for x, y in coords])
            else:
                # Single Polygon
                if hasattr(polygons, 'exterior'):
                    coords = list(polygons.exterior.coords)
                    boundary_points = [(float(x), float(y)) for x, y in coords]
            
            # Also get bounding box
            bounds = polygons.bounds
            xmin, ymin, xmax, ymax = bounds
            
            width = xmax - xmin
            height = ymax - ymin
            center_x = (xmin + xmax) / 2
            center_y = (ymin + ymax) / 2
            
            return {
                'boundary_points': boundary_points,
                'xmin': xmin,
                'ymin': ymin,
                'xmax': xmax,
                'ymax': ymax,
                'width': width,
                'height': height,
                'center': (center_x, center_y)
            }
        else:
            print("[WARN] No drivableRegion found, trying roadRegion...")
            if network.roadRegion:
                polygons = network.roadRegion.polygons
                boundary_points = []
                
                # Extract exterior coordinates from all polygons
                if hasattr(polygons, 'geoms'):
                    # MultiPolygon - iterate through each polygon
                    for geom in polygons.geoms:
                        if hasattr(geom, 'exterior'):
                            coords = list(geom.exterior.coords)
                            boundary_points.extend([(float(x), float(y)) for x, y in coords])
                else:
                    # Single Polygon
                    if hasattr(polygons, 'exterior'):
                        coords = list(polygons.exterior.coords)
                        boundary_points = [(float(x), float(y)) for x, y in coords]
                
                bounds = polygons.bounds
                xmin, ymin, xmax, ymax = bounds
                
                width = xmax - xmin
                height = ymax - ymin
                center_x = (xmin + xmax) / 2
                center_y = (ymin + ymax) / 2
                
                return {
                    'boundary_points': boundary_points,
                    'xmin': xmin,
                    'ymin': ymin,
                    'xmax': xmax,
                    'ymax': ymax,
                    'width': width,
                    'height': height,
                    'center': (center_x, center_y)
                }
            else:
                print("[ERROR] Could not find any region with bounds")
                return None
                
    except Exception as e:
        print(f"[ERROR] Failed to extract map bounds: {e}")
        import traceback
        traceback.print_exc()
        return None


def find_closest_waypoint(car_position: tuple, waypoints: list, search_all: bool = False, 
                          last_known_index: int = 0, search_window: int = 25):
    """Find the closest waypoint to a car's position.
    
    This is useful when the car is out of bounds and you need to find the nearest
    waypoint to navigate back to the track.
    
    Args:
        car_position: Tuple of (x, y) coordinates of the car's position
        waypoints: List of (x, y) tuples representing waypoints
        search_all: If True, search all waypoints. If False, use windowed search
                    around last_known_index (more efficient when car is on track)
        last_known_index: Last known waypoint index (used for windowed search)
        search_window: Size of search window around last_known_index (default: 25)
    
    Returns:
        Dictionary with:
        - 'index': Index of the closest waypoint
        - 'waypoint': (x, y) coordinates of the closest waypoint
        - 'distance': Distance in meters to the closest waypoint
        - 'next_index': Index of the next waypoint (for navigation direction)
        - 'next_waypoint': (x, y) coordinates of the next waypoint
    """
    if not waypoints or len(waypoints) == 0:
        return None
    
    car_x, car_y = float(car_position[0]), float(car_position[1])
    
    # Determine search range
    if search_all or len(waypoints) <= search_window * 2:
        # Search all waypoints (useful when out of bounds)
        start_idx = 0
        end_idx = len(waypoints)
    else:
        # Windowed search (more efficient when on track)
        start_idx = max(0, last_known_index - search_window)
        end_idx = min(len(waypoints), last_known_index + search_window + 1)
    
    # Find closest waypoint
    nearest_idx = 0
    best_distance_sq = float('inf')
    
    for i in range(start_idx, end_idx):
        wp_x, wp_y = float(waypoints[i][0]), float(waypoints[i][1])
        dx = car_x - wp_x
        dy = car_y - wp_y
        distance_sq = dx * dx + dy * dy
        
        if distance_sq < best_distance_sq:
            best_distance_sq = distance_sq
            nearest_idx = i
    
    # Calculate actual distance
    distance = (best_distance_sq ** 0.5)
    
    # Get next waypoint (for navigation direction)
    next_idx = (nearest_idx + 1) % len(waypoints)
    next_waypoint = waypoints[next_idx]
    
    return {
        'index': nearest_idx,
        'waypoint': waypoints[nearest_idx],
        'distance': distance,
        'next_index': next_idx,
        'next_waypoint': next_waypoint
    }


def find_best_racing_waypoint(car_position: tuple, car_heading: float, waypoints: list,
                               last_known_index: int = 0, max_search_distance: float = 100.0,
                               forward_bias: float = 0.9, min_forward_distance: float = 5.0,
                               forward_only: bool = True):
    """Find the best waypoint for racing when car is out of bounds.
    
    This function considers racing strategy, not just closest distance:
    - ALWAYS avoids backtracking when forward_only=True (default)
    - Prefers waypoints that are forward along the track (not backward)
    - Considers car's heading to avoid sharp turns
    - Balances distance vs forward progress
    
    Args:
        car_position: Tuple of (x, y) coordinates of the car's position
        car_heading: Car's heading in radians (0 = east, π/2 = north, etc.)
                     If None, will use waypoint direction instead
        waypoints: List of (x, y) tuples representing waypoints
        last_known_index: Last known waypoint index (helps determine forward direction)
        max_search_distance: Maximum distance to search for waypoints (meters)
        forward_bias: Weight for forward progress vs distance (0.0-1.0, higher = more forward bias)
                      Default: 0.9 (strong forward preference)
        min_forward_distance: Minimum distance forward to consider a waypoint "forward" (meters)
        forward_only: If True (default), ONLY consider forward waypoints (strictly no backtracking)
                      If False, backward waypoints are penalized but not excluded
    
    Returns:
        Dictionary with:
        - 'index': Index of the best waypoint
        - 'waypoint': (x, y) coordinates of the best waypoint
        - 'distance': Distance in meters to the waypoint
        - 'forward_score': Score indicating how forward this waypoint is (0-1)
        - 'heading_alignment': How well the waypoint aligns with car heading (0-1)
        - 'total_score': Combined score used for selection
        - 'next_index': Index of the next waypoint
        - 'next_waypoint': (x, y) coordinates of the next waypoint
    """
    import math
    
    if not waypoints or len(waypoints) == 0:
        return None
    
    if len(waypoints) < 2:
        # Only one waypoint, return it
        return {
            'index': 0,
            'waypoint': waypoints[0],
            'distance': math.sqrt(
                (car_position[0] - waypoints[0][0])**2 + 
                (car_position[1] - waypoints[0][1])**2
            ),
            'forward_score': 0.5,
            'heading_alignment': 0.5,
            'total_score': 0.5,
            'next_index': 0,
            'next_waypoint': waypoints[0]
        }
    
    car_x, car_y = float(car_position[0]), float(car_position[1])
    
    # Calculate waypoint directions (forward direction along track)
    waypoint_directions = []
    for i in range(len(waypoints)):
        next_i = (i + 1) % len(waypoints)
        wp_x, wp_y = float(waypoints[i][0]), float(waypoints[i][1])
        next_wp_x, next_wp_y = float(waypoints[next_i][0]), float(waypoints[next_i][1])
        
        dx = next_wp_x - wp_x
        dy = next_wp_y - wp_y
        length = math.sqrt(dx*dx + dy*dy)
        if length > 1e-6:
            waypoint_directions.append(math.atan2(dy, dx))
        else:
            waypoint_directions.append(0.0)
    
    # Determine forward direction based on last known index
    # Waypoints ahead of last_known_index are considered "forward"
    forward_threshold_idx = last_known_index
    
    best_score = -float('inf')
    best_idx = 0
    best_distance = float('inf')
    best_forward_score = 0.0
    best_heading_align = 0.0
    
    # Search all waypoints within max_search_distance
    for i in range(len(waypoints)):
        wp_x, wp_y = float(waypoints[i][0]), float(waypoints[i][1])
        dx = car_x - wp_x
        dy = car_y - wp_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Skip if too far
        if distance > max_search_distance:
            continue
        
        # Check if waypoint is forward or backward
        is_forward = False
        if i >= forward_threshold_idx:
            # Forward waypoint (normal case: i is ahead of last known)
            is_forward = True
        else:
            # Waypoint index is less than last known - could be backward or wrapped
            # Check if we've wrapped around (closed loop track)
            # Only consider start waypoints as forward if we're very near the end
            # AND the waypoint is near the start (within reasonable wrap distance)
            if forward_threshold_idx > len(waypoints) * 0.8:
                # We're near end of track (last 20%)
                # Only waypoints in first 20% can be considered forward (wrapped)
                wrap_threshold = len(waypoints) * 0.2
                if i < wrap_threshold:
                    # Waypoint is in first 20%, could be forward (wrapped)
                    # But we need to ensure it's actually ahead in track distance
                    # For safety, only consider it forward if it's very close to start
                    # (i.e., we've definitely wrapped around)
                    is_forward = True
                else:
                    # Waypoint is in middle section, definitely backward
                    is_forward = False
            else:
                # Not near end of track, so i < forward_threshold_idx means backward
                is_forward = False
        
        # STRICT FORWARD-ONLY: Skip backward waypoints completely
        if forward_only and not is_forward:
            continue
        
        # Calculate forward score (0-1)
        # Waypoints ahead of last known position get higher scores
        forward_score = 0.0
        if is_forward:
            # Forward waypoint
            if i >= forward_threshold_idx:
                idx_diff = i - forward_threshold_idx
            else:
                # Wrapped around case
                idx_diff = len(waypoints) - forward_threshold_idx + i
            # Normalize: waypoints just ahead get score 1.0, very far ahead get lower
            forward_score = 1.0 / (1.0 + idx_diff * 0.1)  # Decay with distance
        else:
            # Backward waypoint (only considered if forward_only=False)
            backward_penalty = (forward_threshold_idx - i) * 0.2
            forward_score = max(0.0, 0.3 - backward_penalty)
        
        # Calculate heading alignment (0-1)
        # How well does the waypoint direction align with car heading?
        heading_align = 0.5  # Default neutral
        if car_heading is not None:
            wp_dir = waypoint_directions[i]
            # Calculate angle difference
            angle_diff = abs(car_heading - wp_dir)
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
            # Convert to score: 0° = 1.0, 90° = 0.0, 180° = 0.0
            heading_align = max(0.0, 1.0 - (angle_diff / (math.pi / 2)))
        else:
            # No heading available, use direction from car to waypoint
            to_wp_angle = math.atan2(wp_y - car_y, wp_x - car_x)
            wp_dir = waypoint_directions[i]
            angle_diff = abs(to_wp_angle - wp_dir)
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
            heading_align = max(0.0, 1.0 - (angle_diff / (math.pi / 2)))
        
        # Distance score (closer is better, normalized 0-1)
        distance_score = 1.0 / (1.0 + distance / 10.0)  # Decay with distance
        
        # Combined score: weighted combination
        # Higher forward_bias means we care more about forward progress
        total_score = (
            (1.0 - forward_bias) * distance_score +
            forward_bias * forward_score * 0.7 +
            forward_bias * heading_align * 0.3
        )
        
        # Prefer waypoints that are at least min_forward_distance ahead
        if i >= forward_threshold_idx:
            # Calculate forward distance along track
            forward_dist = 0.0
            for j in range(forward_threshold_idx, i):
                curr_wp = waypoints[j]
                next_wp = waypoints[(j + 1) % len(waypoints)]
                seg_dx = next_wp[0] - curr_wp[0]
                seg_dy = next_wp[1] - curr_wp[1]
                forward_dist += math.sqrt(seg_dx*seg_dx + seg_dy*seg_dy)
            
            if forward_dist < min_forward_distance:
                # Too close forward, reduce score slightly
                total_score *= 0.8
        
        if total_score > best_score:
            best_score = total_score
            best_idx = i
            best_distance = distance
            best_forward_score = forward_score
            best_heading_align = heading_align
    
    # If no waypoint found within max_search_distance
    if best_score == -float('inf'):
        if forward_only:
            # Try with larger search distance for forward-only mode
            # This handles cases where all forward waypoints are far
            extended_result = find_best_racing_waypoint(
                car_position, car_heading, waypoints,
                last_known_index, max_search_distance * 2.0,
                forward_bias, min_forward_distance, forward_only=True
            )
            if extended_result:
                return extended_result
            # If still no result, we have no forward waypoints - return None
            return None
        else:
            # Fall back to closest waypoint (may be backward)
            return find_closest_waypoint(car_position, waypoints, search_all=True)
    
    # Get next waypoint
    next_idx = (best_idx + 1) % len(waypoints)
    
    return {
        'index': best_idx,
        'waypoint': waypoints[best_idx],
        'distance': best_distance,
        'forward_score': best_forward_score,
        'heading_alignment': best_heading_align,
        'total_score': best_score,
        'next_index': next_idx,
        'next_waypoint': waypoints[next_idx]
    }


def find_forward_waypoint(car_position: tuple, waypoints: list, last_known_index: int = 0,
                          car_heading: float = None, max_search_distance: float = 100.0):
    """Find the closest forward waypoint (guaranteed no backtracking).
    
    This is a simplified wrapper around find_best_racing_waypoint() that
    ALWAYS returns a forward waypoint and NEVER allows backtracking.
    
    Args:
        car_position: Tuple of (x, y) coordinates of the car's position
        waypoints: List of (x, y) tuples representing waypoints
        last_known_index: Last known waypoint index (required for forward detection)
        car_heading: Optional car heading in radians (improves selection quality)
        max_search_distance: Maximum distance to search (meters)
    
    Returns:
        Dictionary with waypoint information (same format as find_best_racing_waypoint)
        Returns None if no forward waypoint found within max_search_distance
    """
    return find_best_racing_waypoint(
        car_position=car_position,
        car_heading=car_heading,
        waypoints=waypoints,
        last_known_index=last_known_index,
        max_search_distance=max_search_distance,
        forward_bias=0.9,  # Strong forward preference
        min_forward_distance=5.0,
        forward_only=True  # STRICT: No backtracking allowed
    )


def check_position_in_bounds(position: tuple, bounds: dict, tolerance: float = 0.0):
    """Check if a position is within the map bounds.
    
    Args:
        position: Tuple of (x, y) coordinates
        bounds: Dictionary from get_map_bounds() containing 'xmin', 'ymin', 'xmax', 'ymax'
        tolerance: Optional tolerance in meters (allows positions slightly outside bounds)
    
    Returns:
        bool: True if position is within bounds (including tolerance)
    """
    x, y = float(position[0]), float(position[1])
    
    return (bounds['xmin'] - tolerance <= x <= bounds['xmax'] + tolerance and
            bounds['ymin'] - tolerance <= y <= bounds['ymax'] + tolerance)


def main():
    parser = argparse.ArgumentParser(
        description='Get the boundary points and bounding box of a map from an XODR file',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('--xodr', type=str, default=None,
                       help='Path to XODR track file (default: auto-detect Laguna Seca)')
    parser.add_argument('--output', type=str, default=None,
                       help='Output file to save boundary points (CSV or Python format)')
    parser.add_argument('--format', type=str, default='python', choices=['python', 'csv', 'json'],
                       help='Output format: python (list of tuples), csv, or json (default: python)')
    
    args = parser.parse_args()
    
    # Auto-detect Laguna Seca XODR if not specified
    if args.xodr is None:
        # Get project root (parent of tools directory)
        project_root = Path(__file__).parent.parent
        default_xodr = project_root / 'assets' / 'maps' / 'dSPACE' / 'LagunaSeca.xodr'
        if default_xodr.exists():
            args.xodr = str(default_xodr)
            print(f"[INFO] Auto-detected track file: {args.xodr}")
        else:
            print(f"[ERROR] Default map not found: {default_xodr}")
            print("       Please specify --xodr <path>")
            sys.exit(1)
    
    if not Path(args.xodr).exists():
        print(f"[ERROR] Map file not found: {args.xodr}")
        sys.exit(1)
    
    bounds = get_map_bounds(args.xodr)
    
    if bounds:
        print("\n" + "="*60)
        print("MAP BOUNDS")
        print("="*60)
        print(f"X range: [{bounds['xmin']:.2f}, {bounds['xmax']:.2f}] meters")
        print(f"Y range: [{bounds['ymin']:.2f}, {bounds['ymax']:.2f}] meters")
        print(f"Width:   {bounds['width']:.2f} meters")
        print(f"Height:  {bounds['height']:.2f} meters")
        print(f"Center:  ({bounds['center'][0]:.2f}, {bounds['center'][1]:.2f})")
        print("="*60)
        
        boundary_points = bounds.get('boundary_points', [])
        if boundary_points:
            print(f"\nBOUNDARY POINTS ({len(boundary_points)} points):")
            print("-" * 60)
            
            # Print first few and last few points
            if len(boundary_points) <= 20:
                for i, (x, y) in enumerate(boundary_points):
                    print(f"  [{i:4d}] ({x:12.6f}, {y:12.6f})")
            else:
                print("  First 10 points:")
                for i, (x, y) in enumerate(boundary_points[:10]):
                    print(f"  [{i:4d}] ({x:12.6f}, {y:12.6f})")
                print(f"  ... ({len(boundary_points) - 20} more points) ...")
                print("  Last 10 points:")
                for i, (x, y) in enumerate(boundary_points[-10:], start=len(boundary_points)-10):
                    print(f"  [{i:4d}] ({x:12.6f}, {y:12.6f})")
            
            print("\nPython list format:")
            print("boundary_points = [")
            if len(boundary_points) <= 50:
                for x, y in boundary_points:
                    print(f"    ({x:.6f}, {y:.6f}),")
            else:
                print("    # First 5 points:")
                for x, y in boundary_points[:5]:
                    print(f"    ({x:.6f}, {y:.6f}),")
                print(f"    # ... ({len(boundary_points) - 10} more points) ...")
                print("    # Last 5 points:")
                for x, y in boundary_points[-5:]:
                    print(f"    ({x:.6f}, {y:.6f}),")
            print("]")
        else:
            print("\n[WARN] No boundary points extracted")
        
        print("\nBounding box (Python format):")
        print(f"  xmin={bounds['xmin']:.6f}, ymin={bounds['ymin']:.6f}")
        print(f"  xmax={bounds['xmax']:.6f}, ymax={bounds['ymax']:.6f}")
        
        # Save to file if requested
        if args.output:
            boundary_points = bounds.get('boundary_points', [])
            if boundary_points:
                output_path = Path(args.output)
                output_path.parent.mkdir(parents=True, exist_ok=True)
                
                if args.format == 'csv':
                    import csv
                    with open(output_path, 'w', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow(['x', 'y'])  # Header
                        for x, y in boundary_points:
                            writer.writerow([x, y])
                    print(f"\n[INFO] Saved {len(boundary_points)} boundary points to CSV: {output_path}")
                    
                elif args.format == 'json':
                    import json
                    with open(output_path, 'w') as f:
                        json.dump({
                            'boundary_points': boundary_points,
                            'bounds': {
                                'xmin': bounds['xmin'],
                                'ymin': bounds['ymin'],
                                'xmax': bounds['xmax'],
                                'ymax': bounds['ymax']
                            }
                        }, f, indent=2)
                    print(f"\n[INFO] Saved boundary points and bounds to JSON: {output_path}")
                    
                else:  # python format
                    with open(output_path, 'w') as f:
                        f.write("# Boundary points for map\n")
                        f.write(f"# Total points: {len(boundary_points)}\n")
                        f.write(f"# Bounds: x=[{bounds['xmin']:.6f}, {bounds['xmax']:.6f}], y=[{bounds['ymin']:.6f}, {bounds['ymax']:.6f}]\n\n")
                        f.write("boundary_points = [\n")
                        for x, y in boundary_points:
                            f.write(f"    ({x:.6f}, {y:.6f}),\n")
                        f.write("]\n")
                    print(f"\n[INFO] Saved {len(boundary_points)} boundary points to Python file: {output_path}")
    else:
        print("[ERROR] Could not extract map bounds")
        sys.exit(1)


if __name__ == '__main__':
    main()

