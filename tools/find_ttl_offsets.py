#!/usr/bin/env python3
"""
Find optimal TTL coordinate offsets by testing different translations.

This tool tests various offset values to find which ones best align TTLs
with the track boundaries. It helps identify if the default offsets are correct
or need adjustment.

Usage:
    python find_ttl_offsets.py [--ttl <ttl_file>] [--xodr <xodr_path>] [--range <range>]
    
Examples:
    # Test default TTL with offset range
    python find_ttl_offsets.py --ttl assets/ttls/LS_ENU_TTL_CSV/usable/ttl_17.csv
    
    # Test with custom offset range
    python find_ttl_offsets.py --ttl assets/ttls/LS_ENU_TTL_CSV/usable/ttl_17.csv --range 20
"""

import os
import sys
import csv
import argparse
import matplotlib.pyplot as plt
from pathlib import Path
from typing import List, Tuple

# Add Scenic to path if needed
scenic_path = Path(__file__).parent.parent / "src"
if scenic_path.exists():
    sys.path.insert(0, str(scenic_path))


def read_ttl_xy(csv_path: str, dx: float = 0.0, dy: float = 0.0):
    """Read ENU x,y from TTL CSV."""
    points = []
    with open(csv_path, newline="") as f:
        r = csv.reader(f)
        try:
            next(r)  # skip metadata
        except StopIteration:
            pass
        for row in r:
            if not row or len(row) < 2:
                continue
            try:
                x = float(row[0]) + dx
                y = float(row[1]) + dy
                points.append((x, y))
            except Exception:
                continue
    return points


def load_track_boundaries(xodr_path: str):
    """Load track boundaries from XODR file.
    
    Returns:
        (left_boundary_points, right_boundary_points, road_region)
        road_region can be used to check if points are inside the track
    """
    try:
        from scenic.formats.opendrive import xodr_parser
        from scenic.domains.driving.roads import Network
        
        network = Network.fromOpenDrive(xodr_path, ref_points=50)
        
        left_points = []
        right_points = []
        road_polygons = []
        
        def extract_point_coords(pt):
            if isinstance(pt, tuple) or isinstance(pt, list):
                return (float(pt[0]), float(pt[1]))
            elif hasattr(pt, 'x') and hasattr(pt, 'y'):
                return (float(pt.x), float(pt.y))
            return None
        
        for road in network.roads:
            for lane in road.lanes:
                if hasattr(lane, 'leftEdge') and lane.leftEdge:
                    try:
                        if hasattr(lane.leftEdge, 'points'):
                            points_list = lane.leftEdge.points
                        elif hasattr(lane.leftEdge, '__iter__'):
                            points_list = list(lane.leftEdge)
                        else:
                            continue
                        
                        step = max(1, len(points_list) // 200)
                        for pt in points_list[::step]:
                            coords = extract_point_coords(pt)
                            if coords:
                                left_points.append(coords)
                    except:
                        pass
                
                if hasattr(lane, 'rightEdge') and lane.rightEdge:
                    try:
                        if hasattr(lane.rightEdge, 'points'):
                            points_list = lane.rightEdge.points
                        elif hasattr(lane.rightEdge, '__iter__'):
                            points_list = list(lane.rightEdge)
                        else:
                            continue
                        
                        step = max(1, len(points_list) // 200)
                        for pt in points_list[::step]:
                            coords = extract_point_coords(pt)
                            if coords:
                                right_points.append(coords)
                    except:
                        pass
        
        # Remove duplicates
        seen = set()
        unique_left = []
        for p in left_points:
            p_tuple = (round(p[0], 2), round(p[1], 2))
            if p_tuple not in seen:
                seen.add(p_tuple)
                unique_left.append(p)
        
        seen = set()
        unique_right = []
        for p in right_points:
            p_tuple = (round(p[0], 2), round(p[1], 2))
            if p_tuple not in seen:
                seen.add(p_tuple)
                unique_right.append(p)
        
        # Also collect road polygons for containment checking
        for road in network.roads:
            if hasattr(road, 'polygon') and road.polygon:
                try:
                    # Convert to shapely polygon if possible
                    if hasattr(road.polygon, 'exterior'):
                        road_polygons.append(road.polygon)
                    elif hasattr(road, 'footprint') and hasattr(road.footprint, 'polygons'):
                        road_polygons.append(road.footprint.polygons)
                except:
                    pass
        
        return unique_left, unique_right, road_polygons
    except Exception as e:
        print(f"[ERROR] Could not load track: {e}")
        import traceback
        traceback.print_exc()
        return [], [], []


def point_to_line_distance(px: float, py: float, 
                          line_points: List[Tuple[float, float]]) -> float:
    """Compute minimum distance from point to line segment."""
    if not line_points:
        return float('inf')
    
    min_dist = float('inf')
    for i in range(len(line_points) - 1):
        x1, y1 = line_points[i]
        x2, y2 = line_points[i + 1]
        
        # Vector from point to line segment
        dx = x2 - x1
        dy = y2 - y1
        seg_len_sq = dx*dx + dy*dy
        
        if seg_len_sq < 1e-6:
            # Degenerate segment
            dist = ((px - x1)**2 + (py - y1)**2)**0.5
        else:
            # Project point onto line segment
            t = max(0, min(1, ((px - x1)*dx + (py - y1)*dy) / seg_len_sq))
            proj_x = x1 + t * dx
            proj_y = y1 + t * dy
            dist = ((px - proj_x)**2 + (py - proj_y)**2)**0.5
        
        min_dist = min(min_dist, dist)
    
    return min_dist


def is_point_inside_road(px: float, py: float, road_polygons) -> bool:
    """Check if point is inside road polygon(s)."""
    if not road_polygons:
        return False
    
    try:
        import shapely.geometry
        point = shapely.geometry.Point(px, py)
        
        # Check if point is in any road polygon
        for poly in road_polygons:
            if hasattr(poly, 'contains'):
                if poly.contains(point):
                    return True
            elif hasattr(poly, 'geoms'):
                # MultiPolygon
                for geom in poly.geoms:
                    if geom.contains(point):
                        return True
    except:
        pass
    
    return False


def is_point_between_boundaries(px: float, py: float,
                                left_boundary: List[Tuple[float, float]],
                                right_boundary: List[Tuple[float, float]],
                                road_polygons=None,
                                tolerance: float = 2.0) -> bool:
    """Check if point is between left and right boundaries or inside road polygon.
    
    Uses road polygon if available (more accurate), otherwise falls back to boundary distance check.
    """
    # First try road polygon (most accurate)
    if road_polygons:
        if is_point_inside_road(px, py, road_polygons):
            return True
    
    # Fallback to boundary distance check
    if not left_boundary or not right_boundary:
        return False
    
    # Sample boundaries for faster computation
    left_sample = left_boundary[::max(1, len(left_boundary)//50)]
    right_sample = right_boundary[::max(1, len(right_boundary)//50)]
    
    dist_left = point_to_line_distance(px, py, left_sample)
    dist_right = point_to_line_distance(px, py, right_sample)
    
    # Point is "between" if it's reasonably close to both boundaries
    # Track width is typically 10-12m, so being within 15m of boundaries is reasonable
    max_dist = max(dist_left, dist_right)
    return max_dist < 15.0


def compute_alignment_score(ttl_points: List[Tuple[float, float]], 
                            left_boundary: List[Tuple[float, float]],
                            right_boundary: List[Tuple[float, float]],
                            road_polygons=None,
                            sample_rate: int = 50) -> float:
    """Compute how well TTL points align with track boundaries.
    
    Returns a score where lower is better.
    """
    if not ttl_points or not left_boundary or not right_boundary:
        return float('inf')
    
    # Sample TTL points
    sampled_ttl = ttl_points[::sample_rate] if len(ttl_points) > sample_rate else ttl_points
    
    total_distance = 0.0
    points_inside = 0
    points_outside = 0
    
    for tx, ty in sampled_ttl:
        # Compute distance to boundaries
        dist_left = point_to_line_distance(tx, ty, left_boundary[::max(1, len(left_boundary)//100)])
        dist_right = point_to_line_distance(tx, ty, right_boundary[::max(1, len(right_boundary)//100)])
        
        # Check if point is between boundaries or inside road polygon
        is_inside = is_point_between_boundaries(tx, ty, left_boundary, right_boundary, road_polygons)
        
        if is_inside:
            points_inside += 1
            # For inside points, prefer being near center (not too close to either boundary)
            # Ideal distance from each boundary is ~3-5m (half track width)
            center_dist = abs(dist_left - dist_right)  # How far from center
            total_distance += center_dist * 0.1  # Small penalty for being off-center
        else:
            points_outside += 1
            # For outside points, heavily penalize distance from nearest boundary
            min_dist = min(dist_left, dist_right)
            total_distance += min_dist * 5.0  # Heavy penalty for being outside
    
    # Score: distance penalty + outside penalty
    avg_distance = total_distance / len(sampled_ttl) if sampled_ttl else float('inf')
    outside_ratio = points_outside / len(sampled_ttl) if sampled_ttl else 1.0
    outside_penalty = outside_ratio * 50.0  # Heavy penalty for points outside
    
    return avg_distance + outside_penalty


def test_offsets(ttl_path: str, xodr_path: str, 
                 dx_range: Tuple[float, float], dy_range: Tuple[float, float],
                 step: float = 2.0, top_n: int = 5):
    """Test different offset combinations and find the best ones."""
    print(f"[INFO] Loading TTL from: {ttl_path}")
    base_ttl = read_ttl_xy(ttl_path, dx=0, dy=0)
    print(f"[INFO] Loaded {len(base_ttl)} TTL points")
    
    print(f"[INFO] Loading track boundaries from: {xodr_path}")
    left_boundary, right_boundary, road_polygons = load_track_boundaries(xodr_path)
    print(f"[INFO] Loaded {len(left_boundary)} left and {len(right_boundary)} right boundary points")
    if road_polygons:
        print(f"[INFO] Loaded {len(road_polygons)} road polygon(s) for containment checking")
    
    if not base_ttl or not left_boundary or not right_boundary:
        print("[ERROR] Failed to load required data")
        return
    
    print(f"\n[INFO] Testing offsets:")
    print(f"  DX range: {dx_range[0]:.1f} to {dx_range[1]:.1f} (step: {step:.1f})")
    print(f"  DY range: {dy_range[0]:.1f} to {dy_range[1]:.1f} (step: {step:.1f})")
    
    results = []
    dx_start, dx_end = dx_range
    dy_start, dy_end = dy_range
    
    dx = dx_start
    test_count = 0
    while dx <= dx_end:
        dy = dy_start
        while dy <= dy_end:
            test_count += 1
            if test_count % 10 == 0:
                print(f"  Testing offset ({dx:.1f}, {dy:.1f})... ({test_count} tests)")
            
            # Apply offset and compute alignment
            ttl_points = [(x + dx, y + dy) for x, y in base_ttl]
            score = compute_alignment_score(ttl_points, left_boundary, right_boundary, road_polygons)
            
            results.append({
                'dx': dx,
                'dy': dy,
                'score': score
            })
            
            dy += step
        dx += step
    
    # Sort by score (lower is better)
    results.sort(key=lambda x: x['score'])
    
    print(f"\n[INFO] Tested {len(results)} offset combinations")
    print(f"\n[RESULTS] Top {top_n} best offsets (lower score = better alignment):")
    print(f"{'Rank':<6} {'DX':<10} {'DY':<10} {'Score':<12} {'Description'}")
    print("-" * 70)
    
    for i, result in enumerate(results[:top_n]):
        dx, dy = result['dx'], result['dy']
        score = result['score']
        
        # Compare to default
        default_dx, default_dy = -53.6, -15.7
        dx_diff = dx - default_dx
        dy_diff = dy - default_dy
        
        desc = f"Default + ({dx_diff:+.1f}, {dy_diff:+.1f})"
        if i == 0:
            desc += " [BEST]"
        
        print(f"{i+1:<6} {dx:<10.1f} {dy:<10.1f} {score:<12.3f} {desc}")
    
    # Visualize best offset
    best = results[0]
    print(f"\n[INFO] Visualizing best offset: dx={best['dx']:.1f}, dy={best['dy']:.1f}")
    visualize_alignment(ttl_path, xodr_path, best['dx'], best['dy'], 
                       left_boundary, right_boundary, road_polygons, 
                       save_path='best_ttl_alignment.png')


def visualize_alignment(ttl_path: str, xodr_path: str, dx: float, dy: float,
                        left_boundary: List[Tuple], right_boundary: List[Tuple],
                        road_polygons=None, save_path: str = None):
    """Visualize TTL alignment with track boundaries."""
    ttl_points = read_ttl_xy(ttl_path, dx=dx, dy=dy)
    
    fig, ax = plt.subplots(figsize=(14, 12))
    
    # Plot boundaries
    if left_boundary:
        left_x = [p[0] for p in left_boundary]
        left_y = [p[1] for p in left_boundary]
        ax.plot(left_x, left_y, 'gray', linewidth=2.5, alpha=0.7, 
               label='Track Left Boundary', zorder=1)
    
    if right_boundary:
        right_x = [p[0] for p in right_boundary]
        right_y = [p[1] for p in right_boundary]
        ax.plot(right_x, right_y, 'gray', linewidth=2.5, alpha=0.7, 
               label='Track Right Boundary', zorder=1)
    
    # Plot TTL
    if ttl_points:
        ttl_x = [p[0] for p in ttl_points]
        ttl_y = [p[1] for p in ttl_points]
        ax.plot(ttl_x, ttl_y, 'red', linewidth=2.5, alpha=0.9, 
               label=f'TTL (dx={dx:.1f}, dy={dy:.1f})', zorder=2)
        
        # Mark start
        ax.scatter([ttl_x[0]], [ttl_y[0]], c='green', s=100, 
                  marker='o', zorder=10, edgecolors='black', linewidths=2)
    
    ax.set_xlabel('X (East, meters)', fontsize=12)
    ax.set_ylabel('Y (North, meters)', fontsize=12)
    ax.set_title(f'TTL Alignment Test: dx={dx:.1f}, dy={dy:.1f}', fontsize=14, fontweight='bold')
    ax.legend(loc='best', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal', adjustable='box')
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"[INFO] Saved visualization to: {save_path}")
    
    plt.close()


def main():
    parser = argparse.ArgumentParser(
        description='Find optimal TTL coordinate offsets',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('--ttl', type=str,
                       default='assets/ttls/LS_ENU_TTL_CSV/usable/ttl_17.csv',
                       help='Path to TTL CSV file')
    parser.add_argument('--xodr', type=str, default=None,
                       help='Path to XODR track file (default: auto-detect)')
    parser.add_argument('--range', type=float, default=20.0,
                       help='Offset search range around default values (default: 20.0)')
    parser.add_argument('--step', type=float, default=2.0,
                       help='Step size for offset testing (default: 2.0)')
    parser.add_argument('--dx-default', type=float, default=-53.6,
                       help='Default DX offset (default: -53.6)')
    parser.add_argument('--dy-default', type=float, default=-15.7,
                       help='Default DY offset (default: -15.7)')
    
    args = parser.parse_args()
    
    # Auto-detect XODR if not specified
    if args.xodr is None:
        default_xodr = Path('assets/maps/dSPACE/LagunaSeca.xodr')
        if default_xodr.exists():
            args.xodr = str(default_xodr)
        else:
            print("[ERROR] XODR file not found. Please specify --xodr")
            sys.exit(1)
    
    if not os.path.exists(args.ttl):
        print(f"[ERROR] TTL file not found: {args.ttl}")
        sys.exit(1)
    
    if not os.path.exists(args.xodr):
        print(f"[ERROR] XODR file not found: {args.xodr}")
        sys.exit(1)
    
    # Define search range around defaults
    dx_range = (args.dx_default - args.range, args.dx_default + args.range)
    dy_range = (args.dy_default - args.range, args.dy_default + args.range)
    
    test_offsets(args.ttl, args.xodr, dx_range, dy_range, 
                step=args.step, top_n=10)


if __name__ == "__main__":
    main()

