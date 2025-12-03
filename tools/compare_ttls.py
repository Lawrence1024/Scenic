#!/usr/bin/env python3
"""
Compare and validate multiple TTL CSV files.

This tool:
1. Loads all TTL CSV files from a directory
2. Compares waypoints to detect duplicates
3. Validates file structure and metadata
4. Visualizes all TTLs together for comparison
5. Reports statistics and differences

Usage:
    python compare_ttls.py [--dir <directory>] [--xodr <xodr_path>] [--dx <dx>] [--dy <dy>]
    
Examples:
    # Compare all TTLs in the default directory
    python compare_ttls.py
    
    # Compare TTLs in a specific directory
    python compare_ttls.py --dir assets/ttls/LS_ENU_TTL_CSV/usable
    
    # Compare with track overlay
    python compare_ttls.py --xodr assets/maps/dSPACE/LagunaSeca.xodr
"""

import os
import sys
import csv
import argparse
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from pathlib import Path
from typing import List, Tuple, Dict, Set
from collections import defaultdict

# Add Scenic to path if needed
scenic_path = Path(__file__).parent.parent / "src"
if scenic_path.exists():
    sys.path.insert(0, str(scenic_path))


def read_ttl_xy(csv_path: str, dx: float = 0.0, dy: float = 0.0):
    """Read ENU x,y from TTL CSV. Skips the first metadata line.
    
    Returns:
        List of (x, y) tuples and metadata dict
    """
    points = []
    metadata = {}
    
    with open(csv_path, newline="") as f:
        reader = csv.reader(f)
        # Read first line (metadata)
        try:
            meta_row = next(reader)
            if len(meta_row) >= 6:
                metadata = {
                    'id': meta_row[0] if len(meta_row) > 0 else None,
                    'num_points': int(meta_row[1]) if len(meta_row) > 1 else None,
                    'length': float(meta_row[2]) if len(meta_row) > 2 else None,
                    'lat': float(meta_row[3]) if len(meta_row) > 3 else None,
                    'lon': float(meta_row[4]) if len(meta_row) > 4 else None,
                    'elevation': float(meta_row[5]) if len(meta_row) > 5 else None,
                }
        except StopIteration:
            pass
        
        # Read waypoint data
        for i, row in enumerate(reader):
            if not row or len(row) < 2:
                continue
            try:
                x = float(row[0]) + dx
                y = float(row[1]) + dy
                points.append((x, y))
            except Exception:
                if i < 5:  # Only print first few errors
                    print(f"[WARN] {os.path.basename(csv_path)}: Skipping malformed row {i+2}: {row}")
    
    return points, metadata


def compute_path_length(points: List[Tuple[float, float]]) -> float:
    """Compute total path length by summing segment distances."""
    if len(points) < 2:
        return 0.0
    
    total = 0.0
    for i in range(len(points) - 1):
        x1, y1 = points[i]
        x2, y2 = points[i + 1]
        dist = ((x2 - x1)**2 + (y2 - y1)**2)**0.5
        total += dist
    
    return total


def compare_waypoints(pts1: List[Tuple[float, float]], 
                      pts2: List[Tuple[float, float]], 
                      tolerance: float = 1.0) -> Dict:
    """Compare two waypoint lists.
    
    Returns:
        Dict with comparison results
    """
    # Check if paths are identical (within tolerance)
    if len(pts1) != len(pts2):
        return {
            'identical': False,
            'same_length': False,
            'length_diff': abs(len(pts1) - len(pts2)),
            'max_distance': None,
            'mean_distance': None,
            'overlap_percent': None
        }
    
    # Compute distances between corresponding points
    distances = []
    for (x1, y1), (x2, y2) in zip(pts1, pts2):
        dist = ((x2 - x1)**2 + (y2 - y1)**2)**0.5
        distances.append(dist)
    
    max_dist = max(distances) if distances else 0.0
    mean_dist = sum(distances) / len(distances) if distances else 0.0
    
    # Check if identical within tolerance
    identical = max_dist < tolerance
    
    return {
        'identical': identical,
        'same_length': True,
        'length_diff': 0,
        'max_distance': max_dist,
        'mean_distance': mean_dist,
        'overlap_percent': 100.0 if identical else (100.0 * sum(1 for d in distances if d < tolerance) / len(distances))
    }


def find_duplicates(ttl_data: Dict[str, Tuple[List, Dict]]) -> Dict[str, List[str]]:
    """Find duplicate TTL files (identical waypoints).
    
    Returns:
        Dict mapping TTL name to list of duplicate names
    """
    duplicates = defaultdict(list)
    ttl_names = list(ttl_data.keys())
    
    for i, name1 in enumerate(ttl_names):
        pts1, _ = ttl_data[name1]
        for name2 in ttl_names[i+1:]:
            pts2, _ = ttl_data[name2]
            comparison = compare_waypoints(pts1, pts2, tolerance=0.1)
            if comparison['identical']:
                duplicates[name1].append(name2)
                duplicates[name2].append(name1)
    
    return dict(duplicates)


def validate_ttl(points: List[Tuple[float, float]], metadata: Dict) -> Dict:
    """Validate TTL structure and compute statistics.
    
    Returns:
        Dict with validation results
    """
    issues = []
    stats = {}
    
    # Check point count
    if metadata.get('num_points'):
        expected = metadata['num_points']
        actual = len(points)
        if expected != actual:
            issues.append(f"Point count mismatch: metadata says {expected}, file has {actual}")
    
    # Check minimum points
    if len(points) < 2:
        issues.append("Too few points (need at least 2)")
    
    # Check for duplicate consecutive points
    duplicate_points = 0
    for i in range(len(points) - 1):
        if points[i] == points[i + 1]:
            duplicate_points += 1
    
    if duplicate_points > 0:
        issues.append(f"Found {duplicate_points} duplicate consecutive points")
    
    # Compute path length
    computed_length = compute_path_length(points)
    stats['computed_length'] = computed_length
    
    if metadata.get('length'):
        reported_length = metadata['length']
        length_diff = abs(computed_length - reported_length)
        if length_diff > 1.0:  # More than 1m difference
            issues.append(f"Length mismatch: metadata says {reported_length:.2f}m, computed {computed_length:.2f}m (diff: {length_diff:.2f}m)")
    
    # Check for very short segments (potential issues)
    short_segments = 0
    for i in range(len(points) - 1):
        x1, y1 = points[i]
        x2, y2 = points[i + 1]
        dist = ((x2 - x1)**2 + (y2 - y1)**2)**0.5
        if dist < 0.01:  # Less than 1cm
            short_segments += 1
    
    if short_segments > len(points) * 0.1:  # More than 10% are very short
        issues.append(f"Many very short segments ({short_segments}/{len(points)-1})")
    
    stats['duplicate_points'] = duplicate_points
    stats['short_segments'] = short_segments
    stats['valid'] = len(issues) == 0
    stats['issues'] = issues
    
    return stats


def visualize_all_ttls(ttl_data: Dict[str, Tuple[List, Dict]], 
                        xodr_path: str = None,
                        save_path: str = None,
                        show_plot: bool = True):
    """Visualize all TTLs together for comparison."""
    # Color palette for different TTLs - using distinct, easily distinguishable colors
    # These colors are chosen to be maximally distinct and colorblind-friendly
    distinct_colors = [
        '#FF0000',  # Red
        '#0000FF',  # Blue
        '#00FF00',  # Green
        '#FF00FF',  # Magenta
        '#FFA500',  # Orange
        '#00FFFF',  # Cyan
        '#800080',  # Purple
        '#FFC0CB',  # Pink
        '#FFFF00',  # Yellow
        '#A52A2A',  # Brown
        '#808080',  # Gray
        '#000080',  # Navy
    ]
    
    # Use distinct colors, cycling if we have more TTLs than colors
    colors = [distinct_colors[i % len(distinct_colors)] for i in range(len(ttl_data))]
    
    fig, ax = plt.subplots(figsize=(14, 12))
    
    # Load track boundaries if available
    track_centerline = None
    track_left_boundary = None
    track_right_boundary = None
    
    if xodr_path and os.path.exists(xodr_path):
        try:
            from scenic.formats.opendrive import xodr_parser
            from scenic.domains.driving.roads import Network
            
            print(f"[INFO] Loading track boundaries from: {xodr_path}")
            network = Network.fromOpenDrive(xodr_path, ref_points=50)
            
            # Collect centerline points
            centerline_points = []
            # Store boundaries per lane to preserve correct ordering
            left_boundary_segments = []
            right_boundary_segments = []
            # Store pit lane boundaries separately
            pit_left_boundary_segments = []
            pit_right_boundary_segments = []
            
            # Identify pit lane roads using similar logic to RacingTrack
            pit_lane_roads = set()
            pit_lane_pattern = "pit"  # Default pattern to match pit lane names
            
            # First pass: identify pit lane roads
            for road in network.roads:
                road_id = getattr(road, 'id', None)
                road_name = getattr(road, 'name', str(road))
                
                # Check if road name matches pit lane pattern
                if pit_lane_pattern.lower() in str(road_name).lower():
                    pit_lane_roads.add(road)
                    print(f"[INFO] Identified pit lane road: {road_name} (ID: {road_id})")
            
            def extract_point_coords(pt):
                """Extract (x, y) from point, handling both tuple and object formats."""
                if isinstance(pt, tuple) or isinstance(pt, list):
                    return (float(pt[0]), float(pt[1]))
                elif hasattr(pt, 'x') and hasattr(pt, 'y'):
                    return (float(pt.x), float(pt.y))
                else:
                    return None
            
            for road in network.roads:
                is_pit_lane = road in pit_lane_roads
                for lane in road.lanes:
                    # Centerline
                    if hasattr(lane, 'centerline') and lane.centerline:
                        try:
                            if hasattr(lane.centerline, 'points'):
                                points_list = lane.centerline.points
                            elif hasattr(lane.centerline, '__iter__'):
                                points_list = list(lane.centerline)
                            else:
                                points_list = []
                            
                            step = max(1, len(points_list) // 100)
                            for i, pt in enumerate(points_list[::step]):
                                coords = extract_point_coords(pt)
                                if coords:
                                    centerline_points.append(coords)
                        except Exception:
                            pass
                    
                    # Left boundary (if available) - store as separate segment per lane
                    if hasattr(lane, 'leftEdge') and lane.leftEdge:
                        try:
                            if hasattr(lane.leftEdge, 'points'):
                                points_list = lane.leftEdge.points
                            elif hasattr(lane.leftEdge, '__iter__'):
                                points_list = list(lane.leftEdge)
                            else:
                                points_list = []
                            
                            step = max(1, len(points_list) // 100)
                            segment_points = []
                            for i, pt in enumerate(points_list[::step]):
                                coords = extract_point_coords(pt)
                                if coords:
                                    segment_points.append(coords)
                            
                            if segment_points:
                                if is_pit_lane:
                                    pit_left_boundary_segments.append(segment_points)
                                else:
                                    left_boundary_segments.append(segment_points)
                        except Exception:
                            pass  # Skip if we can't extract left edge
                    
                    # Right boundary (if available) - store as separate segment per lane
                    if hasattr(lane, 'rightEdge') and lane.rightEdge:
                        try:
                            if hasattr(lane.rightEdge, 'points'):
                                points_list = lane.rightEdge.points
                            elif hasattr(lane.rightEdge, '__iter__'):
                                points_list = list(lane.rightEdge)
                            else:
                                points_list = []
                            
                            step = max(1, len(points_list) // 100)
                            segment_points = []
                            for i, pt in enumerate(points_list[::step]):
                                coords = extract_point_coords(pt)
                                if coords:
                                    segment_points.append(coords)
                            
                            if segment_points:
                                if is_pit_lane:
                                    pit_right_boundary_segments.append(segment_points)
                                else:
                                    right_boundary_segments.append(segment_points)
                        except Exception:
                            pass  # Skip if we can't extract right edge
            
            # Plot main track boundaries - plot each segment separately to preserve correct ordering
            # This prevents diagonal lines from connecting non-adjacent points
            left_label_added = False
            for segment in left_boundary_segments:
                if segment:
                    left_x = [p[0] for p in segment]
                    left_y = [p[1] for p in segment]
                    label = 'Track Left Boundary' if not left_label_added else None
                    ax.plot(left_x, left_y, color='gray', linewidth=2.5, alpha=0.7, 
                           label=label, zorder=1)
                    left_label_added = True
            
            # Plot right boundary segments
            right_label_added = False
            for segment in right_boundary_segments:
                if segment:
                    right_x = [p[0] for p in segment]
                    right_y = [p[1] for p in segment]
                    label = 'Track Right Boundary' if not right_label_added else None
                    ax.plot(right_x, right_y, color='gray', linewidth=2.5, alpha=0.7, 
                           label=label, zorder=1)
                    right_label_added = True
            
            # Plot pit lane boundaries with a different color/style
            pit_left_label_added = False
            for segment in pit_left_boundary_segments:
                if segment:
                    left_x = [p[0] for p in segment]
                    left_y = [p[1] for p in segment]
                    label = 'Pit Lane Left Boundary' if not pit_left_label_added else None
                    ax.plot(left_x, left_y, color='orange', linewidth=2.5, alpha=0.7, 
                           linestyle='--', label=label, zorder=1)
                    pit_left_label_added = True
            
            pit_right_label_added = False
            for segment in pit_right_boundary_segments:
                if segment:
                    right_x = [p[0] for p in segment]
                    right_y = [p[1] for p in segment]
                    label = 'Pit Lane Right Boundary' if not pit_right_label_added else None
                    ax.plot(right_x, right_y, color='orange', linewidth=2.5, alpha=0.7, 
                           linestyle='--', label=label, zorder=1)
                    pit_right_label_added = True
            
            if not left_boundary_segments and not right_boundary_segments and not pit_left_boundary_segments and not pit_right_boundary_segments:
                print(f"[WARN] Could not extract track boundaries from XODR")
        except Exception as e:
            print(f"[WARN] Could not load track boundaries: {e}")
            import traceback
            traceback.print_exc()
    
    # Plot each TTL
    for i, (name, (points, metadata)) in enumerate(ttl_data.items()):
        if not points:
            continue
        
        x_coords = [p[0] for p in points]
        y_coords = [p[1] for p in points]
        
        # Plot path with thicker, more visible lines
        ax.plot(x_coords, y_coords, '-', linewidth=2.5, alpha=0.9, 
               color=colors[i], label=name, zorder=2+i)
        
        # Mark start with larger, more visible markers
        if len(points) >= 1:
            ax.scatter([x_coords[0]], [y_coords[0]], c=[colors[i]], s=100, 
                      marker='o', zorder=10, edgecolors='black', linewidths=2)
    
    ax.set_xlabel('X (East, meters)', fontsize=12)
    ax.set_ylabel('Y (North, meters)', fontsize=12)
    ax.set_title(f'TTL Comparison: {len(ttl_data)} files', fontsize=14, fontweight='bold')
    ax.legend(loc='best', fontsize=9, ncol=2)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal', adjustable='box')
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"[INFO] Saved comparison plot to: {save_path}")
    
    if show_plot:
        plt.show()
    else:
        plt.close()


def main():
    parser = argparse.ArgumentParser(
        description='Compare and validate multiple TTL CSV files',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument('--dir', type=str, 
                       default='assets/ttls/LS_ENU_TTL_CSV/usable',
                       help='Directory containing TTL CSV files (default: assets/ttls/LS_ENU_TTL_CSV/usable)')
    parser.add_argument('--xodr', type=str, default=None,
                       help='Optional path to XODR track file for overlay (default: auto-detect Laguna Seca)')
    parser.add_argument('--dx', type=float, default=-2.0,
                       help='X offset to apply to TTL points (default: -2.0, best alignment found)')
    parser.add_argument('--dy', type=float, default=-53.0,
                       help='Y offset to apply to TTL points (default: -53.0, best alignment found)')
    parser.add_argument('--save', type=str, default=None,
                       help='Save comparison plot to file')
    parser.add_argument('--no-show', action='store_true',
                       help='Do not display plot interactively')
    parser.add_argument('--tolerance', type=float, default=1.0,
                       help='Tolerance for duplicate detection in meters (default: 1.0)')
    
    args = parser.parse_args()
    
    # Auto-detect Laguna Seca XODR if not specified
    if args.xodr is None:
        default_xodr = Path('assets/maps/dSPACE/LagunaSeca.xodr')
        if default_xodr.exists():
            args.xodr = str(default_xodr)
            print(f"[INFO] Auto-detected track file: {args.xodr}")
    
    # Find all CSV files
    ttl_dir = Path(args.dir)
    if not ttl_dir.exists():
        print(f"[ERROR] Directory not found: {ttl_dir}")
        sys.exit(1)
    
    csv_files = sorted(ttl_dir.glob('*.csv'))
    if not csv_files:
        print(f"[ERROR] No CSV files found in: {ttl_dir}")
        sys.exit(1)
    
    print(f"[INFO] Found {len(csv_files)} TTL CSV files in {ttl_dir}")
    print()
    
    # Load all TTLs
    ttl_data = {}
    for csv_file in csv_files:
        name = csv_file.stem
        print(f"[LOAD] Loading {name}...")
        points, metadata = read_ttl_xy(str(csv_file), dx=args.dx, dy=args.dy)
        ttl_data[name] = (points, metadata)
        
        if points:
            print(f"  Points: {len(points)}, Length: {compute_path_length(points):.1f}m")
            if metadata.get('length'):
                print(f"  Metadata length: {metadata['length']:.1f}m")
        print()
    
    # Validate each TTL
    print("=" * 80)
    print("VALIDATION RESULTS")
    print("=" * 80)
    
    validation_results = {}
    for name, (points, metadata) in ttl_data.items():
        print(f"\n[{name}]")
        stats = validate_ttl(points, metadata)
        validation_results[name] = stats
        
        if stats['valid']:
            print("  [OK] Valid")
        else:
            print("  [ISSUES] Issues found:")
            for issue in stats['issues']:
                print(f"    - {issue}")
        
        print(f"  Computed length: {stats['computed_length']:.2f}m")
        if stats['duplicate_points'] > 0:
            print(f"  Duplicate points: {stats['duplicate_points']}")
        if stats['short_segments'] > 0:
            print(f"  Very short segments: {stats['short_segments']}")
    
    # Compare TTLs for duplicates
    print("\n" + "=" * 80)
    print("DUPLICATE DETECTION")
    print("=" * 80)
    
    duplicates = find_duplicates(ttl_data)
    if duplicates:
        print("\n[WARNING] Found duplicate TTLs (identical waypoints):")
        for name, dup_list in duplicates.items():
            print(f"  {name} is identical to: {', '.join(dup_list)}")
    else:
        print("\n[OK] No duplicate TTLs found (all are unique)")
    
    # Pairwise comparison
    print("\n" + "=" * 80)
    print("PAIRWISE COMPARISON")
    print("=" * 80)
    
    ttl_names = sorted(ttl_data.keys())
    for i, name1 in enumerate(ttl_names):
        pts1, meta1 = ttl_data[name1]
        for name2 in ttl_names[i+1:]:
            pts2, meta2 = ttl_data[name2]
            comparison = compare_waypoints(pts1, pts2, tolerance=args.tolerance)
            
            if comparison['identical']:
                print(f"\n[{name1} vs {name2}]: IDENTICAL (within {args.tolerance}m)")
            elif comparison['same_length']:
                print(f"\n[{name1} vs {name2}]:")
                print(f"  Max distance: {comparison['max_distance']:.2f}m")
                print(f"  Mean distance: {comparison['mean_distance']:.2f}m")
                print(f"  Overlap: {comparison['overlap_percent']:.1f}%")
            else:
                print(f"\n[{name1} vs {name2}]:")
                print(f"  Different lengths: {len(pts1)} vs {len(pts2)} points")
    
    # Summary statistics
    print("\n" + "=" * 80)
    print("SUMMARY STATISTICS")
    print("=" * 80)
    
    print(f"\nTotal TTL files: {len(ttl_data)}")
    valid_count = sum(1 for stats in validation_results.values() if stats['valid'])
    print(f"Valid files: {valid_count}/{len(ttl_data)}")
    print(f"Duplicate files: {len(duplicates)}")
    
    print("\nFile details:")
    print(f"{'Name':<20} {'Points':<10} {'Length (m)':<12} {'Status':<10}")
    print("-" * 60)
    for name, (points, metadata) in sorted(ttl_data.items()):
        length = compute_path_length(points)
        status = "[OK]" if validation_results[name]['valid'] else "[ISSUES]"
        print(f"{name:<20} {len(points):<10} {length:<12.1f} {status:<10}")
    
    # Visualize
    if len(ttl_data) > 0:
        print("\n" + "=" * 80)
        print("GENERATING VISUALIZATION")
        print("=" * 80)
        visualize_all_ttls(ttl_data, xodr_path=args.xodr, 
                          save_path=args.save, show_plot=not args.no_show)


if __name__ == "__main__":
    main()

