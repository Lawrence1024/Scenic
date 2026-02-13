#!/usr/bin/env python3
"""
Generate a racing line from centerline using curvature-based optimization.

This script:
1. Loads the centerline from ttl_fellow_test_xodr_all.csv
2. Computes curvature along the path
3. Generates a racing line by offsetting from centerline based on curvature
4. Respects track width constraints (5-10m max deviation)
5. Outputs in the same CSV format (x,y,z)

Usage:
    python create_new_ttl/generate_racing_line.py
"""

import sys
import os
import csv
import math
import numpy as np
from pathlib import Path
from typing import List, Tuple, Optional
from scipy.interpolate import splprep, splev, UnivariateSpline
from scipy.optimize import minimize_scalar

# Add Scenic src to path
scenic_path = Path(__file__).parent.parent / "src"
if scenic_path.exists():
    sys.path.insert(0, str(scenic_path))

# Configuration
MAX_TRACK_WIDTH = 10.0  # Track width for reference (meters)
MAX_RACING_LINE_OFFSET = 2.0  # Maximum lateral offset from centerline (meters) - conservative to stay on track
MIN_TRACK_WIDTH = 0.5   # Minimum lateral offset (for smooth transitions)
CURVATURE_THRESHOLD = 0.008  # Curvature threshold for corner detection (1/m) - only offset on sharper turns
LOOKAHEAD_DISTANCE = 15.0  # Distance to look ahead (meters) - shorter = less "swing" before turn
SMOOTHING_WINDOW = 5  # Number of points for smoothing offsets
# On corner exit (curvature decreasing = past apex), taper offset back toward centerline so we don't
# stay outside and run off track. 0.0 = centerline on exit, 1.0 = same as approach (can run off).
EXIT_TAPER = 0.2  # Use only 20% of offset on exit - pull back to centerline after apex
# Turn-in: start pulling toward apex this many meters before the apex (sharp curves). Larger = turn in
# earlier so we don't run out of track on U-turns; smaller = later turn-in.
TURN_IN_DISTANCE = 25.0  # meters before apex to start turning in


def compute_curvature(p0: Tuple[float, float], 
                      p1: Tuple[float, float], 
                      p2: Tuple[float, float]) -> float:
    """Compute curvature using 3-point method.
    
    Returns:
        Curvature (1/radius) in 1/meters. Positive = left turn, Negative = right turn.
    """
    x0, y0 = p0
    x1, y1 = p1
    x2, y2 = p2
    
    # Vectors
    v1x = x1 - x0
    v1y = y1 - y0
    v2x = x2 - x1
    v2y = y2 - y1
    
    # Cross product (z-component)
    cross = v1x * v2y - v1y * v2x
    
    # Lengths
    len1 = math.sqrt(v1x*v1x + v1y*v1y)
    len2 = math.sqrt(v2x*v2x + v2y*v2y)
    
    if len1 < 1e-6 or len2 < 1e-6:
        return 0.0
    
    # Curvature = 2 * cross / (len1 * len2 * average_length)
    avg_len = (len1 + len2) / 2.0
    if avg_len < 1e-6:
        return 0.0
    
    curvature = 2.0 * cross / (len1 * len2 * avg_len)
    return curvature


def compute_curvature_profile(centerline: List[Tuple[float, float]]) -> List[float]:
    """Compute curvature at each point along the centerline.
    
    Args:
        centerline: List of (x, y) or (x, y, z) points
        
    Returns:
        List of curvature values (1/meters)
    """
    n = len(centerline)
    curvature = [0.0] * n
    
    if n < 3:
        return curvature
    
    # Compute curvature for interior points
    for i in range(1, n - 1):
        p0 = centerline[i - 1]
        p1 = centerline[i]
        p2 = centerline[i + 1]
        curvature[i] = compute_curvature(p0, p1, p2)
    
    # Use neighbor values for endpoints
    curvature[0] = curvature[1] if n > 1 else 0.0
    curvature[-1] = curvature[-2] if n > 1 else 0.0
    
    return curvature


def compute_normal_vector(p0: Tuple[float, float], 
                         p1: Tuple[float, float]) -> Tuple[float, float]:
    """Compute unit normal vector pointing to the left of the path.
    
    Args:
        p0: Previous point
        p1: Current point
        
    Returns:
        Unit normal vector (nx, ny) pointing left
    """
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    length = math.sqrt(dx*dx + dy*dy)
    
    if length < 1e-6:
        return (0.0, 1.0)  # Default: up (left)
    
    # Normalize tangent
    tx = dx / length
    ty = dy / length
    
    # Rotate 90 degrees counterclockwise (left normal)
    nx = -ty
    ny = tx
    
    return (nx, ny)


def compute_optimal_offset(curvature: float, 
                          lookahead_curvature: float,
                          track_width: float = MAX_TRACK_WIDTH) -> float:
    """Compute optimal lateral offset for racing line.
    
    Conservative strategy to stay on track:
    - Small offset only on sharper turns (no huge swing before corner)
    - Left turns: slight offset right (outside); right turns: slight offset left
    - Straights and mild curves: stay on centerline
    
    Args:
        curvature: Current curvature (1/m)
        lookahead_curvature: Curvature ahead (for anticipation)
        track_width: Track width for reference (meters)
        
    Returns:
        Lateral offset in meters (positive = left, negative = right)
    """
    # Use lookahead for mild anticipation only (reduced weight = less early swing)
    effective_curvature = 0.85 * curvature + 0.15 * lookahead_curvature
    
    abs_curvature = abs(effective_curvature)
    
    # For straights and mild curves, stay on centerline
    if abs_curvature < CURVATURE_THRESHOLD:
        return 0.0
    
    # Cap offset so we don't swing far outside (stay on track)
    max_offset = MAX_RACING_LINE_OFFSET
    
    # Scale offset by curvature - only use full offset on very sharp turns
    curvature_factor = min(1.0, abs_curvature / 0.03)  # 0.03 1/m ~ R=33m for full offset
    
    # Offset direction: opposite of turn direction (outside before apex)
    # Left turn (positive curvature) -> offset right (negative)
    # Right turn (negative curvature) -> offset left (positive)
    offset = -np.sign(effective_curvature) * max_offset * curvature_factor
    
    return offset


def smooth_offsets(offsets: List[float], window: int = SMOOTHING_WINDOW) -> List[float]:
    """Smooth offset values using moving average.
    
    Args:
        offsets: List of offset values
        window: Smoothing window size
        
    Returns:
        Smoothed offset values
    """
    n = len(offsets)
    smoothed = [0.0] * n
    
    half_window = window // 2
    
    for i in range(n):
        start = max(0, i - half_window)
        end = min(n, i + half_window + 1)
        smoothed[i] = np.mean(offsets[start:end])
    
    return smoothed


def generate_racing_line(centerline: List[Tuple[float, float]], 
                        track_width: float = MAX_TRACK_WIDTH) -> List[Tuple[float, float, float]]:
    """Generate racing line from centerline.
    
    Args:
        centerline: List of (x, y) or (x, y, z) centerline points
        track_width: Maximum track width (meters)
        
    Returns:
        List of (x, y, z) racing line points
    """
    n = len(centerline)
    if n < 3:
        return [(p[0], p[1], p[2] if len(p) >= 3 else 0.0) for p in centerline]
    
    # Extract 2D points for curvature computation
    points_2d = [(p[0], p[1]) for p in centerline]
    
    # Compute curvature profile
    curvature = compute_curvature_profile(points_2d)
    
    # Compute cumulative distance for lookahead
    cumulative_dist = [0.0]
    for i in range(1, n):
        dx = points_2d[i][0] - points_2d[i-1][0]
        dy = points_2d[i][1] - points_2d[i-1][1]
        dist = math.sqrt(dx*dx + dy*dy)
        cumulative_dist.append(cumulative_dist[-1] + dist)
    
    # Curvature derivative: positive = entering corner, negative = exiting (past apex)
    curvature_derivative = [0.0] * n
    for i in range(1, n - 1):
        curvature_derivative[i] = curvature[i + 1] - curvature[i - 1]  # simple discrete derivative
    curvature_derivative[0] = curvature_derivative[1]
    curvature_derivative[-1] = curvature_derivative[-2]
    
    # Find apex indices (local maxima of |curvature|) for turn-in
    apex_indices = []
    for i in range(1, n - 1):
        if abs(curvature[i]) >= abs(curvature[i - 1]) and abs(curvature[i]) >= abs(curvature[i + 1]):
            if abs(curvature[i]) >= CURVATURE_THRESHOLD:
                apex_indices.append(i)
    
    # For each point, distance to next apex along path (for turn-in scaling)
    def distance_to_next_apex(idx: int) -> float:
        """Distance from point idx to the next apex ahead. Returns inf if no apex ahead."""
        for apex in apex_indices:
            if apex >= idx:
                return cumulative_dist[apex] - cumulative_dist[idx]
        return float("inf")
    
    # Compute optimal offsets
    offsets = []
    for i in range(n):
        # Find lookahead point
        lookahead_idx = i
        for j in range(i, n):
            if cumulative_dist[j] - cumulative_dist[i] >= LOOKAHEAD_DISTANCE:
                lookahead_idx = j
                break
        
        lookahead_curvature = curvature[lookahead_idx] if lookahead_idx < n else curvature[i]
        offset = compute_optimal_offset(curvature[i], lookahead_curvature, track_width)
        # Turn-in: within TURN_IN_DISTANCE of apex, pull toward centerline so we don't stay
        # outside too long and run out of track on sharp curves / U-turns
        dist_apex = distance_to_next_apex(i)
        if dist_apex <= TURN_IN_DISTANCE and dist_apex >= 0 and abs(curvature[i]) >= CURVATURE_THRESHOLD:
            turn_in_scale = dist_apex / TURN_IN_DISTANCE  # 0 at apex, 1 at TURN_IN_DISTANCE before
            offset *= turn_in_scale
        # On corner exit (curvature decreasing), taper offset so we return toward centerline
        elif curvature_derivative[i] < -1e-6 and abs(curvature[i]) >= CURVATURE_THRESHOLD:
            offset *= EXIT_TAPER
        offsets.append(offset)
    
    # Smooth offsets
    offsets = smooth_offsets(offsets)
    
    # Apply offsets to generate racing line
    racing_line = []
    for i in range(n):
        # Compute normal vector
        if i == 0:
            # Use next point for normal
            normal = compute_normal_vector(points_2d[0], points_2d[1])
        else:
            normal = compute_normal_vector(points_2d[i-1], points_2d[i])
        
        # Apply offset
        offset_x = offsets[i] * normal[0]
        offset_y = offsets[i] * normal[1]
        
        # Get original point
        orig_point = centerline[i]
        x = orig_point[0] + offset_x
        y = orig_point[1] + offset_y
        z = orig_point[2] if len(orig_point) >= 3 else 0.0
        
        racing_line.append((x, y, z))
    
    return racing_line


def load_centerline(csv_path: str) -> List[Tuple[float, float, float]]:
    """Load centerline from CSV file.
    
    Args:
        csv_path: Path to CSV file with x,y,z columns
        
    Returns:
        List of (x, y, z) points
    """
    centerline = []
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            x = float(row['x'])
            y = float(row['y'])
            z = float(row.get('z', '0.0'))
            centerline.append((x, y, z))
    return centerline


def save_racing_line(racing_line: List[Tuple[float, float, float]], 
                    output_path: str):
    """Save racing line to CSV file.
    
    Args:
        racing_line: List of (x, y, z) points
        output_path: Output CSV file path
    """
    with open(output_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['x', 'y', 'z'])
        for point in racing_line:
            writer.writerow([f"{point[0]:.6f}", f"{point[1]:.6f}", f"{point[2]:.6f}"])
    
    print(f"Saved {len(racing_line)} racing line points to {output_path}")


def main():
    """Main function to generate racing line."""
    # Paths
    script_dir = Path(__file__).parent
    project_root = script_dir.parent
    
    centerline_path = project_root / "assets" / "ttls" / "LS_ENU_TTL_CSV" / "transformed" / "ttl_fellow_test_xodr_all.csv"
    output_path = script_dir / "ttl_racing_line_xodr.csv"
    
    print("=" * 70)
    print("Racing Line Generator")
    print("=" * 70)
    print(f"\nLoading centerline from: {centerline_path}")
    
    if not centerline_path.exists():
        print(f"ERROR: Centerline file not found: {centerline_path}")
        return
    
    # Load centerline
    centerline = load_centerline(str(centerline_path))
    print(f"Loaded {len(centerline)} centerline points")
    
    # Generate racing line
    print(f"\nGenerating racing line (conservative: small offset, taper on exit)...")
    print(f"  Max racing line offset: {MAX_RACING_LINE_OFFSET}m")
    print(f"  Curvature threshold: {CURVATURE_THRESHOLD} 1/m")
    print(f"  Lookahead distance: {LOOKAHEAD_DISTANCE}m")
    print(f"  Exit taper: {EXIT_TAPER} (pull back to centerline after apex)")
    print(f"  Turn-in distance: {TURN_IN_DISTANCE}m (start turning in before apex on sharp curves)")
    
    racing_line = generate_racing_line(centerline, MAX_TRACK_WIDTH)
    
    # Save racing line
    print(f"\nSaving racing line to: {output_path}")
    save_racing_line(racing_line, str(output_path))
    
    # Statistics
    print("\n" + "=" * 70)
    print("Statistics")
    print("=" * 70)
    
    # Compute deviations
    deviations = []
    for i in range(len(centerline)):
        dx = racing_line[i][0] - centerline[i][0]
        dy = racing_line[i][1] - centerline[i][1]
        deviation = math.sqrt(dx*dx + dy*dy)
        deviations.append(deviation)
    
    deviations = np.array(deviations)
    print(f"Mean deviation: {np.mean(deviations):.2f} m")
    print(f"Median deviation: {np.median(deviations):.2f} m")
    print(f"Max deviation: {np.max(deviations):.2f} m")
    print(f"95th percentile: {np.percentile(deviations, 95):.2f} m")
    print(f"\nPoints with deviation > 10m: {np.sum(deviations > 10.0)} ({100*np.sum(deviations > 10.0)/len(deviations):.1f}%)")
    print(f"Points with deviation > 5m: {np.sum(deviations > 5.0)} ({100*np.sum(deviations > 5.0)/len(deviations):.1f}%)")
    
    print("\n" + "=" * 70)
    print("Done!")
    print("=" * 70)


if __name__ == "__main__":
    main()
