#!/usr/bin/env python3
"""
Compare and validate multiple TTL CSV files.

This tool:
1. Loads all TTL CSV files from a directory
2. Compares waypoints to detect duplicates
3. Validates file structure and metadata
4. Checks if all TTL points are within track boundaries (requires --xodr)
5. Visualizes all TTLs together for comparison
6. Reports statistics and differences

Usage:
    python compare_ttls.py [--dir <directory>] [--xodr <xodr_path>] [--dx <dx>] [--dy <dy>]
    
Examples:
    # Compare all TTLs in the default directory
    python compare_ttls.py
    
    # Compare TTLs in a specific directory
    python compare_ttls.py --dir assets/ttls/LS_ENU_TTL_CSV
    
    # Compare with track overlay and boundary validation
    python compare_ttls.py --xodr assets/maps/dSPACE/LagunaSeca.xodr
    
    # Compute optimal transformation from TTL to map coordinates
    python compare_ttls.py --compute-transform --xodr assets/maps/dSPACE/LagunaSeca.xodr
    
    # Compute and save transformation to JSON file
    python compare_ttls.py --compute-transform --xodr assets/maps/dSPACE/LagunaSeca.xodr \
        --save-transform assets/maps/dSPACE/Laguna_Seca_transform.json
"""

import os
import sys
import csv
import argparse
import json
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
from pathlib import Path
from typing import List, Tuple, Dict, Set, Optional
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


def save_transformation(transform: Dict, output_path: str, xodr_path: str = None, ttl_name: str = None):
    """Save transformation parameters to a JSON file.
    
    Args:
        transform: Transformation dictionary from compute_optimal_transformation
        output_path: Path to output JSON file
        xodr_path: Optional path to XODR file used for transformation
        ttl_name: Optional name of TTL file used for transformation
    """
    output_file = Path(output_path)
    output_file.parent.mkdir(parents=True, exist_ok=True)
    
    # Prepare data to save
    data = {
        'type': transform.get('type', 'translation'),
        'dx': transform.get('dx', 0.0),
        'dy': transform.get('dy', 0.0),
    }
    
    # Add optional fields
    if 'std_dx' in transform:
        data['std_dx'] = transform['std_dx']
    if 'std_dy' in transform:
        data['std_dy'] = transform['std_dy']
    if 'error' in transform:
        data['mean_alignment_error'] = transform['error']
    if 'in_bounds_percent' in transform:
        data['in_bounds_percent'] = transform['in_bounds_percent']
    if 'method_used' in transform:
        data['optimization_method'] = transform['method_used']
    
    # Add metadata
    if xodr_path:
        data['xodr_file'] = str(xodr_path)
    if ttl_name:
        data['ttl_file'] = ttl_name
    
    data['timestamp'] = __import__('datetime').datetime.now().isoformat()
    
    # Write JSON file
    with open(output_file, 'w') as f:
        json.dump(data, f, indent=2)
    
    print(f"[SAVED] Transformation saved to: {output_file}")
    print(f"        dx = {data['dx']:.6f}, dy = {data['dy']:.6f}")


def compute_optimal_transformation(ttl_points: List[Tuple[float, float]],
                                   map_centerline: List[Tuple[float, float]],
                                   method: str = 'translation',
                                   left_boundary_segments: List[List[Tuple[float, float]]] = None,
                                   right_boundary_segments: List[List[Tuple[float, float]]] = None,
                                   pit_left_boundary_segments: List[List[Tuple[float, float]]] = None,
                                   pit_right_boundary_segments: List[List[Tuple[float, float]]] = None) -> Dict:
    """Compute optimal transformation from TTL coordinates to map coordinates.
    
    This function finds the best transformation to align TTL points with the map centerline,
    optimizing for maximum points within track boundaries.
    
    Args:
        ttl_points: List of (x, y) points from TTL (in ENU coordinates)
        map_centerline: List of (x, y) points from map centerline (in XODR coordinates)
        method: 'translation' (simple offset) or 'affine' (rotation + translation)
        left_boundary_segments: Optional left boundary segments for optimization
        right_boundary_segments: Optional right boundary segments for optimization
    
    Returns:
        Dictionary with transformation parameters:
        - 'type': 'translation' or 'affine'
        - 'dx', 'dy': Translation offsets (if type='translation')
        - 'matrix': Affine transformation matrix (if type='affine')
        - 'offset': Translation offset
        - 'error': Mean alignment error after transformation
        - 'in_bounds_percent': Percentage of points in bounds after transformation
    """
    if not ttl_points or not map_centerline:
        return {'type': 'translation', 'dx': 0.0, 'dy': 0.0, 'error': float('inf'), 'in_bounds_percent': 0.0}
    
    ttl_array = np.array(ttl_points)
    map_array = np.array(map_centerline)
    
    if method == 'translation':
        # Strategy 1: If boundaries are available, optimize for points in bounds
        if left_boundary_segments and right_boundary_segments:
            return _optimize_translation_for_bounds(
                ttl_points, map_centerline, left_boundary_segments, right_boundary_segments,
                pit_left_boundary_segments=pit_left_boundary_segments,
                pit_right_boundary_segments=pit_right_boundary_segments
            )
        
        # Strategy 2: Use improved centerline alignment
        # First, get initial estimate using median (more robust than mean)
        offsets = []
        for ttl_pt in ttl_array:
            distances = np.linalg.norm(map_array - ttl_pt, axis=1)
            nearest_idx = np.argmin(distances)
            nearest_map_pt = map_array[nearest_idx]
            offset = nearest_map_pt - ttl_pt
            offsets.append(offset)
        
        offsets = np.array(offsets)
        
        # Use median instead of mean for robustness against outliers
        median_offset = np.median(offsets, axis=0)
        mean_offset = np.mean(offsets, axis=0)
        std_offset = np.std(offsets, axis=0)
        
        # Try both median and mean, pick the one with better alignment
        candidates = [
            ('median', median_offset),
            ('mean', mean_offset)
        ]
        
        best_transform = None
        best_score = float('inf')
        
        for name, offset in candidates:
            transformed_ttl = ttl_array + offset
            errors = []
            for pt in transformed_ttl:
                distances = np.linalg.norm(map_array - pt, axis=1)
                errors.append(np.min(distances))
            mean_error = np.mean(errors)
            
            if mean_error < best_score:
                best_score = mean_error
                best_transform = {
                    'type': 'translation',
                    'dx': float(offset[0]),
                    'dy': float(offset[1]),
                    'std_dx': float(std_offset[0]),
                    'std_dy': float(std_offset[1]),
                    'error': float(mean_error),
                    'method_used': name
                }
        
        return best_transform
    
    elif method == 'affine':
        # Affine transformation: rotation + translation
        # Use least-squares to find best transformation
        # This is more complex and requires matching corresponding points
        
        # For now, fall back to translation
        # Full affine implementation would require:
        # 1. Point correspondence (matching TTL points to map points)
        # 2. Solving for rotation matrix and translation
        # 3. This is similar to ICP (Iterative Closest Point) algorithm
        
        return compute_optimal_transformation(ttl_points, map_centerline, method='translation',
                                            left_boundary_segments=left_boundary_segments,
                                            right_boundary_segments=right_boundary_segments,
                                            pit_left_boundary_segments=pit_left_boundary_segments,
                                            pit_right_boundary_segments=pit_right_boundary_segments)
    
    else:
        raise ValueError(f"Unknown method: {method}")


def _optimize_translation_for_bounds(ttl_points: List[Tuple[float, float]],
                                     map_centerline: List[Tuple[float, float]],
                                     left_boundary_segments: List[List[Tuple[float, float]]],
                                     right_boundary_segments: List[List[Tuple[float, float]]],
                                     pit_left_boundary_segments: List[List[Tuple[float, float]]] = None,
                                     pit_right_boundary_segments: List[List[Tuple[float, float]]] = None) -> Dict:
    """Optimize translation (dx, dy) to maximize points in bounds.
    
    Uses scipy.optimize if available, otherwise uses iterative refinement.
    """
    ttl_array = np.array(ttl_points)
    map_array = np.array(map_centerline)
    
    # Get initial estimate using centerline alignment (median is robust)
    offsets = []
    for ttl_pt in ttl_array:
        distances = np.linalg.norm(map_array - ttl_pt, axis=1)
        nearest_idx = np.argmin(distances)
        nearest_map_pt = map_array[nearest_idx]
        offset = nearest_map_pt - ttl_pt
        offsets.append(offset)
    
    offsets = np.array(offsets)
    initial_dx = float(np.median(offsets[:, 0]))
    initial_dy = float(np.median(offsets[:, 1]))
    std_dx = float(np.std(offsets[:, 0]))
    std_dy = float(np.std(offsets[:, 1]))
    
    # Try scipy.optimize first (fastest and most accurate)
    try:
        from scipy.optimize import minimize, differential_evolution
        
        # Objective function: minimize negative in-bounds percentage
        def objective(params):
            dx, dy = params
            transformed_points = [(p[0] + dx, p[1] + dy) for p in ttl_points]
            result = check_ttl_in_bounds(
                transformed_points,
                map_centerline,
                left_boundary_segments,
                right_boundary_segments,
                tolerance=0.5,
                pit_left_boundary_segments=pit_left_boundary_segments,
                pit_right_boundary_segments=pit_right_boundary_segments
            )
            # Return negative percentage (we want to maximize it)
            in_bounds_pct = result.get('in_bounds_percent', 0.0)
            return -in_bounds_pct
        
        # Search bounds: ±3 standard deviations or at least ±10m
        bounds = [
            (initial_dx - max(10.0, 3 * std_dx), initial_dx + max(10.0, 3 * std_dx)),
            (initial_dy - max(10.0, 3 * std_dy), initial_dy + max(10.0, 3 * std_dy))
        ]
        
        print(f"[INFO] Optimizing with scipy.optimize: initial dx={initial_dx:.3f}, dy={initial_dy:.3f}")
        
        # Try multiple starting points for better global optimization
        # 1. Median estimate
        # 2. Mean estimate
        # 3. A few random samples within bounds
        mean_dx = float(np.mean(offsets[:, 0]))
        mean_dy = float(np.mean(offsets[:, 1]))
        
        starting_points = [
            [initial_dx, initial_dy],  # Median
            [mean_dx, mean_dy],        # Mean
        ]
        
        # Add a few random starting points
        np.random.seed(42)  # For reproducibility
        for _ in range(3):
            random_dx = initial_dx + np.random.uniform(-std_dx, std_dx)
            random_dy = initial_dy + np.random.uniform(-std_dy, std_dy)
            starting_points.append([random_dx, random_dy])
        
        best_result = None
        best_score = float('inf')
        
        # Try each starting point with local optimization
        for i, x0 in enumerate(starting_points):
            try:
                result = minimize(
                    objective,
                    x0=x0,
                    method='Nelder-Mead',
                    bounds=bounds,
                    options={'maxiter': 150, 'xatol': 0.05, 'fatol': 0.05}
                )
                if result.fun < best_score:
                    best_score = result.fun
                    best_result = result
            except Exception:
                continue
        
        # If we have a good result, refine it further
        if best_result is not None:
            # Final refinement with tighter tolerance
            refined_result = minimize(
                objective,
                x0=best_result.x,
                method='Nelder-Mead',
                bounds=bounds,
                options={'maxiter': 50, 'xatol': 0.01, 'fatol': 0.01}
            )
            if refined_result.fun < best_score:
                best_result = refined_result
        
        if best_result is None:
            # Fallback: use initial estimate
            best_dx, best_dy = initial_dx, initial_dy
            transformed_points = [(p[0] + best_dx, p[1] + best_dy) for p in ttl_points]
            result = check_ttl_in_bounds(
                transformed_points, map_centerline,
                left_boundary_segments, right_boundary_segments, tolerance=0.5
            )
            best_in_bounds = result.get('in_bounds_percent', 0.0)
        else:
            best_dx = float(best_result.x[0])
            best_dy = float(best_result.x[1])
            best_in_bounds = -float(best_result.fun)  # Convert back from negative
        
        # Final fine-tuning: try small perturbations around best solution
        print(f"[INFO] Fine-tuning around best solution: dx={best_dx:.3f}, dy={best_dy:.3f}, {best_in_bounds:.1f}% in bounds")
        fine_tuned_dx, fine_tuned_dy = best_dx, best_dy
        fine_tuned_in_bounds = best_in_bounds
        
        # Try small perturbations in a 0.2m grid around the best solution
        for delta_dx in np.arange(-0.2, 0.25, 0.05):
            for delta_dy in np.arange(-0.2, 0.25, 0.05):
                test_dx = best_dx + delta_dx
                test_dy = best_dy + delta_dy
                transformed_points = [(p[0] + test_dx, p[1] + test_dy) for p in ttl_points]
                result = check_ttl_in_bounds(
                    transformed_points,
                    map_centerline,
                    left_boundary_segments,
                    right_boundary_segments,
                    tolerance=0.5
                )
                in_bounds_pct = result.get('in_bounds_percent', 0.0)
                if in_bounds_pct > fine_tuned_in_bounds:
                    fine_tuned_in_bounds = in_bounds_pct
                    fine_tuned_dx, fine_tuned_dy = test_dx, test_dy
        
        if fine_tuned_in_bounds > best_in_bounds:
            best_dx, best_dy = fine_tuned_dx, fine_tuned_dy
            best_in_bounds = fine_tuned_in_bounds
            print(f"[INFO] Fine-tuning improved result: dx={best_dx:.3f}, dy={best_dy:.3f}, {best_in_bounds:.1f}% in bounds")
        
        print(f"[INFO] Final optimization result: dx={best_dx:.3f}, dy={best_dy:.3f}, {best_in_bounds:.1f}% in bounds")
        method_used = 'scipy_optimize_multi_start'
        
    except ImportError:
        # Fall back to iterative refinement (no scipy required)
        print(f"[INFO] scipy not available, using iterative refinement: initial dx={initial_dx:.3f}, dy={initial_dy:.3f}")
        best_dx, best_dy, best_in_bounds = _iterative_refinement(
            ttl_points, map_centerline, left_boundary_segments, right_boundary_segments,
            initial_dx, initial_dy, std_dx, std_dy,
            pit_left_boundary_segments=pit_left_boundary_segments,
            pit_right_boundary_segments=pit_right_boundary_segments
        )
        method_used = 'iterative_refinement'
    
    # Compute final error metric
    transformed_ttl = ttl_array + np.array([best_dx, best_dy])
    errors = []
    for pt in transformed_ttl:
        distances = np.linalg.norm(map_array - pt, axis=1)
        errors.append(np.min(distances))
    mean_error = np.mean(errors)
    
    return {
        'type': 'translation',
        'dx': best_dx,
        'dy': best_dy,
        'std_dx': std_dx,
        'std_dy': std_dy,
        'error': mean_error,
        'in_bounds_percent': best_in_bounds,
        'method_used': method_used
    }


def _iterative_refinement(ttl_points: List[Tuple[float, float]],
                          map_centerline: List[Tuple[float, float]],
                          left_boundary_segments: List[List[Tuple[float, float]]],
                          right_boundary_segments: List[List[Tuple[float, float]]],
                          initial_dx: float, initial_dy: float,
                          std_dx: float, std_dy: float,
                          pit_left_boundary_segments: List[List[Tuple[float, float]]] = None,
                          pit_right_boundary_segments: List[List[Tuple[float, float]]] = None) -> Tuple[float, float, float]:
    """Iterative refinement to find best dx, dy (no scipy required).
    
    Uses coordinate descent with multiple starting points and adaptive step size.
    """
    # Try multiple starting points
    mean_dx = initial_dx  # Will be computed from offsets if available
    mean_dy = initial_dy
    
    starting_points = [
        (initial_dx, initial_dy),  # Median
        (mean_dx, mean_dy),        # Mean
    ]
    
    # Add a few variations
    starting_points.extend([
        (initial_dx + 0.5 * std_dx, initial_dy),
        (initial_dx - 0.5 * std_dx, initial_dy),
        (initial_dx, initial_dy + 0.5 * std_dy),
        (initial_dx, initial_dy - 0.5 * std_dy),
    ])
    
    best_dx, best_dy = initial_dx, initial_dy
    best_in_bounds = 0.0
    
    # Try each starting point
    for start_dx, start_dy in starting_points:
        dx, dy = start_dx, start_dy
        
        # Evaluate starting point
        transformed_points = [(p[0] + dx, p[1] + dy) for p in ttl_points]
        result = check_ttl_in_bounds(
            transformed_points,
            map_centerline,
            left_boundary_segments,
            right_boundary_segments,
            tolerance=0.5,
            pit_left_boundary_segments=pit_left_boundary_segments,
            pit_right_boundary_segments=pit_right_boundary_segments
        )
        current_in_bounds = result.get('in_bounds_percent', 0.0)
        
        # Iterative refinement: optimize dx and dy alternately
        step_size = 0.5  # Start with 0.5m steps
        max_iterations = 30
        improvement_threshold = 0.05  # Stop if improvement < 0.05%
        
        for iteration in range(max_iterations):
            improved = False
            
            # Optimize dx (fix dy) - try 5 points around current dx
            for test_dx in [dx - step_size, dx - 0.5*step_size, dx, dx + 0.5*step_size, dx + step_size]:
                transformed_points = [(p[0] + test_dx, p[1] + dy) for p in ttl_points]
                result = check_ttl_in_bounds(
                    transformed_points,
                    map_centerline,
                    left_boundary_segments,
                    right_boundary_segments,
                    tolerance=0.5
                )
                in_bounds_pct = result.get('in_bounds_percent', 0.0)
                if in_bounds_pct > current_in_bounds + improvement_threshold:
                    current_in_bounds = in_bounds_pct
                    dx = test_dx
                    improved = True
            
            # Optimize dy (fix dx) - try 5 points around current dy
            for test_dy in [dy - step_size, dy - 0.5*step_size, dy, dy + 0.5*step_size, dy + step_size]:
                transformed_points = [(p[0] + dx, p[1] + test_dy) for p in ttl_points]
                result = check_ttl_in_bounds(
                    transformed_points,
                    map_centerline,
                    left_boundary_segments,
                    right_boundary_segments,
                    tolerance=0.5
                )
                in_bounds_pct = result.get('in_bounds_percent', 0.0)
                if in_bounds_pct > current_in_bounds + improvement_threshold:
                    current_in_bounds = in_bounds_pct
                    dy = test_dy
                    improved = True
            
            if not improved:
                # Reduce step size for finer search
                step_size *= 0.6
                if step_size < 0.02:  # Stop when step size is very small
                    break
        
        # Keep best result across all starting points
        if current_in_bounds > best_in_bounds:
            best_in_bounds = current_in_bounds
            best_dx, best_dy = dx, dy
    
    # Final fine-tuning: try small perturbations around best solution
    fine_tuned_dx, fine_tuned_dy = best_dx, best_dy
    fine_tuned_in_bounds = best_in_bounds
    
    for delta_dx in np.arange(-0.2, 0.25, 0.05):
        for delta_dy in np.arange(-0.2, 0.25, 0.05):
            test_dx = best_dx + delta_dx
            test_dy = best_dy + delta_dy
            transformed_points = [(p[0] + test_dx, p[1] + test_dy) for p in ttl_points]
            result = check_ttl_in_bounds(
                transformed_points,
                map_centerline,
                left_boundary_segments,
                right_boundary_segments,
                tolerance=0.5,
                pit_left_boundary_segments=pit_left_boundary_segments,
                pit_right_boundary_segments=pit_right_boundary_segments
            )
            in_bounds_pct = result.get('in_bounds_percent', 0.0)
            if in_bounds_pct > fine_tuned_in_bounds:
                fine_tuned_in_bounds = in_bounds_pct
                fine_tuned_dx, fine_tuned_dy = test_dx, test_dy
    
    if fine_tuned_in_bounds > best_in_bounds:
        best_dx, best_dy = fine_tuned_dx, fine_tuned_dy
        best_in_bounds = fine_tuned_in_bounds
    
    print(f"[INFO] Iterative refinement complete: dx={best_dx:.3f}, dy={best_dy:.3f}, {best_in_bounds:.1f}% in bounds")
    return best_dx, best_dy, best_in_bounds


def _simple_translation_estimate(ttl_points: List[Tuple[float, float]],
                                 map_centerline: List[Tuple[float, float]],
                                 left_boundary_segments: List[List[Tuple[float, float]]],
                                 right_boundary_segments: List[List[Tuple[float, float]]]) -> Dict:
    """Simple translation estimate using median offset (fallback when scipy not available)."""
    ttl_array = np.array(ttl_points)
    map_array = np.array(map_centerline)
    
    # Get median offset
    offsets = []
    for ttl_pt in ttl_array:
        distances = np.linalg.norm(map_array - ttl_pt, axis=1)
        nearest_idx = np.argmin(distances)
        nearest_map_pt = map_array[nearest_idx]
        offset = nearest_map_pt - ttl_pt
        offsets.append(offset)
    
    offsets = np.array(offsets)
    dx = float(np.median(offsets[:, 0]))
    dy = float(np.median(offsets[:, 1]))
    std_dx = float(np.std(offsets[:, 0]))
    std_dy = float(np.std(offsets[:, 1]))
    
    # Check in-bounds percentage
    transformed_points = [(p[0] + dx, p[1] + dy) for p in ttl_points]
    result = check_ttl_in_bounds(
        transformed_points,
        map_centerline,
        left_boundary_segments,
        right_boundary_segments,
        tolerance=0.5
    )
    in_bounds_pct = result.get('in_bounds_percent', 0.0)
    
    # Compute error
    transformed_ttl = ttl_array + np.array([dx, dy])
    errors = []
    for pt in transformed_ttl:
        distances = np.linalg.norm(map_array - pt, axis=1)
        errors.append(np.min(distances))
    mean_error = np.mean(errors)
    
    return {
        'type': 'translation',
        'dx': dx,
        'dy': dy,
        'std_dx': std_dx,
        'std_dy': std_dy,
        'error': mean_error,
        'in_bounds_percent': in_bounds_pct,
        'method_used': 'median_estimate'
    }


def extract_map_centerline(xodr_path: str) -> Optional[List[Tuple[float, float]]]:
    """Extract centerline points from XODR file.
    
    Returns:
        List of (x, y) centerline points, or None if extraction fails
    """
    if not xodr_path or not os.path.exists(xodr_path):
        return None
    
    try:
        from scenic.formats.opendrive import xodr_parser
        from scenic.domains.driving.roads import Network
        
        network = Network.fromOpenDrive(xodr_path, ref_points=50)
        centerline_points = []
        
        def extract_point_coords(pt):
            """Extract (x, y) from point, handling both tuple and object formats."""
            if isinstance(pt, tuple) or isinstance(pt, list):
                return (float(pt[0]), float(pt[1]))
            elif hasattr(pt, 'x') and hasattr(pt, 'y'):
                return (float(pt.x), float(pt.y))
            else:
                return None
        
        for road in network.roads:
            for lane in road.lanes:
                if hasattr(lane, 'centerline') and lane.centerline:
                    try:
                        if hasattr(lane.centerline, 'points'):
                            points_list = lane.centerline.points
                        elif hasattr(lane.centerline, '__iter__'):
                            points_list = list(lane.centerline)
                        else:
                            points_list = []
                        
                        step = max(1, len(points_list) // 200)  # Sample more points for better matching
                        for pt in points_list[::step]:
                            coords = extract_point_coords(pt)
                            if coords:
                                centerline_points.append(coords)
                    except Exception:
                        pass
        
        return centerline_points if centerline_points else None
    
    except Exception as e:
        print(f"[WARN] Could not extract map centerline: {e}")
        return None


def check_ttl_in_bounds(ttl_points: List[Tuple[float, float]],
                        centerline_points: List[Tuple[float, float]],
                        left_boundary_segments: List[List[Tuple[float, float]]],
                        right_boundary_segments: List[List[Tuple[float, float]]],
                        tolerance: float = 0.5,
                        pit_left_boundary_segments: List[List[Tuple[float, float]]] = None,
                        pit_right_boundary_segments: List[List[Tuple[float, float]]] = None) -> Dict:
    """Check if all TTL points are within track boundaries (including pit lanes).
    
    For each TTL point, checks if it's within the main track boundaries OR pit lane boundaries.
    Pit lanes are treated as normal lanes.
    
    Args:
        ttl_points: List of (x, y) TTL points to check
        centerline_points: List of (x, y) centerline points from track
        left_boundary_segments: List of main track left boundary segments
        right_boundary_segments: List of main track right boundary segments
        tolerance: Tolerance in meters for boundary checking (default: 0.5m)
        pit_left_boundary_segments: Optional list of pit lane left boundary segments
        pit_right_boundary_segments: Optional list of pit lane right boundary segments
    
    Returns:
        Dictionary with validation results:
        - 'all_in_bounds': True if all points are within boundaries
        - 'points_out_of_bounds': List of (index, point, distance) for points outside
        - 'num_out_of_bounds': Number of points outside boundaries
        - 'max_violation_distance': Maximum distance a point is outside boundaries
        - 'mean_violation_distance': Mean distance for points outside boundaries
    """
    if not ttl_points:
        return {
            'all_in_bounds': True,
            'points_out_of_bounds': [],
            'num_out_of_bounds': 0,
            'max_violation_distance': 0.0,
            'mean_violation_distance': 0.0
        }
    
    if not centerline_points or (not left_boundary_segments and not right_boundary_segments):
        return {
            'all_in_bounds': None,  # Cannot determine without boundaries
            'points_out_of_bounds': [],
            'num_out_of_bounds': 0,
            'max_violation_distance': 0.0,
            'mean_violation_distance': 0.0,
            'error': 'No track boundaries available'
        }
    
    # Flatten boundary segments into continuous lists for main track
    left_boundary = []
    for segment in left_boundary_segments:
        left_boundary.extend(segment)
    
    right_boundary = []
    for segment in right_boundary_segments:
        right_boundary.extend(segment)
    
    # Flatten pit lane boundaries if provided
    pit_left_boundary = []
    pit_right_boundary = []
    if pit_left_boundary_segments:
        for segment in pit_left_boundary_segments:
            pit_left_boundary.extend(segment)
    if pit_right_boundary_segments:
        for segment in pit_right_boundary_segments:
            pit_right_boundary.extend(segment)
    
    # Combine main track and pit lane boundaries (pit lanes treated as normal lanes)
    all_left_boundary = left_boundary + pit_left_boundary
    all_right_boundary = right_boundary + pit_right_boundary
    
    if not all_left_boundary or not all_right_boundary:
        return {
            'all_in_bounds': None,
            'points_out_of_bounds': [],
            'num_out_of_bounds': 0,
            'max_violation_distance': 0.0,
            'mean_violation_distance': 0.0,
            'error': 'Incomplete boundary data'
        }
    
    # Convert to numpy arrays for efficient computation
    centerline_array = np.array(centerline_points)
    left_array = np.array(all_left_boundary)
    right_array = np.array(all_right_boundary)
    ttl_array = np.array(ttl_points)
    
    points_out_of_bounds = []
    violation_distances = []
    
    def point_to_line_distance(point, line_start, line_end):
        """Compute perpendicular distance from point to line segment."""
        point = np.array(point)
        line_start = np.array(line_start)
        line_end = np.array(line_end)
        
        # Vector from line_start to line_end
        line_vec = line_end - line_start
        line_len_sq = np.dot(line_vec, line_vec)
        
        if line_len_sq < 1e-10:  # Degenerate line segment
            return np.linalg.norm(point - line_start)
        
        # Vector from line_start to point
        point_vec = point - line_start
        
        # Project point onto line
        t = np.clip(np.dot(point_vec, line_vec) / line_len_sq, 0.0, 1.0)
        closest_point = line_start + t * line_vec
        
        return np.linalg.norm(point - closest_point)
    
    def is_point_between_boundaries(point, centerline_idx, left_bdry, right_bdry):
        """Check if point is between left and right boundaries at given centerline index.
        
        Uses a simpler approach: compute distance to nearest boundary segment.
        If point is closer to centerline than to boundaries, it's inside.
        """
        if centerline_idx < 0 or centerline_idx >= len(centerline_points):
            return False, 10.0  # Assume outside if invalid index
        
        center_pt = np.array(centerline_points[centerline_idx])
        point_arr = np.array(point)
        
        # Compute distance to centerline
        dist_to_center = np.linalg.norm(point_arr - center_pt)
        
        # Find nearest left and right boundary points to this centerline point
        left_bdry_array = np.array(left_bdry)
        right_bdry_array = np.array(right_bdry)
        left_distances = np.linalg.norm(left_bdry_array - center_pt, axis=1)
        right_distances = np.linalg.norm(right_bdry_array - center_pt, axis=1)
        left_idx = np.argmin(left_distances)
        right_idx = np.argmin(right_distances)
        
        # Compute distances from point to boundary segments
        # Check distance to left boundary segments
        min_left_dist = float('inf')
        for i in range(len(left_bdry) - 1):
            dist = point_to_line_distance(point, left_bdry[i], left_bdry[i+1])
            min_left_dist = min(min_left_dist, dist)
        # Also check distance to nearest left boundary point
        min_left_dist = min(min_left_dist, np.linalg.norm(point_arr - np.array(left_bdry[left_idx])))
        
        # Check distance to right boundary segments
        min_right_dist = float('inf')
        for i in range(len(right_bdry) - 1):
            dist = point_to_line_distance(point, right_bdry[i], right_bdry[i+1])
            min_right_dist = min(min_right_dist, dist)
        # Also check distance to nearest right boundary point
        min_right_dist = min(min_right_dist, np.linalg.norm(point_arr - np.array(right_bdry[right_idx])))
        
        # Compute track width at this centerline point
        left_pt = np.array(left_bdry[left_idx])
        right_pt = np.array(right_bdry[right_idx])
        track_width = np.linalg.norm(left_pt - center_pt) + np.linalg.norm(right_pt - center_pt)
        
        # Point is inside if:
        # 1. Distance to centerline is less than track width/2 + tolerance, OR
        # 2. Point is closer to centerline than to either boundary
        dist_to_nearest_boundary = min(min_left_dist, min_right_dist)
        
        # Heuristic: if point is within track width from centerline, it's likely inside
        # Also check if it's closer to centerline than to boundaries
        is_inside = (dist_to_center < track_width / 2 + tolerance) or (dist_to_center < dist_to_nearest_boundary)
        
        # Compute violation distance
        if is_inside:
            violation_dist = 0.0
        else:
            # How far outside: distance beyond the track boundary
            violation_dist = max(0.0, dist_to_nearest_boundary - (track_width / 2))
        
        return is_inside, violation_dist
    
    # Check each TTL point
    for idx, ttl_point in enumerate(ttl_points):
        # Find nearest centerline point
        distances = np.linalg.norm(centerline_array - ttl_point, axis=1)
        nearest_idx = np.argmin(distances)
        
        # Check if point is within boundaries (main track OR pit lanes)
        # Try main track first, then pit lanes if not in main track
        is_inside_main, violation_dist_main = is_point_between_boundaries(
            ttl_point, nearest_idx, all_left_boundary, all_right_boundary
        )
        
        # Point is in bounds if it's in main track OR pit lanes (already combined above)
        is_inside = is_inside_main
        violation_dist = violation_dist_main
        
        if not is_inside and violation_dist > tolerance:
            points_out_of_bounds.append((idx, ttl_point, violation_dist))
            violation_distances.append(violation_dist)
    
    # Compute statistics
    all_in_bounds = len(points_out_of_bounds) == 0
    max_violation = max(violation_distances) if violation_distances else 0.0
    mean_violation = sum(violation_distances) / len(violation_distances) if violation_distances else 0.0
    
    return {
        'all_in_bounds': all_in_bounds,
        'points_out_of_bounds': points_out_of_bounds,
        'num_out_of_bounds': len(points_out_of_bounds),
        'max_violation_distance': max_violation,
        'mean_violation_distance': mean_violation,
        'total_points': len(ttl_points),
        'in_bounds_percent': 100.0 * (len(ttl_points) - len(points_out_of_bounds)) / len(ttl_points) if ttl_points else 0.0
    }


def extract_track_boundaries(
    xodr_path: str,
    main_track_road_names: Optional[Tuple[str, ...]] = None,
) -> Optional[Dict]:
    """Extract track boundaries and centerline from XODR file.
    
    Args:
        xodr_path: Path to the OpenDRIVE file.
        main_track_road_names: If given, only roads with name in this tuple are included
            for left/right boundary segments (pit is always separate). Use for maps that
            have junction roads so you get a single consistent track envelope.
    
    Returns:
        Dictionary with:
        - 'centerline_points': List of (x, y) centerline points
        - 'left_boundary_segments': List of boundary segments (each is a list of points)
        - 'right_boundary_segments': List of boundary segments
        - 'pit_left_boundary_segments': List of pit lane left boundary segments
        - 'pit_right_boundary_segments': List of pit lane right boundary segments
        Or None if extraction fails
    """
    if not xodr_path or not os.path.exists(xodr_path):
        return None
    
    try:
        from scenic.formats.opendrive import xodr_parser
        from scenic.domains.driving.roads import Network
        
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
            # If main_track_road_names is set, skip non-pit roads that aren't in the list (e.g. junctions)
            if main_track_road_names is not None and not is_pit_lane:
                road_name = getattr(road, 'name', '') or ''
                if road_name.strip() not in main_track_road_names:
                    continue
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
        
        return {
            'centerline_points': centerline_points,
            'left_boundary_segments': left_boundary_segments,
            'right_boundary_segments': right_boundary_segments,
            'pit_left_boundary_segments': pit_left_boundary_segments,
            'pit_right_boundary_segments': pit_right_boundary_segments
        }
    
    except Exception as e:
        print(f"[WARN] Could not extract track boundaries: {e}")
        import traceback
        traceback.print_exc()
        return None


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
    track_boundaries = extract_track_boundaries(xodr_path) if xodr_path else None
    
    if track_boundaries:
        left_boundary_segments = track_boundaries['left_boundary_segments']
        right_boundary_segments = track_boundaries['right_boundary_segments']
        pit_left_boundary_segments = track_boundaries['pit_left_boundary_segments']
        pit_right_boundary_segments = track_boundaries['pit_right_boundary_segments']
        
        # Plot main track boundaries - plot each segment separately to preserve correct ordering
        # This prevents diagonal lines from connecting non-adjacent points
        left_label_added = False
        for segment in left_boundary_segments:
            if segment:
                left_x = [p[0] for p in segment]
                left_y = [p[1] for p in segment]
                label = 'Track Left Boundary' if not left_label_added else None
                ax.plot(left_x, left_y, color='#2C2C2C', linewidth=3.0, alpha=0.9, 
                       label=label, zorder=1)
                left_label_added = True
        
        # Plot right boundary segments
        right_label_added = False
        for segment in right_boundary_segments:
            if segment:
                right_x = [p[0] for p in segment]
                right_y = [p[1] for p in segment]
                label = 'Track Right Boundary' if not right_label_added else None
                ax.plot(right_x, right_y, color='#2C2C2C', linewidth=3.0, alpha=0.9, 
                       label=label, zorder=1)
                right_label_added = True
        
        # Plot pit lane boundaries with a different color/style
        pit_left_label_added = False
        for segment in pit_left_boundary_segments:
            if segment:
                left_x = [p[0] for p in segment]
                left_y = [p[1] for p in segment]
                label = 'Pit Lane Left Boundary' if not pit_left_label_added else None
                ax.plot(left_x, left_y, color='#8B4513', linewidth=3.0, alpha=0.9, 
                       linestyle='--', label=label, zorder=1)
                pit_left_label_added = True
        
        pit_right_label_added = False
        for segment in pit_right_boundary_segments:
            if segment:
                right_x = [p[0] for p in segment]
                right_y = [p[1] for p in segment]
                label = 'Pit Lane Right Boundary' if not pit_right_label_added else None
                ax.plot(right_x, right_y, color='#8B4513', linewidth=3.0, alpha=0.9, 
                       linestyle='--', label=label, zorder=1)
                pit_right_label_added = True
    
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
                       default='assets/ttls/LS_ENU_TTL_CSV',
                       help='Directory containing TTL CSV files (default: assets/ttls/LS_ENU_TTL_CSV)')
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
    parser.add_argument('--compute-transform', action='store_true',
                       help='Compute optimal transformation from TTL to map coordinates')
    parser.add_argument('--transform-method', type=str, default='translation',
                       choices=['translation', 'affine'],
                       help='Transformation method: translation (simple offset) or affine (rotation+translation)')
    parser.add_argument('--save-transform', type=str, default=None,
                       help='Save computed transformation (dx, dy) to JSON file (default: auto-save when --compute-transform is used)')
    
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
    
    # Compute optimal transformation if requested
    if args.compute_transform:
        print("=" * 80)
        print("COMPUTING OPTIMAL TRANSFORMATION")
        print("=" * 80)
        
        if not args.xodr or not os.path.exists(args.xodr):
            print("[ERROR] XODR file required for transformation computation. Use --xodr to specify.")
            sys.exit(1)
        
        print(f"\n[INFO] Extracting map centerline from: {args.xodr}")
        map_centerline = extract_map_centerline(args.xodr)
        
        if not map_centerline:
            print("[ERROR] Could not extract map centerline. Cannot compute transformation.")
            sys.exit(1)
        
        print(f"[INFO] Extracted {len(map_centerline)} centerline points from map")
        
        # Extract track boundaries for optimization (including pit lanes)
        print(f"[INFO] Extracting track boundaries for optimization...")
        track_boundaries = extract_track_boundaries(args.xodr)
        left_boundary_segments = None
        right_boundary_segments = None
        pit_left_boundary_segments = None
        pit_right_boundary_segments = None
        if track_boundaries:
            left_boundary_segments = track_boundaries.get('left_boundary_segments', [])
            right_boundary_segments = track_boundaries.get('right_boundary_segments', [])
            pit_left_boundary_segments = track_boundaries.get('pit_left_boundary_segments', [])
            pit_right_boundary_segments = track_boundaries.get('pit_right_boundary_segments', [])
            if left_boundary_segments and right_boundary_segments:
                print(f"[INFO] Extracted boundaries: {len(left_boundary_segments)} left segments, {len(right_boundary_segments)} right segments")
                if pit_left_boundary_segments and pit_right_boundary_segments:
                    print(f"[INFO] Extracted pit lanes: {len(pit_left_boundary_segments)} left segments, {len(pit_right_boundary_segments)} right segments")
                    print(f"[INFO] Pit lanes will be treated as normal lanes")
                print(f"[INFO] Will optimize transformation to maximize points in bounds")
            else:
                print(f"[WARN] Incomplete boundary data - will use centerline alignment only")
        else:
            print(f"[WARN] Could not extract boundaries - will use centerline alignment only")
        
        # Compute transformation for each TTL (using original coordinates, before applying dx/dy)
        print(f"\n[INFO] Computing transformation using method: {args.transform_method}")
        print("\nNote: Computing transformation from ORIGINAL TTL coordinates (before applying --dx/--dy)")
        print("      Load TTLs without offsets first, then apply computed transformation.\n")
        
        # Reload TTLs without offsets for transformation computation
        ttl_data_original = {}
        for csv_file in csv_files:
            name = csv_file.stem
            points, metadata = read_ttl_xy(str(csv_file), dx=0.0, dy=0.0)  # No offset
            ttl_data_original[name] = points
        
        # Compute transformation for first TTL (or average across all)
        if len(ttl_data_original) > 0:
            # Use first TTL for computation, or combine all TTLs
            first_name = list(ttl_data_original.keys())[0]
            ttl_points = ttl_data_original[first_name]
            
            print(f"[INFO] Using TTL '{first_name}' ({len(ttl_points)} points) for transformation computation")
            
            transform = compute_optimal_transformation(
                ttl_points, 
                map_centerline, 
                method=args.transform_method,
                left_boundary_segments=left_boundary_segments,
                right_boundary_segments=right_boundary_segments,
                pit_left_boundary_segments=pit_left_boundary_segments,
                pit_right_boundary_segments=pit_right_boundary_segments
            )
            
            print(f"\n[RESULT] Optimal Transformation ({transform['type']}):")
            if transform['type'] == 'translation':
                print(f"  dx = {transform['dx']:.3f} meters")
                print(f"  dy = {transform['dy']:.3f} meters")
                if 'std_dx' in transform:
                    print(f"  Standard deviation: dx={transform['std_dx']:.3f}m, dy={transform['std_dy']:.3f}m")
                print(f"  Mean alignment error: {transform['error']:.3f} meters")
                if 'in_bounds_percent' in transform:
                    print(f"  Points in bounds: {transform['in_bounds_percent']:.1f}%")
                if 'method_used' in transform:
                    print(f"  Optimization method: {transform['method_used']}")
                print(f"\n[RECOMMENDATION] Use these offsets:")
                print(f"  --dx {transform['dx']:.3f} --dy {transform['dy']:.3f}")
            else:
                print(f"  Matrix: {transform.get('matrix', 'N/A')}")
                print(f"  Offset: {transform.get('offset', 'N/A')}")
                print(f"  Mean alignment error: {transform['error']:.3f} meters")
            
            # Compare with current offsets
            if args.dx != 0.0 or args.dy != 0.0:
                print(f"\n[COMPARISON] Current offsets: dx={args.dx:.3f}, dy={args.dy:.3f}")
                print(f"             Optimal offsets:  dx={transform['dx']:.3f}, dy={transform['dy']:.3f}")
                diff_dx = transform['dx'] - args.dx
                diff_dy = transform['dy'] - args.dy
                print(f"             Difference:       dx={diff_dx:.3f}, dy={diff_dy:.3f}")
                
                # Check how current transformation performs
                if left_boundary_segments and right_boundary_segments:
                    current_transformed = [(p[0] + args.dx, p[1] + args.dy) for p in ttl_points]
                    current_result = check_ttl_in_bounds(
                        current_transformed,
                        map_centerline,
                        left_boundary_segments,
                        right_boundary_segments,
                        tolerance=0.5,
                        pit_left_boundary_segments=pit_left_boundary_segments,
                        pit_right_boundary_segments=pit_right_boundary_segments
                    )
                    print(f"             Current in-bounds: {current_result.get('in_bounds_percent', 0.0):.1f}%")
                    if 'in_bounds_percent' in transform:
                        improvement = transform['in_bounds_percent'] - current_result.get('in_bounds_percent', 0.0)
                        print(f"             Improvement:      {improvement:+.1f}%")
            
            # Save transformation to file (auto-save if --compute-transform is used)
            if args.compute_transform:
                if args.save_transform:
                    # Use user-specified path
                    output_path = args.save_transform
                else:
                    # Auto-generate filename based on XODR file
                    if args.xodr:
                        xodr_path = Path(args.xodr)
                        # Create filename like: LagunaSeca_transform.json
                        output_path = xodr_path.parent / f"{xodr_path.stem}_transform.json"
                    else:
                        # Fallback to default location
                        output_path = Path("assets/maps/dSPACE/ttl_transform.json")
                
                save_transformation(transform, str(output_path), xodr_path=args.xodr, ttl_name=first_name)
        
        print("\n" + "=" * 80)
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
    
    # Check TTL points are within track boundaries
    if args.xodr and os.path.exists(args.xodr):
        print("\n" + "=" * 80)
        print("BOUNDARY VALIDATION")
        print("=" * 80)
        
        print(f"\n[INFO] Extracting track boundaries from: {args.xodr}")
        track_boundaries = extract_track_boundaries(args.xodr)
        
        if track_boundaries:
            centerline_points = track_boundaries['centerline_points']
            left_boundary_segments = track_boundaries['left_boundary_segments']
            right_boundary_segments = track_boundaries['right_boundary_segments']
            pit_left_boundary_segments = track_boundaries.get('pit_left_boundary_segments', [])
            pit_right_boundary_segments = track_boundaries.get('pit_right_boundary_segments', [])
            
            print(f"[INFO] Extracted {len(centerline_points)} centerline points")
            print(f"[INFO] Extracted {len(left_boundary_segments)} left boundary segments")
            print(f"[INFO] Extracted {len(right_boundary_segments)} right boundary segments")
            if pit_left_boundary_segments and pit_right_boundary_segments:
                print(f"[INFO] Extracted {len(pit_left_boundary_segments)} pit lane left segments, {len(pit_right_boundary_segments)} pit lane right segments")
                print(f"[INFO] Pit lanes will be treated as normal lanes for boundary checking")
            
            if centerline_points and left_boundary_segments and right_boundary_segments:
                boundary_results = {}
                all_ttls_in_bounds = True
                
                for name, (points, metadata) in ttl_data.items():
                    print(f"\n[{name}]")
                    result = check_ttl_in_bounds(
                        points,
                        centerline_points,
                        left_boundary_segments,
                        right_boundary_segments,
                        tolerance=args.tolerance,
                        pit_left_boundary_segments=pit_left_boundary_segments if pit_left_boundary_segments else None,
                        pit_right_boundary_segments=pit_right_boundary_segments if pit_right_boundary_segments else None
                    )
                    boundary_results[name] = result
                    
                    if result.get('all_in_bounds') is None:
                        print("  [SKIP] Cannot validate (missing boundary data)")
                    elif result['all_in_bounds']:
                        print(f"  [OK] All {result['total_points']} points are within track boundaries")
                    else:
                        all_ttls_in_bounds = False
                        print(f"  [WARNING] {result['num_out_of_bounds']}/{result['total_points']} points are outside track boundaries")
                        print(f"            In bounds: {result['in_bounds_percent']:.1f}%")
                        print(f"            Max violation distance: {result['max_violation_distance']:.2f}m")
                        print(f"            Mean violation distance: {result['mean_violation_distance']:.2f}m")
                        
                        # Show first few violations
                        if result['points_out_of_bounds']:
                            print(f"            First few violations:")
                            for idx, point, dist in result['points_out_of_bounds'][:5]:
                                print(f"              Point {idx}: ({point[0]:.2f}, {point[1]:.2f}) - {dist:.2f}m outside")
                            if len(result['points_out_of_bounds']) > 5:
                                print(f"              ... and {len(result['points_out_of_bounds']) - 5} more")
                
                # Summary
                print("\n" + "-" * 80)
                if all_ttls_in_bounds:
                    print("[OK] All TTLs are within track boundaries")
                else:
                    print("[WARNING] Some TTLs have points outside track boundaries")
                    print("\nSummary:")
                    for name, result in boundary_results.items():
                        if result.get('all_in_bounds') is False:
                            print(f"  {name}: {result['num_out_of_bounds']}/{result['total_points']} points outside "
                                  f"({result['in_bounds_percent']:.1f}% in bounds)")
            else:
                print("[WARN] Incomplete boundary data - cannot validate TTL points")
        else:
            print("[WARN] Could not extract track boundaries - cannot validate TTL points")
    else:
        print("\n[INFO] No XODR file provided - skipping boundary validation")
        print("       Use --xodr to specify track file for boundary checking")
    
    # Visualize
    if len(ttl_data) > 0:
        print("\n" + "=" * 80)
        print("GENERATING VISUALIZATION")
        print("=" * 80)
        visualize_all_ttls(ttl_data, xodr_path=args.xodr, 
                          save_path=args.save, show_plot=not args.no_show)


if __name__ == "__main__":
    main()

