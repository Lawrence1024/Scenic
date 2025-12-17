#!/usr/bin/env python3
"""
Validate the accuracy of the coordinate transform calibration.

This script tests:
1. Round-trip accuracy (XODR -> RD -> XODR)
2. Comparison with calibration data (expected vs actual ControlDesk readbacks)
3. Error distribution across the map
4. Route-dependent accuracy
5. Recommendations for calibration updates

Usage:
    python debug_calibrate_cord/validate_transform_accuracy.py
"""

import sys
import json
import math
import numpy as np
from pathlib import Path
from typing import List, Dict, Tuple, Any

# Add Scenic src to path
scenic_path = Path(__file__).parent.parent / "src"
if scenic_path.exists():
    sys.path.insert(0, str(scenic_path))

from scenic.simulators.dspace.geometry.coordinate_transform import (
    load_transform,
    apply_coordinate_transform,
    apply_inverse_coordinate_transform
)
from scenic.simulators.dspace.geometry.rd_parser import build_rd_road_index


def print_section(title: str):
    """Print a section header."""
    print("\n" + "=" * 80)
    print(title)
    print("=" * 80)


def test_round_trip_accuracy(
    transform: Dict[str, Any],
    test_points: List[Tuple[float, float]],
    tolerance: float = 0.01
) -> Dict[str, Any]:
    """Test round-trip accuracy: XODR -> RD -> XODR."""
    print_section("Round-Trip Accuracy Test")
    
    errors = []
    max_error = 0.0
    max_error_point = None
    
    for xodr_x, xodr_y in test_points:
        # Forward: XODR -> RD
        rd_x, rd_y = apply_coordinate_transform(transform, (xodr_x, xodr_y))
        
        # Reverse: RD -> XODR
        xodr_x_recovered, xodr_y_recovered = apply_inverse_coordinate_transform(
            transform, (rd_x, rd_y)
        )
        
        # Calculate error
        error_x = abs(xodr_x_recovered - xodr_x)
        error_y = abs(xodr_y_recovered - xodr_y)
        error_distance = math.sqrt(error_x**2 + error_y**2)
        
        errors.append(error_distance)
        
        if error_distance > max_error:
            max_error = error_distance
            max_error_point = (xodr_x, xodr_y, error_distance)
        
        if error_distance > tolerance:
            print(f"  ⚠️  Point ({xodr_x:.3f}, {xodr_y:.3f}): error = {error_distance:.6f}m")
    
    mean_error = np.mean(errors)
    std_error = np.std(errors)
    median_error = np.median(errors)
    
    print(f"\nRound-Trip Statistics:")
    print(f"  Mean error:    {mean_error:.6f} m")
    print(f"  Std deviation: {std_error:.6f} m")
    print(f"  Median error:  {median_error:.6f} m")
    print(f"  Max error:     {max_error:.6f} m")
    if max_error_point:
        print(f"  Max error at:  ({max_error_point[0]:.3f}, {max_error_point[1]:.3f})")
    
    return {
        'mean_error': mean_error,
        'std_error': std_error,
        'median_error': median_error,
        'max_error': max_error,
        'errors': errors
    }


def test_against_calibration_data(
    transform: Dict[str, Any],
    calibration_data_path: Path
) -> Dict[str, Any]:
    """Test transform against calibration data (expected vs actual)."""
    print_section("Calibration Data Comparison")
    
    if not calibration_data_path.exists():
        print(f"  ⚠️  Calibration data file not found: {calibration_data_path}")
        return {}
    
    with open(calibration_data_path, 'r') as f:
        calibration_data = json.load(f)
    
    print(f"Testing against {len(calibration_data)} calibration points...")
    
    errors_expected = []  # Error between transform prediction and expected RD
    errors_actual = []    # Error between transform prediction and actual ControlDesk readback
    offsets = []          # Actual offset (expected - actual)
    
    route_errors = {'R1': [], 'R2': []}
    
    for entry in calibration_data:
        name = entry.get('name', 'unknown')
        scenic_xodr = tuple(entry['scenic_xodr'][:2])  # (x, y)
        rd_expected = tuple(entry['rd_expected'][:2])
        rd_actual = tuple(entry['rd_actual'][:2])
        route = entry.get('modeldesk_route', 'R2')
        
        # What does our transform predict?
        rd_predicted = apply_coordinate_transform(transform, scenic_xodr)
        
        # Error: predicted vs expected (what transform should give)
        error_expected = math.sqrt(
            (rd_predicted[0] - rd_expected[0])**2 + 
            (rd_predicted[1] - rd_expected[1])**2
        )
        errors_expected.append(error_expected)
        
        # Error: predicted vs actual (what ControlDesk actually reads)
        error_actual = math.sqrt(
            (rd_predicted[0] - rd_actual[0])**2 + 
            (rd_predicted[1] - rd_actual[1])**2
        )
        errors_actual.append(error_actual)
        
        # Actual offset (expected - actual) - this is the systematic error
        offset = (
            rd_expected[0] - rd_actual[0],
            rd_expected[1] - rd_actual[1]
        )
        offset_magnitude = math.sqrt(offset[0]**2 + offset[1]**2)
        offsets.append(offset_magnitude)
        
        route_errors[route].append(error_actual)
        
        print(f"\n  {name}:")
        print(f"    Scenic XODR: ({scenic_xodr[0]:.6f}, {scenic_xodr[1]:.6f})")
        print(f"    Transform -> RD: ({rd_predicted[0]:.6f}, {rd_predicted[1]:.6f})")
        print(f"    Expected RD:     ({rd_expected[0]:.6f}, {rd_expected[1]:.6f}) [error: {error_expected:.3f}m]")
        print(f"    Actual RD:        ({rd_actual[0]:.6f}, {rd_actual[1]:.6f}) [error: {error_actual:.3f}m]")
        print(f"    Systematic offset: {offset_magnitude:.3f}m")
    
    print(f"\n{'='*80}")
    print("Summary Statistics:")
    print(f"{'='*80}")
    print(f"\nTransform vs Expected RD:")
    print(f"  Mean error:    {np.mean(errors_expected):.3f} m")
    print(f"  Std deviation: {np.std(errors_expected):.3f} m")
    print(f"  Max error:     {np.max(errors_expected):.3f} m")
    
    print(f"\nTransform vs Actual ControlDesk Readback:")
    print(f"  Mean error:    {np.mean(errors_actual):.3f} m")
    print(f"  Std deviation: {np.std(errors_actual):.3f} m")
    print(f"  Max error:     {np.max(errors_actual):.3f} m")
    
    print(f"\nSystematic Offset (Expected - Actual):")
    print(f"  Mean offset:   {np.mean(offsets):.3f} m")
    print(f"  Std deviation: {np.std(offsets):.3f} m")
    print(f"  Max offset:    {np.max(offsets):.3f} m")
    
    if route_errors['R1']:
        print(f"\nRoute R1 (Pit) Errors:")
        print(f"  Mean: {np.mean(route_errors['R1']):.3f} m")
        print(f"  Count: {len(route_errors['R1'])}")
    
    if route_errors['R2']:
        print(f"\nRoute R2 (Lap) Errors:")
        print(f"  Mean: {np.mean(route_errors['R2']):.3f} m")
        print(f"  Count: {len(route_errors['R2'])}")
    
    return {
        'errors_expected': errors_expected,
        'errors_actual': errors_actual,
        'offsets': offsets,
        'route_errors': {k: v for k, v in route_errors.items() if v}
    }


def test_map_coverage(
    transform: Dict[str, Any],
    road_index: Dict[str, Any],
    num_samples: int = 50
) -> Dict[str, Any]:
    """Test transform accuracy across the map by sampling road points."""
    print_section(f"Map Coverage Test ({num_samples} samples)")
    
    # Sample points from all roads
    test_points = []
    roads = road_index.get('roads', {})
    
    for road_name, road_data in roads.items():
        sec_points = road_data.get('sec_points', [[]])
        if not sec_points or not sec_points[0]:
            continue
        
        points = sec_points[0]
        road_length = road_data.get('length', 0)
        
        # Sample evenly along the road
        num_road_samples = max(1, int(num_samples * road_length / sum(
            r.get('length', 0) for r in roads.values()
        )))
        
        for i in range(num_road_samples):
            s_target = (i / num_road_samples) * road_length
            
            # Find point at s
            for j in range(len(points) - 1):
                x0, y0, s0 = points[j]
                x1, y1, s1 = points[j + 1]
                if s0 <= s_target <= s1:
                    u = (s_target - s0) / (s1 - s0) if s1 - s0 > 1e-6 else 0
                    rd_x = x0 + u * (x1 - x0)
                    rd_y = y0 + u * (y1 - y0)
                    test_points.append((rd_x, rd_y, road_name))
                    break
    
    print(f"Sampled {len(test_points)} points from {len(roads)} roads")
    
    # Test round-trip for each point
    errors = []
    road_errors = {}
    
    for rd_x, rd_y, road_name in test_points:
        # RD -> XODR
        xodr_x, xodr_y = apply_inverse_coordinate_transform(transform, (rd_x, rd_y))
        
        # XODR -> RD
        rd_x_recovered, rd_y_recovered = apply_coordinate_transform(
            transform, (xodr_x, xodr_y)
        )
        
        error = math.sqrt(
            (rd_x_recovered - rd_x)**2 + (rd_y_recovered - rd_y)**2
        )
        errors.append(error)
        
        if road_name not in road_errors:
            road_errors[road_name] = []
        road_errors[road_name].append(error)
    
    print(f"\nRound-Trip Statistics:")
    print(f"  Mean error:    {np.mean(errors):.6f} m")
    print(f"  Std deviation: {np.std(errors):.6f} m")
    print(f"  Median error:  {np.median(errors):.6f} m")
    print(f"  Max error:     {np.max(errors):.6f} m")
    
    print(f"\nPer-Road Statistics:")
    for road_name, road_errs in sorted(road_errors.items()):
        print(f"  {road_name}:")
        print(f"    Mean: {np.mean(road_errs):.6f} m")
        print(f"    Max:  {np.max(road_errs):.6f} m")
        print(f"    Count: {len(road_errs)}")
    
    return {
        'mean_error': np.mean(errors),
        'std_error': np.std(errors),
        'max_error': np.max(errors),
        'road_errors': road_errors
    }


def analyze_error_patterns(
    calibration_data_path: Path,
    transform: Dict[str, Any]
) -> Dict[str, Any]:
    """Analyze error patterns to determine if offset is constant, position-dependent, or route-dependent."""
    print_section("Error Pattern Analysis")
    
    if not calibration_data_path.exists():
        print(f"  ⚠️  Calibration data file not found")
        return {}
    
    with open(calibration_data_path, 'r') as f:
        calibration_data = json.load(f)
    
    # Collect offsets by route
    offsets_by_route = {'R1': [], 'R2': []}
    positions_by_route = {'R1': [], 'R2': []}
    
    for entry in calibration_data:
        scenic_xodr = tuple(entry['scenic_xodr'][:2])
        rd_expected = tuple(entry['rd_expected'][:2])
        rd_actual = tuple(entry['rd_actual'][:2])
        route = entry.get('modeldesk_route', 'R2')
        
        # Offset vector
        offset = (
            rd_expected[0] - rd_actual[0],
            rd_expected[1] - rd_actual[1]
        )
        offset_mag = math.sqrt(offset[0]**2 + offset[1]**2)
        
        offsets_by_route[route].append(offset)
        positions_by_route[route].append(scenic_xodr)
    
    print("Offset Analysis by Route:")
    for route in ['R1', 'R2']:
        if not offsets_by_route[route]:
            continue
        
        offsets = offsets_by_route[route]
        offset_x = [o[0] for o in offsets]
        offset_y = [o[1] for o in offsets]
        
        mean_offset_x = np.mean(offset_x)
        mean_offset_y = np.mean(offset_y)
        std_offset_x = np.std(offset_x)
        std_offset_y = np.std(offset_y)
        
        print(f"\n  Route {route}:")
        print(f"    Mean offset: ({mean_offset_x:.3f}, {mean_offset_y:.3f}) m")
        print(f"    Std deviation: ({std_offset_x:.3f}, {std_offset_y:.3f}) m")
        print(f"    Count: {len(offsets)}")
        
        # Determine if offset is constant
        if std_offset_x < 1.0 and std_offset_y < 1.0:
            print(f"    [OK] Offset appears CONSTANT (low variance)")
        elif std_offset_x < 5.0 and std_offset_y < 5.0:
            print(f"    [WARN] Offset has MODERATE variance (may be position-dependent)")
        else:
            print(f"    [ERROR] Offset has HIGH variance (likely position-dependent)")
    
    # Check if offsets differ between routes
    if offsets_by_route['R1'] and offsets_by_route['R2']:
        mean_r1 = np.mean([math.sqrt(o[0]**2 + o[1]**2) for o in offsets_by_route['R1']])
        mean_r2 = np.mean([math.sqrt(o[0]**2 + o[1]**2) for o in offsets_by_route['R2']])
        
        print(f"\n  Route Comparison:")
        print(f"    R1 mean offset magnitude: {mean_r1:.3f} m")
        print(f"    R2 mean offset magnitude: {mean_r2:.3f} m")
        print(f"    Difference: {abs(mean_r1 - mean_r2):.3f} m")
        
        if abs(mean_r1 - mean_r2) > 2.0:
            print(f"    [WARN] Routes have DIFFERENT offset patterns (route-dependent)")
        else:
            print(f"    [OK] Routes have SIMILAR offset patterns")
    
    return {
        'offsets_by_route': {
            k: [list(o) for o in v] for k, v in offsets_by_route.items() if v
        }
    }


def generate_recommendations(
    round_trip_results: Dict[str, Any],
    calibration_results: Dict[str, Any],
    coverage_results: Dict[str, Any],
    pattern_results: Dict[str, Any]
) -> List[str]:
    """Generate recommendations based on analysis."""
    print_section("Recommendations")
    
    recommendations = []
    
    # Check round-trip accuracy
    if round_trip_results.get('max_error', 0) < 0.01:
        recommendations.append("[OK] Round-trip accuracy is EXCELLENT (< 0.01m) - transform is mathematically sound")
    elif round_trip_results.get('max_error', 0) < 0.1:
        recommendations.append("[OK] Round-trip accuracy is GOOD (< 0.1m) - transform is mathematically sound")
    elif round_trip_results.get('max_error', 0) < 1.0:
        recommendations.append("[WARN] Round-trip accuracy is MODERATE (< 1.0m) - transform may have numerical issues")
    else:
        recommendations.append("[ERROR] Round-trip accuracy is POOR (> 1.0m) - transform has significant errors")
    
    # Check calibration data comparison
    if calibration_results:
        mean_error_actual = np.mean(calibration_results.get('errors_actual', []))
        mean_offset = np.mean(calibration_results.get('offsets', []))
        
        if mean_error_actual < 1.0:
            recommendations.append("[OK] Transform matches ControlDesk readbacks well (< 1m error)")
        elif mean_error_actual < 5.0:
            recommendations.append("[WARN] Transform has moderate errors vs ControlDesk readbacks (1-5m)")
        else:
            recommendations.append("[ERROR] Transform has large errors vs ControlDesk readbacks (> 5m)")
        
        if mean_offset > 2.0:
            recommendations.append(
                f"[WARN] Significant systematic offset detected ({mean_offset:.2f}m) - "
                "this suggests the transform is correct but there's an additional offset "
                "in the (s,t) -> ControlDesk pipeline"
            )
    
    # Check route dependency
    if pattern_results.get('offsets_by_route'):
        routes = list(pattern_results['offsets_by_route'].keys())
        if len(routes) > 1:
            recommendations.append(
                "[WARN] Different routes show different offset patterns - "
                "may need route-specific corrections"
            )
    
    # Overall recommendation
    print("\n" + "=" * 80)
    print("OVERALL ASSESSMENT")
    print("=" * 80)
    
    if round_trip_results.get('max_error', 0) < 0.1:
        if calibration_results and np.mean(calibration_results.get('errors_actual', [])) < 5.0:
            print("\n[OK] RECOMMENDATION: Transform is ACCURATE")
            print("   The coordinate transform itself is mathematically correct.")
            print("   Any positioning errors are likely due to:")
            print("   - Systematic offsets in the (s,t) -> ControlDesk pipeline")
            print("   - Route-specific coordinate system differences")
            print("   - dSPACE model behavior")
            print("\n   ACTION: Keep current transform, but consider:")
            print("   - Adding route-specific offset corrections")
            print("   - Building a direct XODR -> ControlDesk RD transform")
        else:
            print("\n[WARN] RECOMMENDATION: Transform needs VALIDATION")
            print("   The transform is mathematically correct but may not match")
            print("   actual ControlDesk outputs. Consider:")
            print("   - Re-calibrating with more measurement points")
            print("   - Building a direct transform from calibration data")
    else:
        print("\n[ERROR] RECOMMENDATION: Transform needs UPDATE")
        print("   The transform has significant errors. Consider:")
        print("   - Re-building the transform with more calibration points")
        print("   - Using a different transformation method (e.g., piecewise)")
        print("   - Validating the XODR and RD geometry files")
    
    return recommendations


def main():
    """Main validation function."""
    print("=" * 80)
    print("COORDINATE TRANSFORM VALIDATION")
    print("=" * 80)
    
    # Paths
    scenic_root = Path(__file__).parent.parent
    transform_path = scenic_root / "assets" / "maps" / "dSPACE" / "Laguna_Seca_transform.json"
    calibration_data_path = Path(__file__).parent / "calibration_data.json"
    rd_path = scenic_root / "assets" / "maps" / "dSPACE" / "Laguna_Seca.rd"
    
    # Load transform
    if not transform_path.exists():
        print(f"ERROR: Transform file not found: {transform_path}")
        return 1
    
    print(f"\nLoading transform from: {transform_path}")
    transform = load_transform(str(transform_path))
    print(f"  Type: {transform.get('type', 'unknown')}")
    print(f"  Calibration points: {transform.get('num_calibration_points', 'unknown')}")
    
    # Load road index for map coverage test
    road_index = None
    if rd_path.exists():
        print(f"\nLoading road index from: {rd_path}")
        road_index = build_rd_road_index(str(rd_path))
        print(f"  Roads: {list(road_index.get('roads', {}).keys())}")
    
    # Test 1: Round-trip accuracy
    test_points = [
        (163.545, 48.302),      # Ego (pit lane)
        (-101.919263, -457.524908),  # Fellow1
        (0.948038, -272.443171),     # Fellow2
        (191.994781, -418.905118),   # Fellow3
        (70.167889, 109.074718),     # Route R2 s=175
    ]
    
    round_trip_results = test_round_trip_accuracy(transform, test_points)
    
    # Test 2: Calibration data comparison
    calibration_results = test_against_calibration_data(transform, calibration_data_path)
    
    # Test 3: Map coverage
    coverage_results = {}
    if road_index:
        coverage_results = test_map_coverage(transform, road_index, num_samples=50)
    
    # Test 4: Error pattern analysis
    pattern_results = analyze_error_patterns(calibration_data_path, transform)
    
    # Generate recommendations
    recommendations = generate_recommendations(
        round_trip_results,
        calibration_results,
        coverage_results,
        pattern_results
    )
    
    # Save results
    output_path = Path(__file__).parent / "validation_results.json"
    results = {
        'round_trip': round_trip_results,
        'calibration': calibration_results,
        'coverage': coverage_results,
        'patterns': pattern_results,
        'recommendations': recommendations
    }
    
    with open(output_path, 'w') as f:
        json.dump(results, f, indent=2)
    
    print(f"\n{'='*80}")
    print(f"Results saved to: {output_path}")
    print("=" * 80)
    
    return 0


if __name__ == "__main__":
    sys.exit(main())

