"""
Automatic Coordinate Transformation: Scenic (XODR-based) → dSPACE (RD-based)

This module automatically builds a transformation between:
- Scenic's coordinate system (based on XODR parsing)
- dSPACE/Aurelion's coordinate system (based on RD file)

The transformation is computed by comparing XODR and RD geometries at sample points.
"""

import math
import numpy as np
from typing import Tuple, Optional, List
import xml.etree.ElementTree as ET


def build_coordinate_transform(xodr_path: str, rd_path: str, num_samples: int = 50) -> dict:
    """Automatically build transformation from XODR coordinates to RD coordinates.
    
    This compares the two coordinate systems at multiple sample points along the track
    and computes the transformation needed to align them.
    
    Args:
        xodr_path: Path to XODR file (what Scenic uses)
        rd_path: Path to RD file (what Aurelion uses)
        num_samples: Number of sample points to use for calibration
        
    Returns:
        Dictionary with transformation parameters:
        - 'type': 'affine' or 'piecewise'
        - 'matrix': Affine transformation matrix (if type='affine')
        - 'offset': Translation offset (x, y)
        - 'rotation': Rotation angle in radians
        - 'scale': Scale factor (x, y)
        - 'calibration_points': List of (s, xodr_x, xodr_y, rd_x, rd_y) tuples
    """
    from .rd_parser import parse_rd_geometry
    from .xodr_parser import build_xodr_sec_points
    
    print("\n" + "="*80)
    print("BUILDING AUTOMATIC COORDINATE TRANSFORMATION")
    print("="*80)
    
    # 1. Parse both geometries
    print(f"\n1. Parsing geometries...")
    
    # Parse RD file
    rd_roads = parse_rd_geometry(rd_path, step=0.5)
    rd_main = max(rd_roads, key=lambda r: r['total_length'])
    rd_points = rd_main['sec_points'][0]  # List of (x, y, s)
    print(f"   RD: {len(rd_points)} points, length={rd_main['total_length']:.1f}m")
    
    # Parse XODR file
    xodr_index = build_xodr_sec_points(xodr_path)
    # Get the longest road from XODR
    xodr_main = max(xodr_index['roads'].values(), key=lambda r: r['length'])
    xodr_points = xodr_main['sec_points'][0]  # List of (x, y, s)
    print(f"   XODR: {len(xodr_points)} points")
    
    # 2. Sample points at regular s-intervals
    print(f"\n2. Sampling {num_samples} calibration points...")
    
    max_s = min(rd_main['total_length'], max(p[2] for p in xodr_points))
    s_samples = np.linspace(0, max_s, num_samples)
    
    calibration_points = []
    for s in s_samples:
        # Find XODR point at this s
        xodr_pt = _find_point_at_s(xodr_points, s)
        # Find RD point at this s
        rd_pt = _find_point_at_s(rd_points, s)
        
        if xodr_pt and rd_pt:
            xodr_x, xodr_y, _ = xodr_pt
            rd_x, rd_y, _ = rd_pt
            calibration_points.append((s, xodr_x, xodr_y, rd_x, rd_y))
    
    print(f"   Found {len(calibration_points)} valid calibration points")
    
    # 3. Compute transformation
    print(f"\n3. Computing transformation...")
    
    # Extract coordinate pairs
    xodr_coords = np.array([[pt[1], pt[2]] for pt in calibration_points])  # (xodr_x, xodr_y)
    rd_coords = np.array([[pt[3], pt[4]] for pt in calibration_points])    # (rd_x, rd_y)
    
    # Compute differences
    diffs = rd_coords - xodr_coords
    mean_offset = np.mean(diffs, axis=0)
    std_offset = np.std(diffs, axis=0)
    
    print(f"   Mean offset: dx={mean_offset[0]:.3f}m, dy={mean_offset[1]:.3f}m")
    print(f"   Std deviation: dx={std_offset[0]:.3f}m, dy={std_offset[1]:.3f}m")
    
    # Check if simple translation is sufficient
    if std_offset[0] < 5.0 and std_offset[1] < 5.0:
        print(f"   → Simple translation transform (low variance)")
        transform_type = 'translation'
        transform = {
            'type': 'translation',
            'offset': tuple(mean_offset),
            'calibration_points': calibration_points
        }
    else:
        print(f"   → Complex transform needed (high variance)")
        # Compute affine transformation using least squares
        transform = _compute_affine_transform(xodr_coords, rd_coords, calibration_points)
    
    # 4. Validate transformation
    print(f"\n4. Validating transformation...")
    errors = []
    for i, (s, xodr_x, xodr_y, rd_x, rd_y) in enumerate(calibration_points[::5]):  # Sample every 5th
        transformed = apply_coordinate_transform(transform, (xodr_x, xodr_y))
        error = math.sqrt((transformed[0] - rd_x)**2 + (transformed[1] - rd_y)**2)
        errors.append(error)
        if i < 5:  # Show first few
            print(f"   s={s:7.1f}m: XODR({xodr_x:7.1f},{xodr_y:7.1f}) → "
                  f"RD({rd_x:7.1f},{rd_y:7.1f}) [predicted ({transformed[0]:7.1f},{transformed[1]:7.1f})] "
                  f"error={error:.2f}m")
    
    mean_error = np.mean(errors)
    max_error = np.max(errors)
    print(f"   Mean error: {mean_error:.2f}m")
    print(f"   Max error: {max_error:.2f}m")
    
    if mean_error < 2.0:
        print(f"   ✅ Transform validated successfully!")
    elif mean_error < 5.0:
        print(f"   ⚠️  Transform has moderate errors")
    else:
        print(f"   ❌ Transform has high errors - may need piecewise calibration")
    
    print("="*80 + "\n")
    
    return transform


def _find_point_at_s(points: List[Tuple[float, float, float]], s_target: float) -> Optional[Tuple[float, float, float]]:
    """Find point closest to target s-coordinate."""
    if not points:
        return None
    
    # Find closest point by s-coordinate
    min_ds = float('inf')
    best_pt = None
    for pt in points:
        ds = abs(pt[2] - s_target)
        if ds < min_ds:
            min_ds = ds
            best_pt = pt
    
    return best_pt


def _compute_affine_transform(xodr_coords: np.ndarray, rd_coords: np.ndarray, 
                               calibration_points: List) -> dict:
    """Compute affine transformation matrix using least squares.
    
    The affine transform is: [rd_x, rd_y]^T = A * [xodr_x, xodr_y]^T + b
    where A is a 2x2 matrix and b is a 2D offset.
    """
    # Build system: rd = A * xodr + b
    # In homogeneous coordinates: [rd_x, rd_y, 1]^T = M * [xodr_x, xodr_y, 1]^T
    
    n = len(xodr_coords)
    
    # Add homogeneous coordinate
    xodr_homo = np.hstack([xodr_coords, np.ones((n, 1))])
    
    # Solve for transformation matrix using least squares
    # M^T = (xodr_homo^T * xodr_homo)^-1 * xodr_homo^T * rd_coords
    M_x = np.linalg.lstsq(xodr_homo, rd_coords[:, 0], rcond=None)[0]
    M_y = np.linalg.lstsq(xodr_homo, rd_coords[:, 1], rcond=None)[0]
    
    # Extract components
    A = np.array([[M_x[0], M_x[1]], 
                  [M_y[0], M_y[1]]])
    b = np.array([M_x[2], M_y[2]])
    
    # Decompose affine matrix into interpretable components
    # A = [[cos(θ)*sx, -sin(θ)*sy], [sin(θ)*sx, cos(θ)*sy]]
    sx = math.sqrt(A[0,0]**2 + A[1,0]**2)
    sy = math.sqrt(A[0,1]**2 + A[1,1]**2)
    theta = math.atan2(A[1,0], A[0,0])
    
    return {
        'type': 'affine',
        'matrix': A.tolist(),
        'offset': tuple(b),
        'rotation': theta,
        'scale': (sx, sy),
        'calibration_points': calibration_points
    }


def apply_coordinate_transform(transform: dict, pos: Tuple[float, float]) -> Tuple[float, float]:
    """Apply coordinate transformation to convert XODR coords to RD coords.
    
    Args:
        transform: Transform dictionary from build_coordinate_transform()
        pos: (x, y) in XODR coordinate system (from Scenic)
        
    Returns:
        (x', y') in RD coordinate system (for dSPACE/Aurelion)
    """
    x, y = pos
    
    if transform['type'] == 'translation':
        # Simple translation
        dx, dy = transform['offset']
        return (x + dx, y + dy)
    
    elif transform['type'] == 'affine':
        # Affine transformation
        A = np.array(transform['matrix'])
        b = np.array(transform['offset'])
        pos_vec = np.array([x, y])
        result = A @ pos_vec + b
        return tuple(result)
    
    else:
        # Unknown transform type
        return pos


def apply_inverse_coordinate_transform(transform: dict, pos: Tuple[float, float]) -> Tuple[float, float]:
    """Apply inverse coordinate transformation to convert RD coords back to XODR coords.
    
    This is the inverse of apply_coordinate_transform(). It converts from RD coordinates
    (as returned by dSPACE/ControlDesk) back to Scenic/XODR coordinates.
    
    Args:
        transform: Transform dictionary from build_coordinate_transform()
        pos: (x, y) in RD coordinate system (from dSPACE/ControlDesk)
        
    Returns:
        (x', y') in XODR coordinate system (for Scenic)
    """
    x, y = pos
    
    if transform['type'] == 'translation':
        # Inverse of translation: subtract the offset
        dx, dy = transform['offset']
        return (x - dx, y - dy)
    
    elif transform['type'] == 'affine':
        # Inverse of affine transformation: A^-1 * (pos - b)
        A = np.array(transform['matrix'])
        b = np.array(transform['offset'])
        pos_vec = np.array([x, y])
        
        # Compute inverse matrix
        try:
            A_inv = np.linalg.inv(A)
            result = A_inv @ (pos_vec - b)
            return tuple(result)
        except np.linalg.LinAlgError:
            # If matrix is singular, fall back to identity
            print(f"[coordinate_transform] Warning: Affine matrix is singular, cannot invert. Returning original position.")
            return pos
    
    else:
        # Unknown transform type
        return pos


def save_transform(transform: dict, output_path: str):
    """Save transformation to JSON file for reuse."""
    import json
    
    # Make numpy arrays JSON-serializable
    save_data = {
        'type': transform['type'],
        'offset': transform['offset'],
    }
    
    if 'matrix' in transform:
        save_data['matrix'] = transform['matrix']
    if 'rotation' in transform:
        save_data['rotation'] = transform['rotation']
    if 'scale' in transform:
        save_data['scale'] = transform['scale']
    
    # Don't save full calibration points (too large)
    save_data['num_calibration_points'] = len(transform.get('calibration_points', []))
    
    with open(output_path, 'w') as f:
        json.dump(save_data, f, indent=2)
    
    print(f"Saved transformation to: {output_path}")


def load_transform(input_path: str) -> dict:
    """Load transformation from JSON file."""
    import json
    
    with open(input_path, 'r') as f:
        transform = json.load(f)
    
    print(f"Loaded transformation from: {input_path}")
    return transform


if __name__ == "__main__":
    import sys
    import os
    
    # Add parent directory to path for imports
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../..'))
    
    # Test the coordinate transformation builder
    xodr_path = os.path.join(os.path.dirname(__file__), '../../../assets/maps/dSPACE/LagunaSeca.xodr')
    rd_path = os.path.join(os.path.dirname(__file__), '../../../assets/maps/dSPACE/Laguna_Seca.rd')
    
    # Build transform
    transform = build_coordinate_transform(xodr_path, rd_path, num_samples=100)
    
    # Test some points
    print("\nTesting transformation:")
    test_points = [
        (-101.92, -457.52),
        (-109.05, -412.06),
        (200.60, -826.06),
    ]
    
    for x, y in test_points:
        x_new, y_new = apply_coordinate_transform(transform, (x, y))
        print(f"  ({x:8.2f}, {y:8.2f}) → ({x_new:8.2f}, {y_new:8.2f})")
    
    # Save for later use
    output_path = os.path.join(os.path.dirname(__file__), '../../../assets/maps/dSPACE/coordinate_transform_test.json')
    save_transform(transform, output_path)

