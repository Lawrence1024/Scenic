#!/usr/bin/env python3
"""
Transform TTL CSV files by applying coordinate offsets (dx, dy).

This script:
1. Reads original TTL CSV files (in ENU GPS coordinates)
2. Applies transformation offsets (dx, dy) to align with map coordinates
3. Writes transformed CSV files to output directory
4. Preserves metadata and file structure

Usage:
    # Transform with specified offsets
    python transform_ttl_files.py --input-dir <input> --output-dir <output> --dx <dx> --dy <dy>
    
    # Auto-compute optimal offsets and transform
    python transform_ttl_files.py --input-dir <input> --output-dir <output> --auto-compute --xodr <xodr_path>
    
    # Transform single file
    python transform_ttl_files.py --input-file <file.csv> --output-file <output.csv> --dx <dx> --dy <dy>

Examples:
    # Transform all TTLs in directory with known offsets
    python transform_ttl_files.py \
        --input-dir assets/ttls/LS_ENU_TTL_CSV \
        --output-dir assets/ttls/LS_ENU_TTL_CSV \
        --dx -2.0 --dy -53.0
    
    # Auto-compute and transform
    python transform_ttl_files.py \
        --input-dir assets/ttls/LS_ENU_TTL_CSV \
        --output-dir assets/ttls/LS_ENU_TTL_CSV \
        --auto-compute \
        --xodr assets/maps/dSPACE/LagunaSeca.xodr
"""

import os
import sys
import csv
import argparse
import shutil
from pathlib import Path
from typing import Tuple, Dict, Optional

# Add Scenic to path if needed
scenic_path = Path(__file__).parent.parent / "src"
if scenic_path.exists():
    sys.path.insert(0, str(scenic_path))


def read_ttl_metadata(csv_path: str) -> Tuple[Optional[Dict], list]:
    """Read TTL CSV file and return metadata and raw points (without transformation).
    
    Returns:
        (metadata_dict, list_of_raw_points)
    """
    metadata = None
    raw_points = []
    
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
        except (StopIteration, ValueError):
            pass
        
        # Read waypoint data (raw, without transformation)
        for i, row in enumerate(reader):
            if not row or len(row) < 2:
                continue
            try:
                x = float(row[0])
                y = float(row[1])
                raw_points.append((x, y))
            except (ValueError, IndexError):
                if i < 5:  # Only print first few errors
                    print(f"[WARN] {os.path.basename(csv_path)}: Skipping malformed row {i+2}: {row}")
    
    return metadata, raw_points


def write_ttl_csv(output_path: str, metadata: Optional[Dict], points: list):
    """Write TTL CSV file with metadata and transformed points.
    
    Args:
        output_path: Path to output CSV file
        metadata: Metadata dictionary (or None)
        points: List of (x, y) transformed points
    """
    # Create output directory if it doesn't exist
    output_dir = os.path.dirname(output_path)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir, exist_ok=True)
    
    with open(output_path, 'w', newline='') as f:
        writer = csv.writer(f)
        
        # Write metadata line
        if metadata:
            meta_row = [
                metadata.get('id', ''),
                metadata.get('num_points', len(points)),
                metadata.get('length', ''),
                metadata.get('lat', ''),
                metadata.get('lon', ''),
                metadata.get('elevation', ''),
            ]
            writer.writerow(meta_row)
        else:
            # Write empty metadata line if not available
            writer.writerow(['', len(points), '', '', '', ''])
        
        # Write transformed points
        for x, y in points:
            writer.writerow([f"{x:.6f}", f"{y:.6f}"])
    
    print(f"[SAVED] {output_path} ({len(points)} points)")


def transform_ttl_file(input_path: str, output_path: str, dx: float, dy: float):
    """Transform a single TTL CSV file by applying offsets.
    
    Args:
        input_path: Path to input TTL CSV file
        output_path: Path to output TTL CSV file
        dx: X offset to apply
        dy: Y offset to apply
    """
    # Read original file
    metadata, raw_points = read_ttl_metadata(input_path)
    
    if not raw_points:
        print(f"[ERROR] No valid points found in {input_path}")
        return False
    
    # Apply transformation
    transformed_points = [(x + dx, y + dy) for x, y in raw_points]
    
    # Update metadata if available
    if metadata and metadata.get('num_points'):
        metadata['num_points'] = len(transformed_points)
    
    # Write transformed file
    write_ttl_csv(output_path, metadata, transformed_points)
    
    return True


def compute_optimal_offsets(input_dir: str, xodr_path: str) -> Tuple[float, float]:
    """Compute optimal dx/dy offsets by matching TTL points to map centerline.
    
    Args:
        input_dir: Directory containing TTL CSV files
        xodr_path: Path to XODR map file
    
    Returns:
        (dx, dy) optimal offsets
    """
    # Import functions from compare_ttls
    try:
        from compare_ttls import extract_map_centerline, compute_optimal_transformation
    except ImportError:
        # If import fails, try adding tools directory to path
        tools_dir = Path(__file__).parent
        sys.path.insert(0, str(tools_dir))
        from compare_ttls import extract_map_centerline, compute_optimal_transformation
    
    print(f"[INFO] Computing optimal transformation...")
    print(f"       Input directory: {input_dir}")
    print(f"       Map file: {xodr_path}")
    
    # Extract map centerline
    print(f"[INFO] Extracting map centerline...")
    map_centerline = extract_map_centerline(xodr_path)
    if not map_centerline:
        raise ValueError(f"Could not extract map centerline from {xodr_path}")
    print(f"[INFO] Extracted {len(map_centerline)} centerline points")
    
    # Find first TTL file for computation
    ttl_dir = Path(input_dir)
    csv_files = sorted(ttl_dir.glob('*.csv'))
    if not csv_files:
        raise ValueError(f"No CSV files found in {input_dir}")
    
    # Use first TTL file
    first_ttl = csv_files[0]
    print(f"[INFO] Using {first_ttl.name} for transformation computation")
    
    # Read TTL points (without offset)
    _, raw_points = read_ttl_metadata(str(first_ttl))
    if not raw_points:
        raise ValueError(f"No valid points in {first_ttl}")
    
    # Compute transformation
    print(f"[INFO] Computing optimal transformation...")
    transform = compute_optimal_transformation(raw_points, map_centerline, method='translation')
    
    if transform['type'] != 'translation':
        raise ValueError(f"Expected translation transform, got {transform['type']}")
    
    dx = transform['dx']
    dy = transform['dy']
    error = transform['error']
    
    print(f"[RESULT] Optimal offsets:")
    print(f"         dx = {dx:.6f} meters")
    print(f"         dy = {dy:.6f} meters")
    print(f"         Mean alignment error: {error:.3f} meters")
    
    if 'std_dx' in transform:
        print(f"         Std deviation: dx={transform['std_dx']:.3f}m, dy={transform['std_dy']:.3f}m")
    
    return dx, dy


def transform_directory(input_dir: str, output_dir: str, dx: float, dy: float, 
                       overwrite: bool = False):
    """Transform all TTL CSV files in a directory.
    
    Args:
        input_dir: Input directory containing TTL CSV files
        output_dir: Output directory for transformed files
        dx: X offset to apply
        dy: Y offset to apply
        overwrite: Whether to overwrite existing files
    """
    input_path = Path(input_dir)
    output_path = Path(output_dir)
    
    if not input_path.exists():
        print(f"[ERROR] Input directory not found: {input_dir}")
        return False
    
    # Find all CSV files
    csv_files = sorted(input_path.glob('*.csv'))
    if not csv_files:
        print(f"[ERROR] No CSV files found in: {input_dir}")
        return False
    
    print(f"[INFO] Found {len(csv_files)} TTL CSV files")
    print(f"[INFO] Transformation: dx={dx:.6f}, dy={dy:.6f}")
    print(f"[INFO] Output directory: {output_dir}")
    print()
    
    # Create output directory
    output_path.mkdir(parents=True, exist_ok=True)
    
    # Transform each file
    success_count = 0
    for csv_file in csv_files:
        output_file = output_path / csv_file.name
        
        # Check if output file exists
        if output_file.exists() and not overwrite:
            print(f"[SKIP] {csv_file.name} (already exists, use --overwrite to replace)")
            continue
        
        print(f"[TRANSFORM] {csv_file.name}...")
        if transform_ttl_file(str(csv_file), str(output_file), dx, dy):
            success_count += 1
        else:
            print(f"[ERROR] Failed to transform {csv_file.name}")
    
    print()
    print(f"[SUMMARY] Successfully transformed {success_count}/{len(csv_files)} files")
    print(f"[INFO] Transformed files saved to: {output_dir}")
    
    return success_count == len(csv_files)


def main():
    parser = argparse.ArgumentParser(
        description='Transform TTL CSV files by applying coordinate offsets',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    
    # Input/output options
    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument('--input-dir', type=str,
                           help='Input directory containing TTL CSV files')
    input_group.add_argument('--input-file', type=str,
                           help='Input TTL CSV file (single file mode)')
    
    output_group = parser.add_mutually_exclusive_group(required=True)
    output_group.add_argument('--output-dir', type=str,
                            help='Output directory for transformed files (directory mode)')
    output_group.add_argument('--output-file', type=str,
                            help='Output TTL CSV file (single file mode)')
    
    # Transformation options
    transform_group = parser.add_mutually_exclusive_group(required=True)
    transform_group.add_argument('--dx', type=float,
                               help='X offset to apply (in meters)')
    transform_group.add_argument('--auto-compute', action='store_true',
                                help='Automatically compute optimal offsets from map')
    
    parser.add_argument('--dy', type=float,
                       help='Y offset to apply (in meters, required if --dx specified)')
    parser.add_argument('--xodr', type=str,
                       help='XODR map file path (required for --auto-compute)')
    parser.add_argument('--overwrite', action='store_true',
                       help='Overwrite existing output files')
    
    args = parser.parse_args()
    
    # Validate arguments
    if args.input_file and args.output_dir:
        parser.error("--input-file requires --output-file (not --output-dir)")
    if args.input_dir and args.output_file:
        parser.error("--input-dir requires --output-dir (not --output-file)")
    
    if args.auto_compute:
        if not args.xodr:
            # Try to auto-detect
            default_xodr = Path('assets/maps/dSPACE/LagunaSeca.xodr')
            if default_xodr.exists():
                args.xodr = str(default_xodr)
                print(f"[INFO] Auto-detected map file: {args.xodr}")
            else:
                parser.error("--auto-compute requires --xodr (or auto-detect failed)")
        
        if not os.path.exists(args.xodr):
            parser.error(f"XODR file not found: {args.xodr}")
    
    if args.dx is not None:
        if args.dy is None:
            parser.error("--dx requires --dy")
        dx, dy = args.dx, args.dy
    elif args.auto_compute:
        # Compute optimal offsets
        input_path = args.input_dir if args.input_dir else os.path.dirname(args.input_file)
        dx, dy = compute_optimal_offsets(input_path, args.xodr)
        print()
    else:
        parser.error("Must specify either --dx/--dy or --auto-compute")
    
    # Transform files
    if args.input_file:
        # Single file mode
        if not os.path.exists(args.input_file):
            print(f"[ERROR] Input file not found: {args.input_file}")
            sys.exit(1)
        
        print(f"[INFO] Transforming single file...")
        print(f"       Input:  {args.input_file}")
        print(f"       Output: {args.output_file}")
        print(f"       Offsets: dx={dx:.6f}, dy={dy:.6f}")
        print()
        
        success = transform_ttl_file(args.input_file, args.output_file, dx, dy)
        if not success:
            sys.exit(1)
    else:
        # Directory mode
        success = transform_directory(args.input_dir, args.output_dir, dx, dy, 
                                     overwrite=args.overwrite)
        if not success:
            sys.exit(1)
    
    print()
    print("[SUCCESS] Transformation complete!")


if __name__ == "__main__":
    main()

