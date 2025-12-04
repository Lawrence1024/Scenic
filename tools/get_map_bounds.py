#!/usr/bin/env python3
"""
Get the boundary points and bounding box of a map from an XODR file.

Usage:
    python get_map_bounds.py [--xodr <xodr_path>]
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

