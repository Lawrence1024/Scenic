#!/usr/bin/env python3
"""
Find the XODR coordinate for Route R2 s=175 by working directly with XODR geometry.

The issue: Converting from RD -> XODR using the transform doesn't guarantee
the coordinate is on the drivable region in XODR space, because RD and XODR
geometries may differ slightly.

Solution: Find the coordinate directly from XODR geometry at the appropriate
location, ensuring it's actually on-road.
"""

import sys
import math
from pathlib import Path

# Add Scenic src to path
scenic_path = Path(__file__).parent.parent / "src"
if scenic_path.exists():
    sys.path.insert(0, str(scenic_path))

from scenic.domains.driving.roads import Network
from scenic.simulators.dspace.geometry.rd_parser import build_rd_road_index
from scenic.simulators.dspace.geometry.route_projection import ROUTE_ROAD_SEQUENCES, ROUTE_TRANSITION_POINTS
from scenic.simulators.dspace.geometry.coordinate_transform import (
    load_transform,
    apply_inverse_coordinate_transform,
    apply_coordinate_transform
)


def find_nearest_on_road_point(
    network: Network,
    target_xodr: tuple,
    transform: dict,
    target_rd: tuple,
    search_radius: float = 10.0,
    step: float = 0.5
) -> tuple:
    """
    Find the nearest point in the drivable region to the target XODR coordinate.
    
    Strategy: Search in a grid around the target point and find the closest
    point that's both on-road and projects close to the target RD coordinate.
    """
    from scenic.core.vectors import Vector
    
    target_x, target_y = target_xodr
    drivable_region = network.drivableRegion
    
    if not drivable_region:
        return target_xodr
    
    best_point = None
    best_score = float('inf')
    
    # Search in a grid around target
    num_steps = int(search_radius * 2 / step)
    for i in range(-num_steps, num_steps + 1):
        for j in range(-num_steps, num_steps + 1):
            x = target_x + i * step
            y = target_y + j * step
            
            # Check if on-road
            point = Vector(x, y, 0.0)
            if not drivable_region.containsPoint(point):
                continue
            
            # Convert to RD and check distance to target
            rd_test = apply_coordinate_transform(transform, (x, y))
            rd_dist = math.sqrt(
                (rd_test[0] - target_rd[0])**2 + 
                (rd_test[1] - target_rd[1])**2
            )
            
            # Score: prioritize points close to target RD
            score = rd_dist
            
            if score < best_score:
                best_score = score
                best_point = (x, y)
    
    return best_point if best_point else target_xodr


def find_xodr_coordinate_at_route_s(
    xodr_path: Path,
    rd_path: Path,
    route_s: float,
    route_name: str = 'R2'
) -> tuple:
    """
    Find XODR coordinate for a given route s-coordinate.
    """
    print(f"Finding XODR coordinate for Route {route_name} s={route_s}")
    print("=" * 80)
    
    # Load road index (RD geometry)
    print("\n1. Loading RD road index...")
    road_index = build_rd_road_index(str(rd_path))
    
    # Determine which road we're on
    route_sequence = ROUTE_ROAD_SEQUENCES.get(route_name, [])
    transition_point = ROUTE_TRANSITION_POINTS.get(route_name, 0.0)
    
    print(f"   Route sequence: {route_sequence}")
    print(f"   Transition point: {transition_point}")
    
    # Find which road
    if route_s < transition_point:
        road_name = route_sequence[0]
        road_s = route_s
        print(f"   Road: {road_name} (first road in sequence)")
        print(f"   Road-relative s: {road_s}")
    else:
        road_name = route_sequence[1] if len(route_sequence) > 1 else route_sequence[0]
        road_s = route_s - transition_point
        print(f"   Road: {road_name} (after transition)")
        print(f"   Road-relative s (approx): {road_s}")
    
    # Get RD coordinate from road geometry
    print(f"\n2. Getting RD coordinate from road geometry...")
    roads = road_index.get('roads', {})
    road_data = roads.get(road_name)
    
    if not road_data:
        print(f"   ERROR: Road '{road_name}' not found")
        return None
    
    sec_points = road_data.get('sec_points', [[]])
    if not sec_points or not sec_points[0]:
        print(f"   ERROR: No points found")
        return None
    
    points = sec_points[0]
    
    # Find RD coordinate at road_s
    rd_coord = None
    for i in range(len(points) - 1):
        x0, y0, s0 = points[i]
        x1, y1, s1 = points[i + 1]
        if s0 <= road_s <= s1:
            u = (road_s - s0) / (s1 - s0) if s1 - s0 > 1e-6 else 0
            rd_coord = (x0 + u * (x1 - x0), y0 + u * (y1 - y0))
            break
    
    if not rd_coord:
        if points:
            rd_coord = (points[-1][0], points[-1][1])
        else:
            return None
    
    print(f"   RD coordinate: ({rd_coord[0]:.6f}, {rd_coord[1]:.6f})")
    
    # Convert to XODR using transform
    print(f"\n3. Converting RD -> XODR using transform...")
    transform_path = Path(__file__).parent.parent / "assets" / "maps" / "dSPACE" / "Laguna_Seca_transform.json"
    transform = load_transform(str(transform_path))
    
    xodr_coord = apply_inverse_coordinate_transform(transform, rd_coord)
    print(f"   XODR (from transform): ({xodr_coord[0]:.6f}, {xodr_coord[1]:.6f})")
    
    # Find nearest on-road point
    print(f"\n4. Finding nearest on-road XODR coordinate...")
    network = Network.fromOpenDrive(str(xodr_path), ref_points=50)
    
    # Check if transformed coordinate is on-road
    from scenic.core.vectors import Vector
    point = Vector(xodr_coord[0], xodr_coord[1], 0.0)
    is_on_road = network.drivableRegion.containsPoint(point)
    print(f"   Transformed coordinate is on-road: {is_on_road}")
    
    if not is_on_road:
        print(f"   Searching for nearest on-road point...")
        on_road_coord = find_nearest_on_road_point(
            network, xodr_coord, transform, rd_coord, search_radius=5.0, step=0.2
        )
        
        # Verify
        point_check = Vector(on_road_coord[0], on_road_coord[1], 0.0)
        is_on_road_check = network.drivableRegion.containsPoint(point_check)
        print(f"   Found on-road coordinate: ({on_road_coord[0]:.6f}, {on_road_coord[1]:.6f})")
        print(f"   Is on-road: {is_on_road_check}")
        
        # Check what RD it projects to
        rd_check = apply_coordinate_transform(transform, on_road_coord)
        rd_error = math.sqrt(
            (rd_check[0] - rd_coord[0])**2 + (rd_check[1] - rd_coord[1])**2
        )
        print(f"   Projects to RD: ({rd_check[0]:.6f}, {rd_check[1]:.6f})")
        print(f"   Error from target RD: {rd_error:.3f}m")
        
        return on_road_coord
    else:
        return xodr_coord


def main():
    """Main function."""
    scenic_root = Path(__file__).parent.parent
    xodr_path = scenic_root / "assets" / "maps" / "dSPACE" / "LagunaSeca.xodr"
    rd_path = scenic_root / "assets" / "maps" / "dSPACE" / "Laguna_Seca.rd"
    
    # Find coordinate for Route R2 s=175
    result = find_xodr_coordinate_at_route_s(xodr_path, rd_path, route_s=175.0, route_name='R2')
    
    if result:
        print("\n" + "=" * 80)
        print("RESULT")
        print("=" * 80)
        print(f"XODR coordinate for Route R2 s=175:")
        print(f"  ({result[0]:.6f}, {result[1]:.6f}, 0.0)")
        print("\nFor Scenic file:")
        print(f"  ego = new RacingCar at ({result[0]:.6f}, {result[1]:.6f}, 0.0)")
        print("=" * 80)
    else:
        print("\nERROR: Could not find coordinate")
        return 1
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
