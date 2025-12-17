#!/usr/bin/env python3
"""
Find an XODR coordinate that has enough buffer for a vehicle's bounding box.

The issue: Even if a point is on-road, the vehicle's bounding box (width x length)
might extend outside the drivable region, causing InvalidScenarioError.
"""

import sys
import math
from pathlib import Path

# Add Scenic src to path
scenic_path = Path(__file__).parent.parent / "src"
if scenic_path.exists():
    sys.path.insert(0, str(scenic_path))

from scenic.domains.driving.roads import Network
from scenic.core.vectors import Vector
from scenic.simulators.dspace.geometry.rd_parser import build_rd_road_index
from scenic.simulators.dspace.geometry.route_projection import ROUTE_ROAD_SEQUENCES, ROUTE_TRANSITION_POINTS
from scenic.simulators.dspace.geometry.coordinate_transform import (
    load_transform,
    apply_inverse_coordinate_transform,
    apply_coordinate_transform
)


def find_coordinate_with_buffer(
    network: Network,
    target_xodr: tuple,
    transform: dict,
    target_rd: tuple,
    vehicle_width: float = 2.0,
    vehicle_length: float = 4.5,
    search_radius: float = 10.0,
    step: float = 0.1
) -> tuple:
    """
    Find a coordinate where the entire vehicle bounding box fits in the drivable region.
    
    We check multiple points around the vehicle's bounding box to ensure it all fits.
    """
    target_x, target_y = target_xodr
    drivable_region = network.drivableRegion
    
    if not drivable_region:
        return target_xodr
    
    best_point = None
    best_score = float('inf')
    
    # Half dimensions for checking corners
    half_width = vehicle_width / 2
    half_length = vehicle_length / 2
    
    # Search in a grid around target
    num_steps = int(search_radius * 2 / step)
    for i in range(-num_steps, num_steps + 1):
        for j in range(-num_steps, num_steps + 1):
            x = target_x + i * step
            y = target_y + j * step
            center = Vector(x, y, 0.0)
            
            # Check if center is on-road
            if not drivable_region.containsPoint(center):
                continue
            
            # Check if entire bounding box fits
            # Check 4 corners of the bounding box (assuming vehicle is axis-aligned for now)
            # In practice, we should check with orientation, but this is a conservative check
            corners = [
                Vector(x - half_length, y - half_width, 0.0),  # rear-left
                Vector(x + half_length, y - half_width, 0.0),  # front-left
                Vector(x - half_length, y + half_width, 0.0),  # rear-right
                Vector(x + half_length, y + half_width, 0.0),  # front-right
            ]
            
            all_corners_on_road = True
            for corner in corners:
                if not drivable_region.containsPoint(corner):
                    all_corners_on_road = False
                    break
            
            if not all_corners_on_road:
                continue
            
            # Also check a few points along the edges
            edge_points = [
                Vector(x - half_length, y, 0.0),  # rear center
                Vector(x + half_length, y, 0.0),  # front center
                Vector(x, y - half_width, 0.0),   # left center
                Vector(x, y + half_width, 0.0),   # right center
            ]
            
            for edge_point in edge_points:
                if not drivable_region.containsPoint(edge_point):
                    all_corners_on_road = False
                    break
            
            if not all_corners_on_road:
                continue
            
            # This point works! Now score it by distance to target RD
            rd_test = apply_coordinate_transform(transform, (x, y))
            rd_dist = math.sqrt(
                (rd_test[0] - target_rd[0])**2 + 
                (rd_test[1] - target_rd[1])**2
            )
            
            if rd_dist < best_score:
                best_score = rd_dist
                best_point = (x, y)
    
    return best_point if best_point else target_xodr


def main():
    """Main function."""
    scenic_root = Path(__file__).parent.parent
    xodr_path = scenic_root / "assets" / "maps" / "dSPACE" / "LagunaSeca.xodr"
    rd_path = scenic_root / "assets" / "maps" / "dSPACE" / "Laguna_Seca.rd"
    
    # Target: Route R2 s=175
    route_s = 175.0
    route_name = 'R2'
    
    print(f"Finding XODR coordinate for Route {route_name} s={route_s}")
    print("With buffer for vehicle bounding box (2.0m x 4.5m)")
    print("=" * 80)
    
    # Load road index
    road_index = build_rd_road_index(str(rd_path))
    route_sequence = ROUTE_ROAD_SEQUENCES.get(route_name, [])
    transition_point = ROUTE_TRANSITION_POINTS.get(route_name, 0.0)
    
    # Get RD coordinate
    if route_s < transition_point:
        road_name = route_sequence[0]
        road_s = route_s
    else:
        road_name = route_sequence[1] if len(route_sequence) > 1 else route_sequence[0]
        road_s = route_s - transition_point
    
    roads = road_index.get('roads', {})
    road_data = roads.get(road_name)
    points = road_data.get('sec_points', [[]])[0]
    
    # Find RD coordinate
    rd_coord = None
    for i in range(len(points) - 1):
        x0, y0, s0 = points[i]
        x1, y1, s1 = points[i + 1]
        if s0 <= road_s <= s1:
            u = (road_s - s0) / (s1 - s0) if s1 - s0 > 1e-6 else 0
            rd_coord = (x0 + u * (x1 - x0), y0 + u * (y1 - y0))
            break
    
    if not rd_coord and points:
        rd_coord = (points[-1][0], points[-1][1])
    
    print(f"Target RD coordinate: ({rd_coord[0]:.6f}, {rd_coord[1]:.6f})")
    
    # Convert to XODR
    transform_path = scenic_root / "assets" / "maps" / "dSPACE" / "Laguna_Seca_transform.json"
    transform = load_transform(str(transform_path))
    xodr_coord = apply_inverse_coordinate_transform(transform, rd_coord)
    print(f"Transformed XODR coordinate: ({xodr_coord[0]:.6f}, {xodr_coord[1]:.6f})")
    
    # Load network and find coordinate with buffer
    print("\nSearching for coordinate with vehicle bounding box buffer...")
    network = Network.fromOpenDrive(str(xodr_path), ref_points=50)
    
    result = find_coordinate_with_buffer(
        network, xodr_coord, transform, rd_coord,
        vehicle_width=2.0, vehicle_length=4.5,
        search_radius=5.0, step=0.1
    )
    
    if result:
        # Verify
        center = Vector(result[0], result[1], 0.0)
        is_on_road = network.drivableRegion.containsPoint(center)
        
        # Check bounding box
        half_width = 1.0
        half_length = 2.25
        corners = [
            Vector(result[0] - half_length, result[1] - half_width, 0.0),
            Vector(result[0] + half_length, result[1] - half_width, 0.0),
            Vector(result[0] - half_length, result[1] + half_width, 0.0),
            Vector(result[0] + half_length, result[1] + half_width, 0.0),
        ]
        all_fit = all(network.drivableRegion.containsPoint(c) for c in corners)
        
        rd_check = apply_coordinate_transform(transform, result)
        rd_error = math.sqrt(
            (rd_check[0] - rd_coord[0])**2 + (rd_check[1] - rd_coord[1])**2
        )
        
        print("\n" + "=" * 80)
        print("RESULT")
        print("=" * 80)
        print(f"XODR coordinate: ({result[0]:.6f}, {result[1]:.6f}, 0.0)")
        print(f"Center on-road: {is_on_road}")
        print(f"Bounding box fits: {all_fit}")
        print(f"Projects to RD: ({rd_check[0]:.6f}, {rd_check[1]:.6f})")
        print(f"Error from target RD: {rd_error:.3f}m")
        print("\nFor Scenic file:")
        print(f"  ego = new RacingCar at ({result[0]:.6f}, {result[1]:.6f}, 0.0)")
        print("=" * 80)
    else:
        print("\nERROR: Could not find suitable coordinate")
        return 1
    
    return 0


if __name__ == "__main__":
    sys.exit(main())

