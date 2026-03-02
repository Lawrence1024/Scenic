#!/usr/bin/env python3
"""
Debug why waypoint 3422 is being found but CTE calculation might be wrong.
"""

import sys
import csv
from pathlib import Path

# Add Scenic src to path
scenic_path = Path(__file__).parent.parent / "src"
if scenic_path.exists():
    sys.path.insert(0, str(scenic_path))

from scenic.simulators.dspace.ttl.loader import load_ttl_region
import os

def main():
    """Debug waypoint loading and coordinates."""
    scenic_root = Path(__file__).parent.parent
    ttl_folder = scenic_root / "assets" / "ttls" / "LS_ENU_TTL_CSV"
    
    # Load waypoints the same way the behavior does
    region, pts = load_ttl_region(str(ttl_folder), 17, 0.0, 0.0)
    
    if not pts:
        print("ERROR: Could not load waypoints")
        return 1
    
    print(f"Loaded {len(pts)} waypoints")
    print(f"First waypoint (index 0): ({pts[0][0]:.6f}, {pts[0][1]:.6f})")
    print(f"Last waypoint (index {len(pts)-1}): ({pts[-1][0]:.6f}, {pts[-1][1]:.6f})")
    
    # Vehicle position from log
    vehicle_pos = (70.23, 109.16)
    
    # Find waypoint 3422
    if len(pts) > 3422:
        wp3422 = pts[3422]
        print(f"\nWaypoint 3422: ({wp3422[0]:.6f}, {wp3422[1]:.6f})")
        
        # Calculate distance
        dx = vehicle_pos[0] - wp3422[0]
        dy = vehicle_pos[1] - wp3422[1]
        dist = (dx*dx + dy*dy)**0.5
        print(f"Distance from vehicle to waypoint 3422: {dist:.2f}m")
        
        # Check nearby waypoints
        print(f"\nNearby waypoints:")
        for i in range(max(0, 3420), min(len(pts), 3425)):
            wp = pts[i]
            dx = vehicle_pos[0] - wp[0]
            dy = vehicle_pos[1] - wp[1]
            dist = (dx*dx + dy*dy)**0.5
            print(f"  Index {i}: ({wp[0]:.6f}, {wp[1]:.6f}), distance: {dist:.2f}m")
        
        # Check lookahead calculation
        print(f"\nLookahead calculation (20m from waypoint 3422):")
        lookahead = 20.0
        rem = lookahead
        j = 3422
        found_target = False
        
        while rem > 0.0 and j < len(pts) - 1:
            x0, y0 = float(pts[j][0]), float(pts[j][1])
            x1, y1 = float(pts[j+1][0]), float(pts[j+1][1])
            seg_dx = x1 - x0
            seg_dy = y1 - y0
            seg_len = (seg_dx*seg_dx + seg_dy*seg_dy) ** 0.5
            
            if seg_len <= 1e-6:
                j += 1
                continue
            
            if rem <= seg_len:
                u = rem / seg_len
                tgt_x = x0 + u * seg_dx
                tgt_y = y0 + u * seg_dy
                print(f"  Target point: ({tgt_x:.2f}, {tgt_y:.2f})")
                print(f"  Segment: ({x0:.2f}, {y0:.2f}) -> ({x1:.2f}, {y1:.2f}), length: {seg_len:.2f}m")
                
                # Calculate CTE
                nx = -seg_dy / seg_len
                ny = seg_dx / seg_len
                cte = (vehicle_pos[0] - tgt_x)*nx + (vehicle_pos[1] - tgt_y)*ny
                print(f"  Normal: ({nx:.3f}, {ny:.3f})")
                print(f"  CTE: {cte:.3f}m")
                found_target = True
                break
            else:
                rem -= seg_len
                j += 1
        
        if not found_target:
            print("  ERROR: Could not find target point within lookahead distance")
    else:
        print(f"ERROR: Waypoint list has only {len(pts)} points, cannot access index 3422")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())

