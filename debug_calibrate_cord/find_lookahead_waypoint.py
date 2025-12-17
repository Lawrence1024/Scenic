#!/usr/bin/env python3
"""Find the lookahead waypoint position for fellow car placement."""

import csv
from pathlib import Path

def main():
    scenic_root = Path(__file__).parent.parent
    ttl_file = scenic_root / "assets" / "ttls" / "LS_ENU_TTL_CSV" / "transformed" / "ttl_17.csv"
    
    with open(ttl_file, 'r') as f:
        reader = csv.reader(f)
        next(reader)  # Skip header
        wps = [(float(row[0]), float(row[1])) for row in reader]
    
    # Start from waypoint 3422 (nearest to ego)
    nearest_idx = 3422
    lookahead = 20.0
    rem = lookahead
    j = nearest_idx
    found_target = False
    tgt_x = None
    tgt_y = None
    
    # Walk forward along polyline for lookahead target
    while rem > 0.0 and j < len(wps) - 1:
        x0, y0 = wps[j][0], wps[j][1]
        x1, y1 = wps[j+1][0], wps[j+1][1]
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
            found_target = True
            print(f"Lookahead target (20m from wp{nearest_idx}): ({tgt_x:.6f}, {tgt_y:.6f})")
            print(f"Segment: wp[{j}]=({x0:.6f}, {y0:.6f}) -> wp[{j+1}]=({x1:.6f}, {y1:.6f})")
            break
        else:
            rem -= seg_len
            j += 1
    
    if not found_target:
        print("ERROR: Could not find lookahead target")
        return 1
    
    return 0

if __name__ == "__main__":
    exit(main())

