# Find XODR Coordinates for (s, t) Coordinates

This folder contains scripts to empirically determine XODR coordinates for specific (s, t) coordinates on the main racing road (R2).

## Purpose

The script `find_xodr_for_st_coordinates.py` places vehicles at specified (s, t) coordinates on the R2 route (main racing road), reads their actual positions from ControlDesk, and transforms them back to XODR coordinates.

## Usage

### Prerequisites

1. **ModelDesk**: Open with Laguna Seca project and active experiment
2. **ControlDesk**: Running and connected
3. **Coordinate Transform**: `assets/maps/dSPACE/Laguna_Seca_transform.json` must exist

### Running the Script

```bash
python create_new_ttl/find_xodr_for_st_coordinates.py
```

### What It Does

1. **Loads coordinate transform** from `Laguna_Seca_transform.json`
2. **Connects to ModelDesk** and creates a test scenario
3. **Connects to ControlDesk** for reading positions
4. **Places all fellow vehicles at once**:
   - Creates all fellow vehicles at their respective (s, t) coordinates on R2 route
   - Saves and downloads the scenario
   - Resets and starts the simulation
   - Reads all positions from ControlDesk arrays (indices 0, 1, 2, etc.)
   - Transforms each RD → XODR using inverse coordinate transform
5. **Reports results** in console and saves to `st_to_xodr_results.txt`

### Test Coordinates

Currently configured to test:
- (s=200.0, t=0.0)
- (s=300.0, t=0.0)
- (s=400.0, t=0.0)

To test different coordinates, edit the `TEST_COORDINATES` list in the script.

### Output

The script outputs:
- Console summary table showing (s, t) → XODR mapping
- File: `st_to_xodr_results.txt` with the same information

Example output:
```
s          | t          | XODR X         | XODR Y         | RD X           | RD Y
--------------------------------------------------------------------------------
     200.0 |      0.000 |   123.456789 |  -234.567890 |   128.123456 |  -229.234567
     300.0 |      0.000 |   234.567890 |  -345.678901 |   239.234567 |  -340.345678
     400.0 |      0.000 |   345.678901 |  -456.789012 |   350.345678 |  -451.456789
```

## Notes

- Uses **fellow vehicles** (not ego) since we can place multiple fellows simultaneously
- All positions are tested **in batch** (all fellows placed at once, then all positions read)
- Positions are read from ControlDesk array indices 0, 1, 2, etc. (matching the order of TEST_COORDINATES)
- Positions are on **R2 route** (main racing road, not pit lane)
- The script creates a scenario copy to avoid modifying the original scenario

## Coordinate System

- **Input**: (s, t) in route-relative Frenet coordinates on R2
- **Intermediate**: RD coordinates (dSPACE Road Designer format)
- **Output**: XODR coordinates (OpenDRIVE format, used by Scenic)

The transformation chain:
```
(s, t) → ModelDesk → ControlDesk RD → Inverse Transform → XODR
```

