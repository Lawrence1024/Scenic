# TTL Tooling (create_new_ttl)

This folder lives under the dSPACE simulator: **`src/scenic/simulators/dspace/create_new_ttl`**. It contains scripts for generating and validating target trajectory lines (TTLs) for the Laguna Seca racing domain. Run all commands **from the Scenic repository root**.

---

## Find XODR Coordinates for (s, t) Coordinates

The script `find_xodr_for_st_coordinates.py` empirically determines XODR coordinates for specific (s, t) coordinates on the main racing road (R2).

## Purpose

The script `find_xodr_for_st_coordinates.py` places vehicles at specified (s, t) coordinates on the R2 route (main racing road), reads their actual positions from ControlDesk, and transforms them back to XODR coordinates.

## Usage

### Prerequisites

1. **ModelDesk**: Open with Laguna Seca project and active experiment
2. **ControlDesk**: Running and connected
3. **Coordinate Transform**: `assets/maps/dSPACE/Laguna_Seca_transform.json` must exist

### Running the Script

```bash
python src/scenic/simulators/dspace/create_new_ttl/find_xodr_for_st_coordinates.py
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
- File: `st_to_xodr_results.txt` in this folder with the same information

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

## Interactive Map Visualizer

The script `interactive_map_visualizer.py` draws the track and TTLs in one view so you can compare them and zoom/pan.

### Usage

```bash
python src/scenic/simulators/dspace/create_new_ttl/interactive_map_visualizer.py
```

### What It Draws (in order)

1. **Track boundaries** from `assets/maps/dSPACE/LagunaSeca.xodr` (left/right edges; main track + pit in gray/black)
2. **Centerline** from `assets/ttls/LS_ENU_TTL_CSV/ttl_fellow_test_xodr_all.csv` (blue)
3. **Aligned line** from this folder’s `temp_aligned_to_centerline.csv` (orange)

Use the matplotlib window toolbar to **zoom** (magnifying glass), **pan** (hand), **save**, or **reset view** (home).

## Main-Track-Only OpenDRIVE (from TTL)

The script `build_main_track_xodr.py` creates a new OpenDRIVE file that has a single road for the main track only (no pit, no junctions). The reference line is the TTL centerline (`ttl_fellow_test_xodr_all.csv`), and left/right lane widths are taken from the source XODR at the closest reference-line position for each TTL point.

### Usage

```bash
python src/scenic/simulators/dspace/create_new_ttl/build_main_track_xodr.py
python src/scenic/simulators/dspace/create_new_ttl/build_main_track_xodr.py --output path/to/out.xodr --xodr path/to/LagunaSeca.xodr --ttl path/to/ttl.csv
```

### Output

- Default output: `assets/maps/dSPACE/LagunaSeca_MainTrack_FromTTL.xodr`
- One road, one lane section, left and right driving lanes with widths sampled from the source XODR (main-track roads: The Corkscrew1, Andretti Hairpin1_3). If the source has only one-sided lanes, total width is split equally left/right.

## Racing Line Generator

The script `generate_racing_line.py` creates a viable racing line from the centerline using curvature-based optimization.

### Usage

```bash
python src/scenic/simulators/dspace/create_new_ttl/generate_racing_line.py
```

### What It Does

1. **Loads centerline** from `assets/ttls/LS_ENU_TTL_CSV/ttl_fellow_test_xodr_all.csv`
2. **Computes curvature** at each point along the path
3. **Generates racing line** by offsetting from centerline based on:
   - Curvature magnitude and direction
   - Lookahead anticipation (50m ahead)
   - Track width constraints (max 10m deviation)
4. **Smooths the racing line** using moving average
5. **Caps curvature** to the vehicle kinematic limit (κ_max ≈ 0.097 1/m) so the path is followable by the MPC
6. **Outputs** to this folder’s `ttl_racing_line_xodr.csv` and to `assets/ttls/LS_ENU_TTL_CSV/ttl_racing_line_xodr.csv` (same format: x,y,z). For lap racing, close the loop with `close_ttl_loop.py`; the canonical main-road TTL in assets is **ttl_main_road.csv** (closed loop).

### Racing Line Strategy

- **Left turns** (positive curvature): Offset to right (outside) before turn, cut inside at apex
- **Right turns** (negative curvature): Offset to left (outside) before turn, cut inside at apex
- **Straights**: Stay near centerline
- **Maximum offset**: 75% of track width (7.5m) to leave safety margin

### Configuration

Key parameters (can be modified in the script):
- `MAX_TRACK_WIDTH = 10.0` meters
- `CURVATURE_THRESHOLD = 0.005` 1/m (R = 200m)
- `LOOKAHEAD_DISTANCE = 50.0` meters
- `SMOOTHING_WINDOW = 5` points

### Output

The script generates:
- CSV file: `ttl_racing_line_xodr.csv` with x,y,z columns (open line). After closing with `close_ttl_loop.py`, use **ttl_main_road.csv** in `assets/ttls/LS_ENU_TTL_CSV` for lap racing.
- Statistics showing deviation from centerline:
  - Mean, median, max deviation
  - Percentage of points exceeding thresholds

Example output:
```
Mean deviation: 1.43 m
Median deviation: 0.18 m
Max deviation: 7.50 m
95th percentile: 6.18 m
Points with deviation > 10m: 0 (0.0%)
Points with deviation > 5m: 318 (8.9%)
```

### Main road and pitlane TTLs (assets)

The folder `assets/ttls/LS_ENU_TTL_CSV` holds the TTLs used by racing examples:

- **ttl_main_road.csv** – Main road closed loop (Andretti Hairpin + Corkscrew). Single file; no separate open or `_closed` variant.
- **ttl_pitlane.csv** – Pit lane closed loop (Pit Lane + Corkscrew). Generated by `combine_and_compare_ttl.py` in this folder; written into that assets folder. Has a small gap (~3 m) between last and first point (acceptable for racing).

See `assets/ttls/LS_ENU_TTL_CSV/README.md` for usage and legacy names.

### Notes

- The racing line respects track width constraints (all points within 10m of centerline)
- Uses curvature-based optimization similar to CiMPCC approach
- Smooth transitions between straight and corner sections
- Output format matches the centerline CSV format for easy integration

## Other scripts (run from repo root)

| Script | Purpose |
|--------|---------|
| `close_ttl_loop.py` | Close an open TTL CSV into a loop (interpolate last→first). |
| `combine_and_compare_ttl.py` | Build pitlane→main→pitlane combined TTL and closed pitlane loop; writes `ttl_pitlane.csv` to assets. |
| `visualize_combined_ttl.py` | Three-panel plot: pitlane only, main only, combined. Use `--overlap` to overlay main + pitlane. |
| `analyze_ttl_quality.py` | Per-point distance to boundaries and curvature (e.g. for segment 43). |
| `verify_ttl_against_xodr.py` | Plot TTL vs XODR centerline/boundaries. |
| `visualize_ttl_boundaries.py` | TTL vs track boundaries. |
| `visualize_ttl_wrap.py` | Check TTL end→start wrap (gap). |
| `compare_racing_lines.py` | Compare two TTL CSVs. |
| `compare_and_zoom.py` | Compare centerline vs another CSV with optional zoom. |
| `generate_pitlane_ttl.py` | Alternative pitlane TTL (replace main-loop arc with pit path). |

Example:
```bash
python src/scenic/simulators/dspace/create_new_ttl/close_ttl_loop.py --csv assets/ttls/LS_ENU_TTL_CSV/ttl_racing_line_xodr.csv -o assets/ttls/LS_ENU_TTL_CSV/ttl_main_road.csv
python src/scenic/simulators/dspace/create_new_ttl/combine_and_compare_ttl.py
python src/scenic/simulators/dspace/create_new_ttl/visualize_combined_ttl.py --overlap
```
