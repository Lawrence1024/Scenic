# TTL tooling (create_new_ttl)

Scripts for generating and validating **target trajectory lines (TTLs)** for the Laguna Seca racing domain. Located at `src/scenic/simulators/dspace/create_new_ttl`. **Run all commands from the Scenic repository root.**

## Assets

TTLs used by racing examples live in **`assets/ttls/LS_ENU_TTL_CSV`**:

- **ttl_main_road.csv** – Main road closed loop (Andretti Hairpin + Corkscrew).
- **ttl_pitlane.csv** – Pit lane closed loop; built by `combine_and_compare_ttl.py` (writes into that folder).

See `assets/ttls/LS_ENU_TTL_CSV/README.md` for format and usage.

## Background

Everything is in **XODR** coordinates: TTLs, placement, and MPC tracking. The sim converts XODR ↔ RD for dSPACE; boundary plots (e.g. from `visualize_ttl_boundaries.py`) show TTL vs XODR edges. **CTE** is distance to the TTL (racing line), not to the lane center.

## Scripts

| Script | Purpose |
|--------|---------|
| **combine_and_compare_ttl.py** | Build pitlane→main→pitlane and closed pitlane loop; writes `ttl_pitlane.csv` to assets. |
| **visualize_combined_ttl.py** | Three-panel plot (pitlane / main / combined). Use `--overlap` to overlay main + pitlane. |
| **close_ttl_loop.py** | Close an open TTL CSV into a loop (interpolate last→first). |
| **generate_racing_line.py** | Curvature-based racing line from centerline; outputs open line then close with `close_ttl_loop.py`. |
| **find_xodr_for_st_coordinates.py** | Full-route dSPACE measurement sweep for `R2` and `R1`; batches up to 30 fellows, checkpoints progress, uses measurement-only segment setup with longitudinal `Velocity=Constant(0)` and lateral `Continue`, and writes outputs under `create_new_ttl/measurements/` including `(s, t)`, RD `(x, y, z)`, GNSS `(lon, lat, heading)`, and XODR `(x, y, z)`. |
| **measurements_to_centerlines.py** | Export R2 (main) and R1 (pit) centerlines from `measurements/route_st_to_xodr_measurements.csv` as TTL CSVs (t=0, XODR coords). Use `--copy-to-assets` to replace `assets/ttls/LS_ENU_TTL_CSV/ttl_main_road.csv` and `ttl_pitlane.csv`. |
| **build_main_track_xodr.py** | Build main-track-only XODR from TTL centerline → `LagunaSeca_MainTrack_FromTTL.xodr`. |
| **analyze_ttl_quality.py** | Per-point distance to boundaries and curvature (e.g. segment 43). |
| **verify_ttl_against_xodr.py** | Plot TTL vs XODR centerline/boundaries. |
| **visualize_ttl_boundaries.py** | TTL vs track boundaries (XODR). |
| **compare_racing_lines.py** | Compare two TTL CSVs (stats + plot). |
| **interactive_map_visualizer.py** | Interactive track + TTL view (zoom/pan). |
| **generate_pitlane_ttl.py** | Alternative pitlane TTL (replace main-loop arc with pit path). |

## Usage (from repo root)

**Pitlane loop and overlay plot:**
```bash
python src/scenic/simulators/dspace/create_new_ttl/combine_and_compare_ttl.py
python src/scenic/simulators/dspace/create_new_ttl/visualize_combined_ttl.py --overlap
```

**Racing line (open → closed):**
```bash
python src/scenic/simulators/dspace/create_new_ttl/generate_racing_line.py
python src/scenic/simulators/dspace/create_new_ttl/close_ttl_loop.py --csv assets/ttls/LS_ENU_TTL_CSV/ttl_racing_line_xodr.csv -o assets/ttls/LS_ENU_TTL_CSV/ttl_main_road.csv
```

**Main-track XODR:**
```bash
python src/scenic/simulators/dspace/create_new_ttl/build_main_track_xodr.py
```

**(s,t) → XODR full sweep** (needs ModelDesk + ControlDesk):
```bash
python src/scenic/simulators/dspace/create_new_ttl/find_xodr_for_st_coordinates.py
```

**Centerlines from measurements** (after running the sweep above):
```bash
python src/scenic/simulators/dspace/create_new_ttl/measurements_to_centerlines.py
# Optional: replace assets TTLs
python src/scenic/simulators/dspace/create_new_ttl/measurements_to_centerlines.py --copy-to-assets
# Then generate XODR (racing domain): python -m scenic.domains.racing.XODR_generation.build_ttl_xodr ...
```

Defaults:
- `R2` and `R1`
- `s=0..3500`, `step=1`, `t in {-4, 0, +4}`
- batch size `30`
- resumable via `create_new_ttl/measurements/route_st_to_xodr_checkpoint.json`

Scripts that produce figures support `--save <path>`; outputs are not committed. The only image kept in the repo is **ttl_two_loops_overlay.png** (reference for main + pitlane overlay).
