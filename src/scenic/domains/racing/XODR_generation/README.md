# XODR Generation

This folder contains scripts and utilities to generate OpenDRIVE (`.xodr`) files for use with the Scenic racing domain.

## What is generated

- **Three roads** built from TTL centerline CSVs:
  - **Road 1 (MainTrack_A):** Arc from pit entry to pit exit along the main centerline (7 m each side).
  - **Road 2 (PitTrack):** Pit lane centerline, **trimmed** where it overlaps main (within 5 m) so main track width dominates (3.5 m each side).
  - **Road 3 (MainTrack_B):** Arc from pit exit to pit entry along the main centerline (7 m each side) — includes Andretti Hairpin, Corkscrew, etc.
- **Full main loop** = Road 1 + Road 3. In overlap regions (e.g. Corkscrew) pit points near main are removed so only main track width is used.
- **Fixed lane widths:** 7 m each side for main, 3.5 m each side for pit (configurable).
- **Connected topology:** Predecessor/successor links (no junction elements). Cycle: MainTrack_A → PitTrack → MainTrack_B → MainTrack_A.

The output XODR can be used as `param map = localPath('...')` in Scenic racing scenarios so that the drivable region matches your TTL-based geometry and avoids “does not fit in container” issues from mismatched OpenDRIVE.

## Centerline and lanes

Each generated road has **one reference line** (OpenDRIVE planView), which is the TTL centerline polyline. There is **one lane section** for the whole road:

- **Center** (lane id=0, type=none): the reference line itself.
- **Left** (lane id=1) and **Right** (lane id=-1): driving lanes with constant width (7 m or 3.5 m each side).

So there is **no "multiple lanes" in the sense of multiple centerlines**: there is a single reference line per road, and (s, t) projection / placement is done against that **reference line** (not against a specific lane's center). For maps with true multi-lane roads (e.g. several lanes each side), you would want lane-specific centerlines; this generator intentionally uses one centerline per road and symmetric left/right width, so "centerline" here is the road reference line to which all lanes are attached.

## Requirements

- TTL centerline CSVs with columns `x,y,z` (or at least `x,y`). Typical inputs:
  - `ttl_main_road.csv` — main circuit centerline (closed loop).
  - `ttl_pitlane.csv` — pit lane centerline (start near main at pit exit, end near main at pit entry).

## Usage

Run from the **Scenic repository root**:

```bash
# Use default paths (assets/ttls/LS_ENU_TTL_CSV/ttl_main_road.csv, ttl_pitlane.csv)
# Output: src/scenic/domains/racing/XODR_generation/generated/track_from_ttl.xodr
python -m scenic.domains.racing.XODR_generation.build_ttl_xodr

# Custom inputs and output
python -m scenic.domains.racing.XODR_generation.build_ttl_xodr \
  --main path/to/ttl_main_road.csv \
  --pit path/to/ttl_pitlane.csv \
  --output path/to/my_track.xodr

# Custom lane widths (meters each side)
python -m scenic.domains.racing.XODR_generation.build_ttl_xodr \
  --main-width 7 --pit-width 3.5 -o generated/track_from_ttl.xodr
```

Or run the script directly:

```bash
python src/scenic/domains/racing/XODR_generation/build_ttl_xodr.py --help
```

## Output location

By default, the generated `.xodr` file is written to:

- **`XODR_generation/generated/track_from_ttl.xodr`** (under this package).

Use `--output` / `-o` to write elsewhere.

## Programmatic use

```python
from pathlib import Path
from scenic.domains.racing.XODR_generation import build_connected_ttl_xodr

out_path = build_connected_ttl_xodr(
    main_ttl_path=Path("assets/ttls/LS_ENU_TTL_CSV/ttl_main_road.csv"),
    pit_ttl_path=Path("assets/ttls/LS_ENU_TTL_CSV/ttl_pitlane.csv"),
    output_path=Path("generated/track_from_ttl.xodr"),
    main_width=7.0,
    pit_width=3.5,
)
```

## How connection points are chosen

- **Pit exit:** Index on the main centerline closest to the **first point** of the pit TTL.
- **Pit entry:** Index on the main centerline closest to the **last point** of the pit TTL.

The main road in the XODR is the arc from pit entry to pit exit along the main polyline, so the two roads connect tightly at both ends.
