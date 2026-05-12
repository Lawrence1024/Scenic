**Reference calibration data — checked in deliberately.** The CSV and
checkpoint JSON in this directory are the canonical route-measurement
sweep used to derive `assets/ttls/LS_ENU_TTL_CSV/ttl_main_road.csv` and
`ttl_pitlane.csv`. They are required to reproduce TTL generation from
scratch without re-running the (expensive, ModelDesk-dependent)
`find_xodr_for_st_coordinates.py` sweep. Total ~10 MB.

---

Generated route-measurement outputs are written here by `find_xodr_for_st_coordinates.py`.

Expected files:
- `route_st_to_xodr_measurements.csv`
- `route_st_to_xodr_summary.txt`
- `route_st_to_xodr_checkpoint.json`

This directory is the canonical home for dSPACE route calibration sweeps.

## Export centerlines (R2 → main, R1 → pit)

From the measurements CSV you can export TTL centerline CSVs (x,y,z in RD coordinates) for replacing `ttl_main_road.csv` and `ttl_pitlane.csv`:

```bash
# From repo root
python src/scenic/simulators/dspace/create_new_ttl/measurements_to_centerlines.py
```

This reads `route_st_to_xodr_measurements.csv`, extracts centerline points (t=0) for R2 (main) and R1 (pit) using RD coordinates, and writes:

- `create_new_ttl/ttl_main_road_from_measurements.csv`
- `create_new_ttl/ttl_pitlane_from_measurements.csv`

To copy these into the assets folder as the canonical TTLs:

```bash
python src/scenic/simulators/dspace/create_new_ttl/measurements_to_centerlines.py --copy-to-assets
```

Then generate an OpenDRIVE from the new centerlines (with predefined widths):

```bash
python -m scenic.domains.racing.XODR_generation.build_ttl_xodr \
  --main src/scenic/simulators/dspace/create_new_ttl/ttl_main_road_from_measurements.csv \
  --pit src/scenic/simulators/dspace/create_new_ttl/ttl_pitlane_from_measurements.csv \
  -o src/scenic/domains/racing/XODR_generation/generated/track_from_ttl.xodr
```

Or after `--copy-to-assets`, use the default paths (assets TTLs) for the XODR generator.
