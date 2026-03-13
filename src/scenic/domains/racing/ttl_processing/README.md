# TTL and calibration processing

Scripts for fitting and validating coordinate transforms using route measurement data.

## GNSS ↔ RD calibration

**`fit_gnss_rd_calibration.py`** fits the affine transform (GNSS lon/lat ↔ dSPACE RD x/y) from `route_st_to_xodr_measurements.csv` (created by `find_xodr_for_st_coordinates.py`) and writes `gps_rd_calibration.json` under `simulators/dspace/geometry/`.

- **Input:** `src/scenic/simulators/dspace/create_new_ttl/measurements/route_st_to_xodr_measurements.csv` (columns: `gps_longitude_deg`, `gps_latitude_deg`, `rd_x_m`, `rd_y_m`).
- **Output:** `src/scenic/simulators/dspace/geometry/gps_rd_calibration.json` (used for GNSS readback when targeting RD).

Run from repo root:

```bash
python -m scenic.domains.racing.ttl_processing.fit_gnss_rd_calibration
```

Options:

- `--csv PATH` — custom measurements CSV
- `-o PATH` — custom output JSON path
- `--max-points N` — subsample to N points (default: use all)
- `--validate-only` — load existing calibration and print residuals (no fit, no write)
