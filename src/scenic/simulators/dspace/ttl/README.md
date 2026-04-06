# TTL loading

TTL CSV files are loaded without any offset; they are assumed to be in map (XODR) coordinates, matching Scenic vehicle positions.

## Key functions

- **`get_ttl_config(scene_params)`** – Returns `(ttl_folder, ttl_file_name)` from scene params. Default folder is `assets/ttls/LS_ENU_TTL_CSV`; default file is `ttl_main_road.csv`.
- **`load_ttl_region(ttl_folder, ttl_file_name)`** – Loads the named CSV under `ttl_folder` and returns `(PolylineRegion, waypoints)`.
- **`attach_ttl(sim, obj, vehicle_type="vehicle")`** – Loads TTL from object or scene params and attaches region/waypoints to the object.

## Usage

Set `param ttlFolder` and optionally `param ttlFileName` or per-vehicle `ttlFolder` / `ttlFileName`. Filenames are basenames only (for example `ttl_optimal_xodr.csv`); there is no indexed `ttl_N.csv` convention.
