# TTL loading

TTL CSV files are loaded without any offset; they are assumed to be in map (XODR) coordinates, matching Scenic vehicle positions.

## Key functions

- **`get_ttl_config(scene_params)`** – Returns `(ttl_folder, ttl_index, ttl_file_name_or_None)` from scene params.
- **`load_ttl_region(ttl_folder, ttl_index, ttl_file_name=None)`** – Loads TTL CSV and returns `(PolylineRegion, waypoints)`.
- **`attach_ttl(sim, obj, vehicle_type="vehicle")`** – Loads TTL from object or scene params and attaches region/waypoints to the object.

## Usage

Set `param ttlFolder` and optionally `param ttlFileName` or per-vehicle `ttlFolder` / `ttlFileName`. Default folder is `assets/ttls/LS_ENU_TTL_CSV`.
