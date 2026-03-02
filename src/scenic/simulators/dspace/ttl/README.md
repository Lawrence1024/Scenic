# TTL coordinate system integration

The TTL loading code automatically detects and handles coordinate system transformations based on the folder containing the TTL files.

## Automatic detection

- **`transformed` folder:** Files are in XODR coordinates. Offset set to `(0, 0)`; no transformation. Waypoints can be used directly with Scenic vehicle positions.
- **Other folders** (e.g. `usable`, `raw`): Files in ENU/RD. Default offset `(-53.6, -15.7)` applied during loading.

## Key functions

- **`get_ttl_config(scene_params)`** – Detects `transformed` in path; sets offset to (0,0) for transformed, (-53.6,-15.7) otherwise; warns if explicit offset is set for transformed.
- **`load_ttl_region(...)`** – Logs coordinate system and offset used.
- **`attach_ttl(sim, obj, vehicle_type="vehicle")`** – Auto-detects offset from folder path; respects explicit overrides; default folder `transformed`.

## Usage

**Default (recommended):** Use `transformed` folder; offset (0, 0) is automatic.

**Explicit override:** Set `param ttlFolder`, `param ttlDX`, `param ttlDY` as needed.

**Non-transformed:** Use path to `usable` or `raw`; offset is auto-set to (-53.6, -15.7).

## Alignment

Vehicle positions (Scenic) and TTL waypoints from `transformed` both use XODR coordinates, so waypoint following works without manual offset.
