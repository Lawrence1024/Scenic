# GNSS (WGS84) Readback from dSPACE

This folder contains scripts and documentation for reading **GNSS coordinates in WGS84** (World Geodetic System 1984) from dSPACE ControlDesk. The variables provide longitude, latitude, and heading in degrees and are useful for logging, external tools, or comparing with other coordinate systems (e.g. RD, XODR).

## Coordinate system: WGS84

- **Longitude**: degrees (e.g. -121.75°)
- **Latitude**: degrees (e.g. 36.58°)
- **Heading**: degrees (vehicle orientation, typically 0–360 or -180–180 depending on model)

Values are from the model’s **GPS_CALC** block and represent the same pose as the vehicle state, expressed in WGS84 instead of the simulation’s local frame (RD) or map frame (XODR).

## Variable paths (ControlDesk)

Paths use the `Platform()://ASM_Traffic/Model Root/...` namespace. Spelling matches the document “Response (WGS84)” (with `Model Root` normalized for consistency with other dSPACE scripts in this repo).

### Ego vehicle

Single values (no array index). Source: **Environment / Road / PlantModel** (plant model of the road environment).

| Quantity   | Path |
|-----------|------|
| Longitude | `Platform()://ASM_Traffic/Model Root/Environment/Road/PlantModel/GPS_POSITION/GPS_CALC/Longitude_deg` |
| Latitude  | `Platform()://ASM_Traffic/Model Root/Environment/Road/PlantModel/GPS_POSITION/GPS_CALC/Latitude_deg` |
| Heading   | `Platform()://ASM_Traffic/Model Root/Environment/Road/PlantModel/GPS_POSITION/GPS_CALC/Heading_deg` |

### Fellow vehicles

Array values per vehicle. Source: **VesiInterface / Vehicle Sensors / ground_truth** (ground-truth GNSS from vehicle sensors).

| Quantity   | Path (fellow index `i`) |
|-----------|--------------------------|
| Longitude | `Platform()://ASM_Traffic/Model Root/VesiInterface/Vehicle Sensors/ground_truth/GPS_POSITION/GPS_CALC/Longitude_deg[i]` |
| Latitude  | `Platform()://ASM_Traffic/Model Root/VesiInterface/Vehicle Sensors/ground_truth/GPS_POSITION/GPS_CALC/Latitude_deg[i]` |
| Heading   | `Platform()://ASM_Traffic/Model Root/VesiInterface/Vehicle Sensors/ground_truth/GPS_POSITION/GPS_CALC/Heading_deg[i]` |

- `i` is the fellow index (0-based: 0, 1, 2, …).
- If your project uses different path spelling (e.g. `Vesilnterface`, `ModelRoot`), adjust the paths in the script or in ControlDesk accordingly.

## What we see

- **Ego**: One set of (longitude, latitude, heading) in WGS84 for the ego vehicle. Updates as the simulation runs.
- **Fellows**: One set per fellow; each set is (longitude, latitude, heading) in WGS84. Index 0 = first fellow, index 1 = second, etc.
- Values are in **degrees** (not radians). Heading is in degrees; convert to radians if needed (e.g. for Scenic).
- If a path is missing or the experiment is not running, the read script will report an error for that variable.

## Script: `read_gnss_wgs84.py`

Reads the variables above and prints ego and fellow GNSS (WGS84) to the console.

**Prerequisites**

- ControlDesk running with the ASM_Traffic experiment loaded and (typically) simulation running.
- Python with `scenic.simulators.dspace.controldesk.connection` available (run from repo root or with `src` on `PYTHONPATH`).

**Usage**

```bash
cd debug_gps_cord
python read_gnss_wgs84.py
```

**Options**

- `--samples N` – Read N times (default: 1).
- `--interval SEC` – Delay in seconds between samples (default: 0.5).
- `--fellows N` – Number of fellow indices to read (default: 1).

**Examples**

```bash
# Single read: ego + first fellow
python read_gnss_wgs84.py

# 10 samples every 1 s, 2 fellows
python read_gnss_wgs84.py --samples 10 --interval 1 --fellows 2
```

**Output**

- One line per vehicle: `Ego (WGS84)` and `Fellow[i] (WGS84)` with `lon=...° lat=...° heading=...°`.
- On read failure for a variable, that vehicle’s line shows `(read failed)` and the script prints the exception.

## Vehicle Coordinate System E ↔ WGS84 conversion

The dSPACE variables **Pos_x_Vehicle_CoorSys_E[m]**, **Pos_y_Vehicle_CoorSys_E[m]**, **Pos_z_Vehicle_CoorSys_E[m]** (DISP_Plant) give position in a local Cartesian frame in meters. The script **`vehicle_e_to_wgs84.py`** derives the conversion to WGS84 using one known corresponding point (same physical location in both systems).

**Assumption**: Vehicle Coordinate System E is treated as a local **ENU** (East-North-Up) tangent plane: x = East, y = North, z = Up (meters). The script solves for the tangent-plane origin (lon0, lat0, h0) so that the calibration point matches.

**Calibration point** (hardcoded from one snapshot):

- Vehicle E: x = -20.001929315 m, y = -49.881083992 m, z = 3.63857835596 m  
- WGS84: lon = -121.75605141°, lat = 36.58691208°

**Usage**

```bash
cd debug_gps_cord
python vehicle_e_to_wgs84.py           # derive transform and save to vehicle_e_wgs84_transform.json
python vehicle_e_to_wgs84.py --validate   # load saved transform and run round-trip check
```

The script writes **`vehicle_e_wgs84_transform.json`** with `lon0_deg`, `lat0_deg`, `h0_m`. Use the functions `local_to_wgs84(lon0, lat0, h0, x, y, z)` and `wgs84_to_local(lon0, lat0, h0, lon, lat, alt)` for conversions (see `vehicle_e_to_wgs84.py`).

**Note**: If Vehicle E is not ENU (e.g. different axis order or rotation), the conversion would need a second calibration point or a known rotation; the current script assumes ENU.

## Relation to other coordinates

- **Vehicle Coordinate System E**: Local Cartesian (x, y, z) in meters (DISP_Plant). Conversion to WGS84 is given by `vehicle_e_to_wgs84.py` and `vehicle_e_wgs84_transform.json`.
- **RD (Road Designer)**: Simulation world frame; DISP_Plant positions may be in the same or a related frame as Vehicle E. Convert WGS84 ↔ RD with a proper geodetic transform if needed.
- **XODR**: Map frame used by Scenic; transform via `coordinate_transform` (XODR ↔ RD). GNSS (WGS84) is independent of XODR until you define a mapping (e.g. via a reference point or transform).
- **GPS_CALC**: Outputs used here are the model’s computed WGS84 pose (longitude, latitude, heading). They are what we document and read in this folder.

## References

- Ego/Fellow position readback in **RD** (DISP_Plant / FellowTrailer): `src/scenic/simulators/dspace/controldesk/readback.py`, `debug_ego_cord/README.md`.
- dSPACE simulator overview: `src/scenic/simulators/dspace/README.md`.
