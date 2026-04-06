# Laguna Seca TTL waypoints (LS_ENU_TTL_CSV)

This folder contains **target trajectory line (TTL)** CSV waypoints for the Laguna Seca circuit. All files are in **XODR (map) coordinates** and use offset **(0, 0)** when loaded by the Scenic dSPACE TTL loader.

## Contents

- **ttl_main_road.csv** – Main road closed loop. Used for Lap route, segment map, and road-index projection when `ttlFolder` is set.
- **ttl_pitlane.csv** – Pit lane closed loop. Used for Pit route and segment map.
- **ttl_optimal.csv**, **ttl_optimal_xodr.csv** – Optimal / racing-line style trajectories for behaviors and examples.
- **ttl_left.csv**, **ttl_left_xodr.csv**, **ttl_right.csv**, **ttl_right_xodr.csv** – Lane-offset variants (XODR-aligned).
- **ttl_pit.csv**, **ttl_pit_xodr.csv** – Pit-related trajectories.
- **gps_origin.txt** – GPS origin (lat/lon/alt) for reference.

## Format

CSV with header `x,y,z` (meters in XODR frame). The loader uses zero offset for this folder.

## Usage

In Scenic, set `ttlFolder` to this folder and optionally `ttlFileName` (e.g. `ttl_main_road.csv` or `ttl_pitlane.csv`):

```scenic
with ttlFolder localPath('../../assets/ttls/LS_ENU_TTL_CSV'), \
     with ttlFileName 'ttl_main_road.csv'
```

Default in the dSPACE TTL loader is this folder with `ttl_main_road.csv` when params omit `ttlFileName`. When ego placement is similar to both main road and pitlane (TTL distances within ~2 m), the loader assigns **Lap** (main road) by default; the assigned route is logged (e.g. `[Ego] Assigned route: Lap`).
