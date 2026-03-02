# Laguna Seca TTL waypoints (LS_ENU_TTL_CSV)

This folder contains **target trajectory line (TTL)** CSV waypoints for the Laguna Seca circuit. All files are in **XODR (map) coordinates** and use offset **(0, 0)** when loaded by the Scenic dSPACE TTL loader.

## Contents

- **ttl_racing_line_xodr.csv** – Main racing line (open; last point need not join first).
- **ttl_racing_line_xodr_closed.csv** – Closed-loop racing line (recommended for lap racing).
- **ttl_fellow_test_xodr_all.csv** – Centerline used by `create_new_ttl` and verification scripts.
- **ttl_17.csv**, **ttl_2.csv**, **ttl_3.csv**, **ttl_9.csv**, **ttl_15.csv**, **ttl_16.csv** – Indexed TTL variants.
- **ttl27_v5.csv** – Fellow/example TTL (e.g. fellow_ttl27_example).
- **gps_origin.txt** – GPS origin (lat/lon/alt) for reference.

## Format

CSV with header `x,y,z` (meters in XODR frame). The loader uses zero offset for this folder.

## Usage

In Scenic, set `ttlFolder` to this folder and optionally `ttlFileName` (e.g. `ttl_racing_line_xodr_closed.csv`):

```scenic
with ttlFolder localPath('../../assets/ttls/LS_ENU_TTL_CSV'), \
     with ttlFileName 'ttl_racing_line_xodr_closed.csv', \
     with ttlDX 0.0, with ttlDY 0.0
```

Default in the dSPACE TTL loader is this folder with offset (0, 0).
