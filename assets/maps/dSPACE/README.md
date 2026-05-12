# dSPACE Maps for Scenic

This directory contains OpenDRIVE and dSPACE RoadNetwork assets for the Laguna Seca racing circuit.

## Files

| File | Purpose |
|------|--------|
| **LGS_v1.xodr** | Main OpenDRIVE map used by the racing domain, visualization, and dSPACE simulator (default map). |
| **LGS_v1_gps_rd_calibration.json** | GPS↔RD calibration data for the dSPACE geometry pipeline (LGS_v1 frame). |
| **LVGS.xodr** | Las Vegas variant OpenDRIVE map. |
| **LGS_v1_MainTrack_FromTTL.xodr** | Main-track-only XODR produced by `src/scenic/simulators/dspace/create_new_ttl/build_main_track_xodr.py` (run from repo root); used by the TTL tooling (e.g. interactive map visualizer). Not committed; generate on demand. |
| **README.md** | This file. |

## Usage

- **Racing / visualization:** Use `LGS_v1.xodr` as the map (e.g. `--map assets/maps/dSPACE/LGS_v1.xodr` or default in the segment visualizer).
- **dSPACE simulator:** The dSPACE model and geometry pipeline use `LGS_v1.xodr` and `LGS_v1_gps_rd_calibration.json` for accurate placement and readback.

Conversion from RD to XODR: `src/scenic/simulators/dspace/converters/rd_to_xodr.py`.
