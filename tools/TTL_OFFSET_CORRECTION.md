# TTL Offset Correction

## Problem Identified

The original TTL coordinate offsets were **incorrect**, causing TTLs to appear outside track boundaries.

## Original (Incorrect) Offsets
- **dx**: -53.6 meters
- **dy**: -15.7 meters

## Corrected Offsets (Based on Alignment Testing)
- **dx**: -28.6 meters (+25.0 correction)
- **dy**: -45.7 meters (-30.0 correction)

## What Was Wrong?

The TTLs themselves were **calculated correctly** - they are valid optimized racing lines. However, the **coordinate transformation** to align them with the XODR track coordinate system was incorrect.

The issue was:
1. TTLs are stored in ENU (East-North-Up) GPS coordinates
2. XODR track uses a different local coordinate system
3. The transformation between these systems was miscalculated

## Impact

With the corrected offsets:
- TTLs now properly align with track boundaries
- All TTLs fit within the track (as optimized racing lines should)
- The visualization shows TTLs correctly positioned on the track

## How to Verify

Run the offset finder tool to verify alignment:
```bash
python tools/find_ttl_offsets.py --ttl assets/ttls/LS_ENU_TTL_CSV/usable/ttl_17.csv
```

Or use the comparison tool with corrected offsets:
```bash
python tools/compare_ttls.py --dir assets/ttls/LS_ENU_TTL_CSV/usable
```

## Updated Files

The following files have been updated with corrected defaults:
- `src/scenic/simulators/dspace/ttl/loader.py` - Default offsets updated
- `tools/compare_ttls.py` - Default offsets updated

## Backward Compatibility

If you have existing Scenic files using the old offsets, you can override them:
```scenic
param ttlDX = -28.6
param ttlDY = -45.7
```

Or specify per-vehicle:
```scenic
fellow1 = new RacingCar with ttlDX -28.6, ttlDY -45.7
```

