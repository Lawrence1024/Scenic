"""SD-15a smoke test: verify `new RacingCar on mainTrack` (and the new
per-vehicle TTL placement) actually place a vehicle at a uniformly-sampled
point inside the expected region.

USAGE (no simulation, just scene generation):
    scenic examples/racing/sampled/smoke_on_mainTrack.scenic --2d \\
        --model scenic.simulators.dspace.racing_model \\
        --count 5 --seed 42

EXPECTED: 5 distinct (x, y) positions for the ego (sampled on the optimal
TTL via the per-vehicle default) AND 5 distinct positions for car2 (on
mainTrack via explicit `on mainTrack`). Per-TTL placement keeps the ego
on a racing line; mainTrack placement spans the full road envelope.

This file does NOT trigger the cosim bridge or any simulation — it just
exercises the Scenic sampling layer over the mainTrack / TTL regions.
"""
param map = localPath('../../../assets/maps/dSPACE/LGS_v1.xodr')
param use2DMap = True
param time_step = 0.01
param control_period = 0.05
param ttlFolder = localPath('../../../assets/ttls/LS_ENU_TTL_CSV')
# Disable the dSPACE/cosim bridge — this is a Scenic-only smoke test.
param launch_veos_ipc_client = False
model scenic.simulators.dspace.racing_model

# Per-vehicle TTL placement (NEW): the RacingCar default
# `position: new Point on ttlRegion(self.ttlFileName)` samples uniformly
# on this car's TTL (here the optimal racing line). No `on R` needed.
ego = new RacingCar with raceNumber 1, \
    with ttlFileName 'ttl_optimal_xodr.csv', \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')

# Explicit `on mainTrack` overrides the default for car2 -- samples
# uniformly anywhere on the full mainTrack polygon (not just one TTL).
car2 = new RacingCar on mainTrack, \
    with raceNumber 2, \
    with ttlFileName 'ttl_left_xodr.csv', \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')
