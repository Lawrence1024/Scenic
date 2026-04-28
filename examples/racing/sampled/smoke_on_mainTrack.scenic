"""SD-15a smoke test: verify `new RacingCar on mainTrack` actually places
a vehicle at a uniformly-sampled point inside mainTrack (the union of main
racing roads, with the pit lane excluded).

USAGE (no simulation, just scene generation):
    scenic examples/racing/sampled/smoke_on_mainTrack.scenic --2d \\
        --model scenic.simulators.dspace.racing_model \\
        --count 5 --seed 42

EXPECTED: 5 distinct (x, y) positions all on the main racing line, none in
the pit lane. If Scenic logs `Generated scene` 5 times with different ego
positions, the region wiring is correct.

This file does NOT trigger the cosim bridge or any simulation — it just
exercises the Scenic sampling layer over the mainTrack region.
"""
param map = localPath('../../../assets/maps/dSPACE/LGS_v1.xodr')
param use2DMap = True
param time_step = 0.01
param control_period = 0.05
param ttlFolder = localPath('../../../assets/ttls/LS_ENU_TTL_CSV')
# Disable the dSPACE/cosim bridge — this is a Scenic-only smoke test.
param launch_veos_ipc_client = False
model scenic.simulators.dspace.racing_model

# Sample ego uniformly over the mainTrack region. Scenic should return
# a different (x, y) on each --count iteration when --seed is held.
ego = new RacingCar on mainTrack, \
    with raceNumber 1, \
    with ttlFileName 'ttl_optimal_xodr.csv', \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')
