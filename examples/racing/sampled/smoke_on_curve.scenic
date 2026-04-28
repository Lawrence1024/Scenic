"""SD-24 smoke test: verify the new curve / straight regions and the
unified `trackRegion(ttlFileName, segment)` placement pipeline actually
sample where they should.

USAGE (no simulation, just scene generation):
    scenic examples/racing/sampled/smoke_on_curve.scenic --2d \\
        --model scenic.simulators.dspace.racing_model \\
        --count 5 --seed 42

EXPECTED:
- ``ego`` rides the optimal TTL but only on curve sections — its (x, y)
  per sample should land on the optimal-TTL polyline AND inside one of
  Laguna Seca's corner polygons (mainCurve).
- ``fellow`` lands anywhere on the full curve polygon (main + pit
  curves union) — gives lateral wiggle that the ego placement does not.
- ``car3`` lands on mainStraight only (explicit cross-product region;
  no TTL involvement).

This file does NOT trigger the cosim bridge or any simulation — it
just exercises the Scenic sampling layer over the new SD-24 regions.
"""
param map = localPath('../../../assets/maps/dSPACE/LGS_v1.xodr')
param use2DMap = True
param time_step = 0.01
param control_period = 0.05
param ttlFolder = localPath('../../../assets/ttls/LS_ENU_TTL_CSV')
# Disable the dSPACE/cosim bridge — this is a Scenic-only smoke test.
param launch_veos_ipc_client = False
model scenic.simulators.dspace.racing_model

# TTL-aware curve placement. Note: Scenic does not bind `self` in inline
# `with` specifiers (only in class defaults), so we pass the literal TTL
# filename to `trackRegion(...)`. Result: the optimal TTL polyline ∩
# mainCurve, since the filename pattern resolves to category 'main'.
# Each sample lands EXACTLY on the racing line AND inside a corner.
ego = new RacingCar with raceNumber 1, \
    with ttlFileName 'ttl_optimal_xodr.csv', \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV'), \
    with position new Point on trackRegion('ttl_optimal_xodr.csv', 'curve')

# Lateral wiggle: `on curve` is the full pit + main curve union (no TTL
# implication). The fellow can land anywhere within any curve polygon.
fellow = new RacingCar on curve, \
    with raceNumber 2

# Explicit cross-product region — no TTL routing, just a polygon.
car3 = new RacingCar on mainStraight, \
    with raceNumber 3
