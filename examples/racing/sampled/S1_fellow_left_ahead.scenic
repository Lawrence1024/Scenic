"""SD-15b sampled scenario S1: fellow on left TTL ahead of ego, both on mainTrack.

Replaces the F-bank's fixed-position F3L scenario with a Scenic-sampled
version. Each --count iteration places ego at a different uniformly-sampled
point on mainTrack (excludes pit lane); fellow is placed at a variable gap
in [20, 60] m ahead and a fixed +5 m left of ego (so fellow rides the
left racing line).

This is the first scenario in the sampled bank (the falsifiable pipeline).
Run via the in-process driver in halton mode for uniform coverage:
    python src/scenic/domains/racing/benchmarks/verifai_runner.py \\
        examples/racing/sampled/S1_fellow_left_ahead.scenic \\
        --sampler halton --monitor min --count 10 --seed 42 --time 3000

OR a single sample directly:
    scenic examples/racing/sampled/S1_fellow_left_ahead.scenic --2d \\
        --model scenic.simulators.dspace.racing_model --simulate \\
        --count 1 --seed 42 --time 3000 *>S1.log

INTENT (the falsification thesis):
  At a fixed seed the runner samples N starting layouts. Across N runs we
  observe how the smart ego performs at different track sections (corners
  vs straights, near vs far from pit). Failures (collision, ABORT, low
  commit_pass_success rate) flag layouts that need attention.

  Because the Scenic seed is held, every re-run with the same seed gives
  the same layouts — debugging is fully reproducible. Vary the seed to
  widen coverage, or pass --seed to the runner.
"""
param map = localPath('../../../assets/maps/dSPACE/LGS_v1.xodr')
param use2DMap = True
param time_step = 0.01
param control_period = 0.05
param ttlFolder = localPath('../../../assets/ttls/LS_ENU_TTL_CSV')
param launch_veos_ipc_client = True
param scenic_control = True
param fellowHarnessLog = True
param prediction_enabled = True
model scenic.simulators.dspace.racing_model

# SAMPLED GAP: delta_s uniformly in [20, 60] m. Fellow at +5 m left of ego.
# Scenic resolves the Range when constructing the scene; placement.py reads
# the resolved float at simulation time and computes (delta_s, delta_t).
gap_m = Range(20, 60)

# Ego placed uniformly on its OWN TTL (ttl_optimal_xodr.csv). The RacingCar
# default `position: new Point on ttlRegion(self.ttlFileName)` resolves the
# region per-vehicle from the car's own ttlFileName attribute, so we don't
# need an explicit `on mainTrack` here. See racing/model.scenic:147 (default)
# and ttlRegion helper at racing/model.scenic:103.
ego = new RacingCar with raceNumber 1, \
    with ttlFileName 'ttl_optimal_xodr.csv', \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')

ego.behavior = FollowRacingLineMPCBehavior(
    target_speed=60,
    manage_gears=True,
    use_waypoints=True,
    mpc_config_path=None,
    prediction_enabled=globalParameters.prediction_enabled,
    tactical_planner_enabled=True,
)

# Fellow at (delta_s, +5) relative to ego in race s-t coords. The dSPACE
# placement layer (modeldesk/placement.py:413) resolves this against the
# ego anchor at simulation start, so fellow ends up gap_m ahead and 5 m
# left of wherever Scenic placed ego. Scenic's initial sample uses the
# fellow's OWN ttlFileName (left TTL) via the per-vehicle default.
opponent = new RacingCar with _racing_st_offset (gap_m, 5), \
    with raceNumber 2, \
    with ttlFileName 'ttl_left_xodr.csv', \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')

opponent.behavior = FellowFollowTTLGeometricBehavior(speed_mph=20)
