"""SD-24 active-falsification scenario: ego start anywhere on a Laguna Seca
corner, fellow at variable gap ahead.

Replaces the F8 / F10 / F11 / F12 family — each of which hardcoded a
specific corner-entry (x, y) for the ego start — with a single scenario
where the ego is uniformly sampled across the union of all main-loop
curve sections (Corkscrew, Andretti, etc.) along the optimal racing
line. With ``--sampler ce``, VerifAI biases the fellow gap toward
values that break the planner; the corner is sampled uniformly via
Scenic's global RNG (deterministic at fixed --seed thanks to SD-22).

USAGE:
    python src/scenic/domains/racing/benchmarks/verifai_runner.py \\
        examples/racing/falsifiable/F_curve_falsify.scenic \\
        --sampler ce --monitor safety --count 50 --seed 42 --time 3000

PLACEMENT SEMANTICS:
- ``ego`` rides the optimal TTL polyline, restricted to corner sections
  (``trackRegion('ttl_optimal_xodr.csv', 'curve')`` resolves to the
  TTL ∩ mainCurve sub-polyline).
- ``opponent`` is placed at race-frame offset ``(gap_m, +5)`` relative
  to the ego — VerifAI samples ``gap_m`` from ``VerifaiRange(20, 60)``.
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

# ACTIVE-FALSIFICATION KNOB: same as S1_falsify.
gap_m = VerifaiRange(20, 60)

# Ego on the optimal TTL, restricted to corner sections. Each iteration
# samples uniformly along the polyline that lies inside any Laguna Seca
# corner — Corkscrew, Andretti link, T2/T3, etc. Compared to the F-bank's
# hardcoded F8/F10/F11/F12 coordinates, this gives the falsifier ONE
# scenario that exercises every corner entry.
ego = new RacingCar with raceNumber 1, \
    with ttlFileName 'ttl_optimal_xodr.csv', \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV'), \
    with position new Point on trackRegion('ttl_optimal_xodr.csv', 'curve')

ego.behavior = FollowRacingLineMPCBehavior(
    target_speed=60,
    manage_gears=True,
    use_waypoints=True,
    mpc_config_path=None,
    prediction_enabled=globalParameters.prediction_enabled,
    tactical_planner_enabled=True,
)

# Fellow at (gap_m, +5) relative to ego — same idiom as S1_falsify.
opponent = new RacingCar with _racing_st_offset (gap_m, 5), \
    with raceNumber 2, \
    with ttlFileName 'ttl_left_xodr.csv', \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')

opponent.behavior = FellowFollowTTLGeometricBehavior(speed_mph=20)
