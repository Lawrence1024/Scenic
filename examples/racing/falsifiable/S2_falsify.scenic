"""SD-28 active-falsification scenario S2: gap + side + fellow speed.

A more comprehensive twin of `S1_falsify.scenic`. S1 samples only one
variable (the longitudinal gap). S2 samples three:

  1. gap_m         (continuous, m)   -- how far ahead the fellow starts
                                        (along ego's route, in meters)
  2. fellow side   (binary)          -- which TTL the fellow follows
                                        (ttl_left_xodr vs ttl_right_xodr).
                                        Determines pass-side geometry; the
                                        runtime FellowFollowTTLGeometricBehavior
                                        steers the fellow onto its TTL
                                        within the first few ticks regardless
                                        of the initial lateral coordinate.
  3. fellow speed  (continuous, mph) -- the cruise speed handed to
                                        FellowFollowTTLGeometricBehavior.

VerifAI's sampler explores the joint space of all three. With CE the
sampler will concentrate on gap-side-speed combinations that break the
ego planner; with halton/random it gives uniform coverage of the
3-dim sample space.

Run command (30 samples, deterministic seed):

    python src/scenic/domains/racing/benchmarks/verifai_runner.py \\
        examples/racing/falsifiable/S2_falsify.scenic \\
        --sampler ce --monitor safety --count 30 --seed 42 --time 3000

Bumping `--count` is reasonable here -- the sample space is 3-dim
instead of S1's 1-dim, so 50-60 samples gives the CE sampler more
room to converge on adversarial regions.

Lateral placement: the fellow is placed at `_racing_st_offset (gap_m, 0)`,
i.e. the same lateral coordinate as ego in route frame. The runtime
TTL-tracking behavior then converges the fellow to its actual TTL
polyline (ttl_left_xodr or ttl_right_xodr) within a few control ticks.
Pre-2026-05-06 we placed the fellow at lat=+/-5 m to "match" the side
TTL, but that lateral coupling was redundant (the runtime behavior
overrides it) and constrained the projection trajectory. Removing it
gives the sampler a cleaner geometric interpretation: gap_m is
honestly the along-route distance from ego.

Speed knob caveat: Scenic does NOT auto-resolve Distribution kwargs
passed to behavior constructors at scene-sample time. Object
properties (like `with _racing_st_offset (...)`) DO get resolved.
We register fellow speed as a `param` and use a thin wrapper
behavior `_FellowFollowFromParams` that reads the resolved scalar
from `simulation().scene.params` at first activation, then delegates
to `FellowFollowTTLGeometricBehavior` with a literal float.
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

# --- ACTIVE-FALSIFICATION KNOBS --------------------------------------------

# Knob 1: longitudinal gap (m). Range bumped from 20-60 to 20-80 on
# 2026-05-06 alongside the lateral-coupling removal: with the runtime
# TTL projection now solely responsible for the fellow's lateral, gap_m
# is an honest along-route distance, so a wider range gives the sampler
# more useful coverage without producing geometrically-degenerate samples.
gap_m = VerifaiRange(20, 80)

# Knob 2: fellow side -- BINARY (0 = left TTL, 1 = right TTL).
# Determines which TTL polyline the fellow tracks at runtime. The
# initial lateral coordinate of the fellow is held at 0 (same as ego's
# route-frame t); the FellowFollowTTLGeometricBehavior pulls the fellow
# onto its TTL within a few control ticks. Pre-2026-05-06 we synthesized
# a +/-5 m initial lateral to "match" the chosen TTL, but that coupling
# was redundant with the runtime tracking behavior and constrained the
# initial-projection geometry; removed for cleaner sample distribution.
from scenic.core.distributions import distributionFunction

_fellow_side_idx = VerifaiDiscreteRange(0, 1)  # 0 = left, 1 = right

@distributionFunction
def _fellow_ttl_for_side(idx):
    return ['ttl_left_xodr.csv', 'ttl_right_xodr.csv'][int(idx)]

fellow_ttl_file = _fellow_ttl_for_side(_fellow_side_idx)

# Knob 3: fellow cruise speed (mph). FellowFollowTTLGeometricBehavior
# accepts mph natively. Range covers slow blocking (15 mph) up through
# fellow-as-fast-as-typical-ego (65 mph). At 65 mph the fellow
# occasionally outruns ego in tight curves where the racing line caps
# ego's actual speed below its 60 m/s target -- those samples produce
# no pass attempts, but CE adapts away from them once it sees the
# uninformative scores.
#
# This one is registered via `param` rather than as a top-level Scenic
# variable because Scenic does NOT auto-resolve Distribution kwargs
# passed to behavior constructors at scene-sample time. By contrast,
# distributions used in object property positions (like `with
# _racing_st_offset (...)`) DO get resolved. The behavior reads the
# already-sampled scalar value via `globalParameters.fellow_speed_mph`.
param fellow_speed_mph = VerifaiRange(15, 65)

# --- AGENTS ----------------------------------------------------------------

# Ego: placed uniformly on its OWN TTL via the per-vehicle default
# `position: new Point on ttlRegion(self.ttlFileName)`. NOT controlled
# by VerifAI -- ego start advances per the global Scenic RNG so each
# sample explores a different track location.
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
    # SD-44 (2026-05-06): the stability guard was previously OFF in S2 falsify
    # because we never explicitly enabled it. Result: all SD-44 work in the
    # guard (friction-circle brake-steer coupling, OBB proximity trigger,
    # sticky exit gate) was silently inert during S2 sweeps. Sample 2 / 7
    # failures we saw were diagnosed as "guard didn't fire" but actually
    # "guard wasn't running." Same set of layers full_stack_runner enables.
    stability_guard_enabled=True,
    assessment_enabled=True,
    commit_abort_enabled=True,
    segment_aware_enabled=True,
)

# Opponent: placed at (gap_m, 0) in race s-t coords, on the TTL chosen
# by the fellow_side sample. The lateral coordinate is intentionally 0
# -- the runtime FellowFollowTTLGeometricBehavior steers the fellow
# onto ttl_left_xodr or ttl_right_xodr as appropriate within a few
# control ticks. See module docstring for rationale.
#
# NOTE on `at (0, 0)`: with ttlFileName a Distribution (sampled via
# VerifaiOptions), the racing_model's default position formula
# `new Point on trackRegion(self.ttlFileName)` chokes -- internally it
# calls `_ttl_category(ttl_file_name)` which does `if not ttl_file_name`
# on what is now a Scenic Distribution and raises
# RandomControlFlowError. Pinning a placeholder world-frame position
# bypasses the default formula. `_racing_st_offset` overrides the
# placeholder at simulation time via modeldesk/placement.py, so the
# value `(0, 0)` here is never actually used by the simulator.
opponent = new RacingCar at (0, 0), \
    with regionContainedIn everywhere, \
    with _racing_st_offset (gap_m, 0), \
    with raceNumber 2, \
    with ttlFileName fellow_ttl_file, \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')

# Wrapper behavior: reads `fellow_speed_mph` from the sampled scene
# params at runtime (first activation tick), then delegates to
# FellowFollowTTLGeometricBehavior with the resolved scalar. This
# detour is necessary because Scenic doesn't auto-resolve Distribution
# kwargs to behavior constructors at scene-sample time.
behavior _FellowFollowFromParams():
    _speed = float((getattr(simulation().scene, 'params', None) or {}).get('fellow_speed_mph', 31))
    do FellowFollowTTLGeometricBehavior(speed_mph=_speed)

opponent.behavior = _FellowFollowFromParams()
