"""Active-falsification scenario S3: ego vs adversarial blocker.

Negative passing test under VerifAI. Mirrors S2's parameter-declaration
idiom (gap + side via the @distributionFunction synchronization trick,
plus three new blocker-aggression knobs). The fellow uses the new
`FellowActiveBlockBehavior` that mirrors ego's lateral position and
matches ego speed minus an offset, deliberately blocking overtakes.

Per-behavior campaign discipline: this file's parameter space contains
only knobs that affect the blocker. No behavior-selection knob — keeping
the search space focused avoids the "dead variable" problem you'd see
in a single shared S3 with a discrete-behavior-selector + per-behavior
sub-knobs (where unselected branches' params are sampled but unused).

Sampled knobs:

  1. gap_m                     (continuous, m)   -- starting longitudinal gap to blocker
  2. fellow side               (binary)          -- which TTL the blocker starts on (synced
                                                    to its lateral offset; see S2 for the
                                                    @distributionFunction trick)
  3. blocker_speed_offset_mph  (continuous, mph) -- blocker speed = ego speed + this
                                                    (negative = slower so ego closes)
  4. blocker_max_lat_speed_mps (continuous, m/s) -- lateral slew rate cap; higher = more
                                                    aggressive lateral-blocking response
  5. blocker_max_lat_offset_m  (continuous, m)   -- how far off centerline the blocker is
                                                    willing to move when chasing ego

Run command (10-sample CE smoke test):

    python src/scenic/domains/racing/benchmarks/verifai_runner.py \\
        examples/racing/falsifiable/S3_blocker_falsify.scenic \\
        --sampler ce --monitor safety --count 10 --seed 42 --time 3000

Bumping --count to 30-50 is reasonable: a 5-dim space gives CE more room
to converge on adversarial corners.

Runtime knob plumbing: same wrapper-behavior pattern as S2. Scenic does
not auto-resolve Distribution kwargs passed to behavior constructors at
scene-sample time, so the per-knob VerifaiRange values are registered as
`param`s and the wrapper behavior reads them from
`simulation().scene.params` at first activation tick.
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

# Knob 1: longitudinal gap (m). Same range as S2 for comparability.
gap_m = VerifaiRange(20, 60)

# Knob 2: starting side (binary). Same synchronization trick as S2.
from scenic.core.distributions import distributionFunction

_fellow_side_idx = VerifaiDiscreteRange(0, 1)  # 0 = left, 1 = right

@distributionFunction
def _fellow_lat_for_side(idx):
    return [5.0, -5.0][int(idx)]

@distributionFunction
def _fellow_ttl_for_side(idx):
    return ['ttl_left_xodr.csv', 'ttl_right_xodr.csv'][int(idx)]

fellow_lat_offset = _fellow_lat_for_side(_fellow_side_idx)
fellow_ttl_file   = _fellow_ttl_for_side(_fellow_side_idx)

# Knobs 3-5: blocker aggression. Registered as `param`s for the wrapper-
# behavior pattern (Scenic does not resolve Distribution kwargs to
# behavior constructors).
#
# Knob 3: blocker_speed_offset_mph -- the blocker holds at ego.speed +
# this offset. Negative keeps the blocker slower than ego so the gap
# collapses. Range [-15, -2]: at -15 mph the blocker is much slower
# (gap closes fast); at -2 mph the blocker barely concedes.
param blocker_speed_offset_mph = VerifaiRange(-15, -2)

# Knob 4: blocker_max_lat_speed_mps -- the lateral velocity cap on the
# blocker's d slew. Higher = blocker can shift sideways faster to cut
# off ego. Range [1.5, 4.5]: 1.5 m/s is a sluggish defender; 4.5 m/s is
# very aggressive (~10 mph crab).
param blocker_max_lat_speed_mps = VerifaiRange(1.5, 4.5)

# Knob 5: blocker_max_lat_offset_m -- how far off centerline the blocker
# will move when tracking ego's lateral position. Range [3.0, 6.0].
# Smaller = blocker stays near optimal; larger = blocker chases ego
# wider, surfacing pass-feasibility edge cases on the side TTLs.
param blocker_max_lat_offset_m = VerifaiRange(3.0, 6.0)

# --- AGENTS ----------------------------------------------------------------

# Ego: placed uniformly on its own TTL via the per-vehicle default
# (`position: new Point on ttlRegion(self.ttlFileName)`). NOT controlled
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
)

# Blocker: placed at (gap_m, fellow_lat_offset) in race s-t coords,
# on the TTL chosen by the synchronized side sample. See S2 for the
# `at (0, 0)` placeholder rationale.
opponent = new RacingCar at (0, 0), \
    with regionContainedIn everywhere, \
    with _racing_st_offset (gap_m, fellow_lat_offset), \
    with raceNumber 2, \
    with ttlFileName fellow_ttl_file, \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')

# Wrapper behavior: reads sampled knobs from scene params at runtime,
# then delegates to FellowActiveBlockBehavior with literal floats.
behavior _FellowActiveBlockFromParams():
    _params = (getattr(simulation().scene, 'params', None) or {})
    _spd_off = float(_params.get('blocker_speed_offset_mph', -5.0))
    _lat_rate = float(_params.get('blocker_max_lat_speed_mps', 3.0))
    _lat_off = float(_params.get('blocker_max_lat_offset_m', 5.0))
    do FellowActiveBlockBehavior(
        speed_offset_mph=_spd_off,
        max_lat_speed_mps=_lat_rate,
        max_lat_offset_m=_lat_off,
    )

opponent.behavior = _FellowActiveBlockFromParams()
