"""SD-28 active-falsification scenario S2: gap + side + fellow speed.

A more comprehensive twin of `S1_falsify.scenic`. S1 samples only one
variable (the longitudinal gap). S2 samples three:

  1. gap_m         (continuous, m)   -- how far ahead the fellow starts
  2. fellow side   (binary)          -- which TTL the fellow occupies
                                        (left vs right racing line);
                                        lateral offset and ttlFileName
                                        are kept synchronized so that
                                        a "left" sample places the fellow
                                        at +5m lateral on ttl_left_xodr.
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

Synchronization trick: VerifAI's sampler treats each VerifaiRange /
VerifaiDiscreteRange as an independent dimension, so two separate
samplers (one for lateral offset, one for TTL filename) would let
VerifAI emit inconsistent combos like lat=+5 with the right TTL.
We sample ONE `VerifaiDiscreteRange(0, 1)` and route it through two
`@distributionFunction`-decorated helpers (`_fellow_lat_for_side` and
`_fellow_ttl_for_side`). Both helpers share the same underlying
sampled index, so left/right always agrees between offset and TTL.

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

# Knob 1: longitudinal gap (m).
gap_m = VerifaiRange(20, 60)

# Knob 2: fellow side -- BINARY. The lateral offset and the TTL
# filename describe the same physical placement so they must stay
# synchronized. Two independent VerifaiOptions would let VerifAI
# sample inconsistent combos like (lat=+5, right TTL).
#
# We sample ONE VerifaiDiscreteRange and route it through two
# Python helper functions; Scenic auto-lifts these to FunctionDistribution
# instances that resolve at sampling time. Both helpers share the
# same underlying sampled index, so left/right always agrees between
# the lateral offset and the TTL filename.
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
)

# Opponent: placed at (gap_m, fellow_lat_offset) in race s-t coords,
# on the TTL chosen by the same fellow_setup sample.
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
    with _racing_st_offset (gap_m, fellow_lat_offset), \
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
