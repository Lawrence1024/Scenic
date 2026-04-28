"""SD-16 active-falsification scenario S1: VerifAI sampler controls the gap.

Drop-in twin of `examples/racing/sampled/S1_fellow_left_ahead.scenic` with
ONE difference -- the fellow gap is wrapped in `VerifaiRange(20, 60)`
instead of `Range(20, 60)`. That single change promotes the scenario into
VerifAI's sampler space (`scenic.core.external_params.VerifaiSampler`),
so an outer falsification loop can drive the gap toward layouts that
break the ego planner.

The legacy `Range`-based S1 stays untouched in `examples/racing/sampled/`
for the subprocess-style `sampled_runner.py`. This file targets the
in-process VerifAI driver:

    python src/scenic/domains/racing/benchmarks/verifai_runner.py \\
        examples/racing/falsifiable/S1_falsify.scenic \\
        --sampler ce --monitor min --count 50 --seed 42 --time 3000

The runner overrides `verifaiSamplerType` via its `--sampler` flag, so
this file does NOT pin a sampler -- it is sampler-agnostic and works with
halton (smoke), ce (active falsification), bo (Bayesian opt), random, etc.

For everything else (placement semantics, behaviors, TTL wiring) see the
docstring on the original S1 scenario.
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

# ACTIVE-FALSIFICATION KNOB: the gap is the only VerifAI-controlled
# parameter in this scenario. Wrapping in VerifaiRange makes Scenic's
# compiler register a VerifaiParameter; the runner picks the sampler
# type via `param verifaiSamplerType` (default 'halton', overridden by
# verifai_runner's `--sampler` flag).
gap_m = VerifaiRange(20, 60)

# Ego placed uniformly on its OWN TTL (ttl_optimal_xodr.csv) via the
# per-vehicle default `position: new Point on ttlRegion(self.ttlFileName)`.
# This placement is NOT controlled by VerifAI -- it advances per the
# global Scenic RNG, so iteration N's ego start depends on N. By design:
# VerifAI controls only what is wrapped in `VerifaiRange/Options/etc.`.
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

# Fellow at (gap_m, +5) relative to ego in race s-t coords -- the gap
# float here is whatever VerifAI sampled this iteration.
opponent = new RacingCar with _racing_st_offset (gap_m, 5), \
    with raceNumber 2, \
    with ttlFileName 'ttl_left_xodr.csv', \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')

opponent.behavior = FellowFollowTTLGeometricBehavior(speed_mph=20)
