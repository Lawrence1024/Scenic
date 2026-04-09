"""Racing-specific behaviors for dynamic agents.

These behaviors extend the driving domain behaviors with racing-specific
strategies and maneuvers, using abstract racing protocols that simulators
must implement.
"""

from scenic.domains.driving.behaviors import *
from scenic.domains.driving.actions import SetThrottleAction, SetBrakeAction, SetSteerAction
from scenic.domains.racing.actions import SetMaxSpeedAction, SetTTLAction, SetSpeedLimitAction, SetTTLSelectionAction, SetTargetGapAction, SetStrategyAction, SetPowertrainModeAction, SetScaleFactorAction, SetPush2PassAction, StopCarAction, SetGearAction, SetFellowPlantAction
import scenic.domains.racing.model as _racing
import math
import numpy as np

from scenic.domains.racing.waypoints import (
    initialize_racing_waypoint_start_index,
    select_forward_racing_waypoint,
)
from scenic.domains.racing.fellow import (
    compute_constant_offset_plant_command,
    compute_fellow_swerve_out_of_control_command,
    compute_follow_ttl_geometric_plant_command,
    compute_sudden_stop_plant_command,
)
from scenic.domains.racing.segments import (
    build_waypoint_segment_map,
    build_waypoint_segment_map_from_ttl,
    get_segment_at_waypoint,
    get_segment_at_waypoint_ring_strict,
    get_segment_label,
)
from scenic.domains.racing.constants import DELTA_MAX_RAD
from scenic.domains.racing.mode import RACING_MODE_MAIN, RACING_MODE_PIT


behavior FellowConstantSpeedTrackOffsetBehavior(speed_mph=31):
    """Constant-speed fellow with lateral offset fixed from Scenic placement.

    Intended for **dSPACE** traffic fellows controlled via External_Signals
    (``Const_v_Fellows_External``, ``Const_d_Fellows_External``): each step this behavior
    **takes** :class:`~scenic.domains.racing.actions.SetFellowPlantAction` with **v** from
    **speed_mph** (converted to km/h) and **d** from lateral placement (``_route_s_t``).
    The dSPACE controller reads ``_fellow_plant_state`` and writes those values to the
    platform. Not MPC; does not populate throttle/steer control state.

    Other simulators do not apply this unless they implement the same (v, d) contract.
    """
    self._fellow_vd_plant_enabled = True
    while True:
        v_kmh, d_m, mode = compute_constant_offset_plant_command(self)
        take SetFellowPlantAction(v_kmh, d_m)
        self._fellow_plant_log_mode = mode
        wait

behavior FellowFollowTTLGeometricBehavior(speed_mph=31):
    """dSPACE fellow: constant speed and lateral **d** from TTL geometry (no PID/MPC).

    Each step **takes** :class:`~scenic.domains.racing.actions.SetFellowPlantAction` with **v**
    from **speed_mph** and **d** from feedforward δ(s) on control-interval steps (aligned
    with simulator readback) on the main track centerline (optimal TTL vs ``ttl_main_road``),
    matching the racing line used by MPC fellows. The dSPACE controller writes those values
    to External_Signals.

    Waypoint progress uses :func:`select_forward_racing_waypoint` (same family as
    ``FollowRacingLineMPCBehavior`` / racing line followers) for a stable polyline index;
    **d** is pure geometry.

    Requires **Lap** route, ``ttlFolder``, ``ttlFileName`` (optimal CSV), TTL waypoints on
    the agent, and a valid delta table. Uses ``dspaceActor`` pose from readback. Other
    simulators may ignore this behavior.
    """
    self._fellow_vd_plant_enabled = True
    while True:
        v_kmh, d_m, mode = compute_follow_ttl_geometric_plant_command(
            self, simulation(), speed_mph
        )
        take SetFellowPlantAction(v_kmh, d_m)
        self._fellow_plant_log_mode = mode
        wait

behavior FellowSuddenStopIntervalBehavior(speed_mph=150, interval=20.0, duration=3.0):
    """dSPACE fellow: periodic full stops (commanded v=0), then back to cruise speed.

    Uses simulation time (:obj:`Simulation.currentRealTime`). Each cycle lasts
    **interval + duration** seconds: cruise at **speed_mph** for **interval** seconds,
    then commanded **0 km/h** for **duration** seconds, then repeat forever. Unlike
    :obj:`FellowSwerveOutOfControlBehavior`, lateral **d** always tracks TTL geometry
    (no open-loop swerve legs): each step uses the same δ(s) path as
    :obj:`FellowFollowTTLGeometricBehavior` (Lap route, ``ttlFolder``, ``ttlFileName``,
    optimal CSV, waypoints). For placement-only fallback when geometry is inactive, see
    that behavior's requirements.

    Each step **takes** :class:`~scenic.domains.racing.actions.SetFellowPlantAction` with
    outputs from :func:`compute_sudden_stop_plant_command`; the dSPACE
    :class:`~scenic.simulators.dspace.vehicle.controller.VehicleController` writes them to
    ``Const_v_Fellows_External`` / ``Const_d_Fellows_External``.

    Example scene: ``examples/combined/fellow_sudden_stop.scenic``.

    Args:
        speed_mph: Cruise speed between stops in **mph**. Default **150**.
        interval: Cruise phase length in seconds (≥ 0). Default **20**.
        duration: Stop phase length in seconds (commanded longitudinal v=0; ≥ 0). Default **3**.
            If **duration** is **0**, the fellow stays in cruise only (no stop phase).
    """
    self._fellow_vd_plant_enabled = True
    while True:
        v_kmh, d_m, mode = compute_sudden_stop_plant_command(
            self,
            simulation(),
            speed_mph=speed_mph,
            interval_s=interval,
            duration_s=duration,
        )
        take SetFellowPlantAction(v_kmh, d_m)
        self._fellow_plant_log_mode = mode
        wait

behavior FellowSwerveOutOfControlBehavior(
    speed_mph=150,
    interval=10.0,
    swerve_right_s=1.8,
    swerve_left_s=2.0,
    swerve_amp_m=6.0,
    swerve_d_rate_m_s=6.5,
    stop_hold_d=True,
):
    """dSPACE fellow: TTL cruise, then gradual swerve right then left, then stop.

    Default numeric parameters match ``examples/combined/fellow_swerve_out_of_control.scenic``.

    For **interval** seconds the fellow matches :obj:`FellowFollowTTLGeometricBehavior`
    lateral **d** (delta(s)) at **speed_mph**. Then **d** slews toward **-swerve_amp_m** m
    (right of centerline) and toward **+swerve_amp_m** m (left) at up to **swerve_d_rate_m_s**
    m/s change in commanded **d**, so lateral commands ramp instead of stepping. **swerve_right_s**
    and **swerve_left_s** bound how long each leg lasts; each leg should be at least about
    **swerve_amp_m / swerve_d_rate_m_s** seconds to reach full offset in one direction, and
    crossing from **-amp** to **+amp** needs about **2 * swerve_amp_m / swerve_d_rate_m_s** in
    the second leg. Then **v = 0**. If **stop_hold_d** is true (default), commanded **d** stays
    at the end-of-maneuver value so the car does not creep laterally while stationary; if false,
    **d** slews toward TTL delta(s) like a moving target (can look like sliding in place).

    Centerline convention: positive **d** = left, negative = right (same as placement).

    Requires Lap route, ``ttlFolder``, ``ttlFileName``, waypoints, and delta table when
    using TTL phases (same as geometric fellow).

    Each step **takes** :class:`~scenic.domains.racing.actions.SetFellowPlantAction` with
    outputs from :func:`compute_fellow_swerve_out_of_control_command`; dSPACE writes plant
    outputs to fellow External_Signals (same as other (v, d) plant behaviors).

    Args:
        speed_mph: Cruise and swerve-leg speed in **mph**. Default **150**.
        interval: Seconds of TTL cruise before the swerve maneuver. Default **10**.
        swerve_right_s: Duration (s) of the leg slewing **d** toward **-swerve_amp_m** (right).
            Default **1.8**.
        swerve_left_s: Duration (s) of the leg slewing **d** toward **+swerve_amp_m** (left).
            Default **2.0**.
        swerve_amp_m: Lateral command magnitude (m) for each swerve target relative to
            centerline (positive = left, negative = right). Default **6.0**.
        swerve_d_rate_m_s: Maximum rate of change of commanded **d** (m/s). Default **6.5**.
        stop_hold_d: If true (default), after **v = 0** keep **d** fixed at the end of the
            maneuver; if false, slew **d** toward TTL δ(s) while stopped (can look like drift).
    """
    self._fellow_vd_plant_enabled = True
    while True:
        v_kmh, d_m, mode = compute_fellow_swerve_out_of_control_command(
            self,
            simulation(),
            speed_mph=speed_mph,
            interval_s=interval,
            swerve_right_s=swerve_right_s,
            swerve_left_s=swerve_left_s,
            swerve_amp_m=swerve_amp_m,
            swerve_d_rate_m_s=swerve_d_rate_m_s,
            stop_hold_d=stop_hold_d,
        )
        take SetFellowPlantAction(v_kmh, d_m)
        self._fellow_plant_log_mode = mode
        wait

behavior FollowRacingLineMPCBehavior(target_speed=30, manage_gears=True, use_waypoints=True, mpc_config_path=None):
    """Follow the car's TTL using MPC (Model Predictive Control) for lateral control.
    
    Primary Scenic behavior for line-following on the racing TTL. Lateral and longitudinal
    control use MPC for predictive tracking, especially in high-speed cornering.
    Lookahead distance for the MPC path is computed internally from speed and horizon
    (see lookahead_dist in the behavior); it is not a user parameter.
    
    Outputs NORMALIZED control signals (-1.0 to 1.0).
    The Simulator (simulator.py) automatically scales these to dSPACE VesiInterface units.
    
    Args:
        target_speed: Target speed in m/s
        manage_gears: Whether to automatically manage gears
        use_waypoints: Whether to use waypoint-based control
        mpc_config_path: Path to MPC config YAML file (optional, uses default if None)
    """
    
    # SETUP & DEFAULTS
    if not hasattr(self, 'ttl') or self.ttl is None:
        take SetTTLAction(track.racingLine if hasattr(track, 'racingLine') and track.racingLine else mainRacingRoad)
    take SetMaxSpeedAction(target_speed)
    
    throttle_limit = 1.0

    # Ego: base throttle cap (lowered further on large CTE); raised for more throttle on straights
    if self is simulation().scene.egoObject:
        throttle_limit = 1.0   # No cap so MPC can reach 140 mph on straights (IAC vehicle)

    # Steering slew-rate and CTE safety thresholds
    max_steer_delta = 0.2          # per step (normalized units)
    cte_slowdown_threshold = 15.0  # m: start slowing down
    cte_stop_threshold = 50.0      # m: full brake to avoid runaway
    
    # Progressive throttle reduction thresholds (for better control when CTE is large)
    cte_throttle_reduction_start = 2.0   # m: start reducing throttle progressively (lowered from 5.0)
    cte_throttle_reduction_max = 10.0    # m: maximum throttle reduction zone
    min_throttle_at_large_cte = 0.03     # minimum throttle when CTE > 10m

    # Get Controllers: Longitudinal MPC + Lateral MPC
    _lon_controller, _lat_controller = simulation().getRacingControllers(self, use_mpc=True, mpc_config_path=mpc_config_path)
    _fbhv = getattr(self, '_follow_mpc_behavior_log_prefix', '[FollowRacingLineMPCBehavior]')

    # Gear thresholds (m/s)
    gear_up_thresholds = [0.0, 12.0, 22.0, 32.0, 42.0, 52.0]
    gear_down_thresholds = [0.0, 9.0, 18.0, 28.0, 38.0, 48.0]

    wp_last_idx = 0

    # CRITICAL: Wait for simulation to initialize and position to be available
    wait
    while not hasattr(self, 'position') or self.position is None:
        wait
    # Also wait for dSPACE heading readback so waypoint-ahead initialization uses a real heading.
    while (not hasattr(self, 'dspaceActor') or self.dspaceActor is None
           or not hasattr(self.dspaceActor, 'heading') or self.dspaceActor.heading is None):
        wait
    
    # Initialize waypoint index based on starting position
    # CRITICAL FIX: Find the first waypoint that is AHEAD of the vehicle
    wp_list_init = (self.waypoints if hasattr(self, 'waypoints') else None)
    if use_waypoints and wp_list_init and len(wp_list_init) >= 2:
        try:
            px = float(self.position.x); py = float(self.position.y)
            
            car_heading = None
            car_heading_src = None
            if hasattr(self, 'dspaceActor') and self.dspaceActor is not None and hasattr(self.dspaceActor, 'heading') and self.dspaceActor.heading is not None:
                try:
                    car_heading = float(self.dspaceActor.heading)
                    car_heading_src = "dspaceActor.heading"
                except:
                    pass
            if car_heading is None and hasattr(self, 'heading') and self.heading is not None:
                try:
                    car_heading = float(self.heading)
                    car_heading_src = "self.heading"
                except:
                    pass
            if car_heading is None:
                print(f"{_fbhv} Warning: heading unavailable at init; dot-product ahead search disabled")
            
            # Step 1: Find the nearest waypoint (by distance)
            nearest_idx = 0
            best_d2 = 1e18
            for i in range(len(wp_list_init)):
                wx, wy = float(wp_list_init[i][0]), float(wp_list_init[i][1])
                dx = px - wx; dy = py - wy
                d2 = dx*dx + dy*dy
                if d2 < best_d2:
                    best_d2 = d2; nearest_idx = i
            
            # Step 2: If we have a valid heading, find the first waypoint AHEAD of the vehicle
            # This prevents trying to track waypoints behind the vehicle
            if car_heading is not None:
                # Vehicle forward direction (math is already imported at module level)
                veh_fx = math.cos(car_heading)
                veh_fy = math.sin(car_heading)
                
                # Starting from nearest waypoint, find first one ahead (closed loop: wrap)
                # A waypoint is "ahead" if the dot product of (vehicle_to_waypoint) and (vehicle_forward) > 0
                n_wp = len(wp_list_init)
                wp_last_idx = nearest_idx

                for off in range(0, min(100, n_wp)):
                    i = (nearest_idx + off) % n_wp
                    wx, wy = float(wp_list_init[i][0]), float(wp_list_init[i][1])
                    to_wp_x = wx - px
                    to_wp_y = wy - py
                    dot_product = to_wp_x * veh_fx + to_wp_y * veh_fy

                    if dot_product > 0:  # Waypoint is ahead
                        wp_last_idx = i
                        wp_dist = (to_wp_x*to_wp_x + to_wp_y*to_wp_y) ** 0.5
                        print(f"{_fbhv} Initialized: starting at ({px:.2f}, {py:.2f}), heading={car_heading*180/math.pi:.1f}deg (src={car_heading_src})")
                        print(f"  Found first waypoint AHEAD: index={wp_last_idx} at ({wx:.2f}, {wy:.2f}), distance={wp_dist:.2f}m")
                        print(f"  Dot product={dot_product:.2f} (positive means ahead)")
                        break
                else:
                    wp_last_idx = nearest_idx
                    print(f"{_fbhv} Warning: No waypoint ahead found in search window, using nearest waypoint {nearest_idx}")
            else:
                # No heading available, use nearest waypoint
                wp_last_idx = nearest_idx
                print(f"{_fbhv} Initialized (no heading): nearest waypoint index={nearest_idx}, distance={best_d2**0.5:.2f}m")
        except Exception as e:
            print(f"{_fbhv} Warning: Could not initialize waypoint index: {e}, starting from index 0")
            wp_last_idx = 0
        
        # Initialize waypoint progress tracking
        if not hasattr(self, '_waypoints_passed'):
            self._waypoints_passed = 0

    while True:
        _fl_mpc = getattr(self, '_follow_mpc_log_prefix', '[FollowRacingLineMPC]')
        # Calculate Control Signals
        current_speed = (self.speed if self.speed is not None else 0)

        # Non-control-step fast path: exit before any heavy work (no waypoint/profile, NumPy, readbacks, progress, controller assembly)
        _run_full_control = (simulation().is_control_step if hasattr(simulation(), 'is_control_step') else True) or not hasattr(self, '_last_final_steer')
        if not _run_full_control:
            self._fastpath_ticks = getattr(self, '_fastpath_ticks', 0) + 1
            _fc = getattr(self, '_full_control_ticks', 0)
            _fp = getattr(self, '_fastpath_ticks', 0)
            if (_fc + _fp) % 100 == 0 and (_fc + _fp) > 0:
                print(f"{_fl_mpc} full_control_ticks={_fc} fastpath_ticks={_fp}")
            final_steer = getattr(self, '_last_final_steer', 0.0)
            final_throttle = getattr(self, '_last_final_throttle', 0.0)
            final_brake = getattr(self, '_last_final_brake', 0.0)
            # Pit mode (segment-based): coast when speed >= 35 mph, else cap throttle at 50%
            PIT_LIMIT_MS = 15.646   # 35 mph — same as PIT_MAX_SPEED_MS in full-control block
            if getattr(self, '_in_pit_by_segment', False):
                if current_speed >= PIT_LIMIT_MS:
                    final_throttle = 0.0
                else:
                    final_throttle = min(final_throttle, 0.5)
            _fast_actions = [SetSteerAction(final_steer), SetThrottleAction(final_throttle), SetBrakeAction(final_brake)]
            take _fast_actions
            wait
            continue

        self._full_control_ticks = getattr(self, '_full_control_ticks', 0) + 1
        
        # --- Waypoint Management for MPC ---
        wp_list = (self.waypoints if hasattr(self, 'waypoints') else None)
        
        # Ensure position is available
        if not hasattr(self, 'position') or self.position is None:
            wait
            continue
        
        px = float(self.position.x); py = float(self.position.y)
        car_heading = None
        # Prefer dSPACE/ControlDesk readback heading if available
        if hasattr(self, 'dspaceActor') and self.dspaceActor is not None and hasattr(self.dspaceActor, 'heading') and self.dspaceActor.heading is not None:
            try:
                car_heading = float(self.dspaceActor.heading)
            except:
                pass
        if car_heading is None and hasattr(self, 'heading') and self.heading is not None:
            try:
                car_heading = float(self.heading)
            except:
                pass
        _bt = getattr(simulation(), 'behavior_timing', None)
        if _bt is not None:
            _bt.start_step()
            _bt.start_section('state_unpack')
        # Full control path (waypoint/profile/MPC); we only reach here when not on fast path
        if True:
            # Update waypoint index for MPC
            # PROGRESS-BASED ADVANCEMENT (from suggestion.md): Advance based on arc-length progress
            # Instead of radius-based advancement, track cumulative distance along waypoints
            # and advance when we've progressed past a waypoint segment
            old_wp_idx = wp_last_idx
            if use_waypoints and wp_list and len(wp_list) >= 2:
                if _bt is not None:
                    _bt.end_section('state_unpack')
                    _bt.start_section('path_progress')
                # When MPC rejects or association is bad, reset wp_last_idx to nearest waypoint (full scan) to avoid stale index and oscillations
                _md = getattr(_lat_controller, '_log_match_dist_m', None)
                _gr = getattr(_lat_controller, '_log_gate_reason', None)
                if (_md is not None and _md > 5.0) or _gr in ('too_far', 's_jump'):
                    nearest_idx = 0
                    best_d2 = 1e18
                    for i in range(len(wp_list)):
                        wx, wy = float(wp_list[i][0]), float(wp_list[i][1])
                        d2 = (px - wx)**2 + (py - wy)**2
                        if d2 < best_d2:
                            best_d2 = d2
                            nearest_idx = i
                    wp_last_idx = nearest_idx
                    # Invalidate cumulative-dist cache so it is recomputed from new wp_last_idx
                    self._cached_cumulative_dist_wp_idx = 0
                    self._cached_cumulative_dist_to_wp = 0.0
                # Build segment map once (main + pit): from OpenDRIVE track or from TTL waypoints (same as visualization).
                if not hasattr(self, '_waypoint_segment_map') or self._waypoint_segment_map is None:
                    try:
                        scene = simulation().scene
                        params = getattr(scene, 'params', None) or {}
                        main_ttl = getattr(scene, '_main_ttl_waypoints', None)
                        if main_ttl is not None:
                            pit_ttl = getattr(scene, '_pit_ttl_waypoints', None) or []
                            self._waypoint_segment_map = build_waypoint_segment_map_from_ttl(main_ttl, pit_ttl, waypoints=wp_list)
                            self._last_valid_segment_id = None
                            self._last_valid_segment_name = ""
                            num_seg = len(set(seg_id for seg_id, _ in self._waypoint_segment_map)) if self._waypoint_segment_map else 0
                            print(f"{_fbhv} Segment map built from TTL waypoints ({num_seg} segments, main+pit, overlap=main); ring-strict segment filtering active")
                        else:
                            track = params.get('track') or getattr(scene, 'track', None)
                            if track is not None:
                                self._waypoint_segment_map = build_waypoint_segment_map(wp_list, track)
                                self._last_valid_segment_id = None
                                self._last_valid_segment_name = ""
                                num_seg = len(set(seg_id for seg_id, _ in self._waypoint_segment_map)) if self._waypoint_segment_map else 0
                                print(f"{_fbhv} Segment map built from OpenDRIVE ({num_seg} segments, main+pit); ring-strict segment filtering active")
                            else:
                                self._waypoint_segment_map = None
                                print(f"{_fbhv} Segment map not built (no track and no TTL waypoints); log will show segment ?")
                    except Exception as e:
                        self._waypoint_segment_map = None
                        print(f"{_fbhv} Segment map not built: {e}; log will show segment ?")
                try:
                    # Initialize progress tracking if needed
                    if not hasattr(self, '_waypoint_progress'):
                        self._waypoint_progress = 0.0  # Cumulative distance along waypoints
                        self._waypoint_progress_idx = 0  # Waypoint index at last progress update
                    if not hasattr(self, '_cached_cumulative_dist_wp_idx'):
                        self._cached_cumulative_dist_wp_idx = 0
                        self._cached_cumulative_dist_to_wp = 0.0
                    # Cumulative distance to current waypoint index (O(1) update when wp_last_idx advances)
                    # Path is closed: segment i is (wp_list[i], wp_list[(i+1) % n]); wrap last -> first.
                    n_wp = len(wp_list)
                    cached_idx = getattr(self, '_cached_cumulative_dist_wp_idx', 0)
                    cached_dist = getattr(self, '_cached_cumulative_dist_to_wp', 0.0)
                    if wp_last_idx <= cached_idx:
                        if wp_last_idx < cached_idx:
                            cumulative_dist_to_wp = 0.0
                            for i in range(wp_last_idx):
                                wp0 = wp_list[i]
                                wp1 = wp_list[(i + 1) % n_wp]
                                dx = float(wp1[0]) - float(wp0[0])
                                dy = float(wp1[1]) - float(wp0[1])
                                seg_len = (dx*dx + dy*dy) ** 0.5
                                cumulative_dist_to_wp += seg_len
                            self._cached_cumulative_dist_to_wp = cumulative_dist_to_wp
                            self._cached_cumulative_dist_wp_idx = wp_last_idx
                        else:
                            cumulative_dist_to_wp = cached_dist
                    else:
                        cumulative_dist_to_wp = cached_dist
                        for i in range(cached_idx, wp_last_idx):
                            wp0 = wp_list[i]
                            wp1 = wp_list[(i + 1) % n_wp]
                            dx = float(wp1[0]) - float(wp0[0])
                            dy = float(wp1[1]) - float(wp0[1])
                            seg_len = (dx*dx + dy*dy) ** 0.5
                            cumulative_dist_to_wp += seg_len
                        self._cached_cumulative_dist_to_wp = cumulative_dist_to_wp
                        self._cached_cumulative_dist_wp_idx = wp_last_idx

                    # Project vehicle position onto current waypoint segment (closed loop: segment wp_last_idx -> (wp_last_idx+1) % n_wp)
                    s_0 = cumulative_dist_to_wp  # Default: progress to segment start
                    wp0 = wp_list[wp_last_idx]
                    wp1 = wp_list[(wp_last_idx + 1) % n_wp]
                    x0, y0 = float(wp0[0]), float(wp0[1])
                    x1, y1 = float(wp1[0]), float(wp1[1])
                    seg_dx = x1 - x0
                    seg_dy = y1 - y0
                    seg_len = (seg_dx*seg_dx + seg_dy*seg_dy) ** 0.5
                    
                    if seg_len > 1e-6:
                        # Project vehicle position onto segment
                        wx = px - x0
                        wy = py - y0
                        u_proj = (wx*seg_dx + wy*seg_dy) / (seg_len*seg_len)
                        u_proj = max(0.0, min(1.0, u_proj))
                        
                        # Current progress s_0 = cumulative distance to segment start + progress along segment
                        s_0 = cumulative_dist_to_wp + u_proj * seg_len
                        
                        # Advance waypoint index based on progress (wrap last -> first for closed loop)
                        while True:
                            segment_end_dist = cumulative_dist_to_wp + seg_len
                            if s_0 >= segment_end_dist - 0.5:  # Small threshold (0.5m) to handle numerical issues
                                wp_last_idx = (wp_last_idx + 1) % n_wp
                                cumulative_dist_to_wp = 0.0 if wp_last_idx == 0 else segment_end_dist
                                # Next segment
                                wp0 = wp_list[wp_last_idx]
                                wp1 = wp_list[(wp_last_idx + 1) % n_wp]
                                x0, y0 = float(wp0[0]), float(wp0[1])
                                x1, y1 = float(wp1[0]), float(wp1[1])
                                seg_dx = x1 - x0
                                seg_dy = y1 - y0
                                seg_len = (seg_dx*seg_dx + seg_dy*seg_dy) ** 0.5
                                if seg_len > 1e-6:
                                    wx = px - x0
                                    wy = py - y0
                                    u_proj = (wx*seg_dx + wy*seg_dy) / (seg_len*seg_len)
                                    u_proj = max(0.0, min(1.0, u_proj))
                                    s_0 = cumulative_dist_to_wp + u_proj * seg_len
                                else:
                                    seg_len = 1e-6
                            else:
                                break
                    else:
                        # Degenerate segment - advance to next (with wrap)
                        wp_last_idx = (wp_last_idx + 1) % n_wp
                        cumulative_dist_to_wp = 0.0 if wp_last_idx == 0 else (cumulative_dist_to_wp + seg_len)
                    # Keep cache in sync when wp_last_idx was advanced in the while loop or degenerate branch
                    self._cached_cumulative_dist_to_wp = cumulative_dist_to_wp
                    self._cached_cumulative_dist_wp_idx = wp_last_idx

                    # Update progress tracking
                    self._waypoint_progress = s_0
                    self._waypoint_progress_idx = wp_last_idx

                    # Calculate distance to current waypoint for logging
                    current_wp_dist = None
                    if wp_last_idx < len(wp_list):
                        wp_x, wp_y = float(wp_list[wp_last_idx][0]), float(wp_list[wp_last_idx][1])
                        dx = px - wp_x; dy = py - wp_y
                        current_wp_dist = (dx*dx + dy*dy) ** 0.5

                    # Log current waypoint
                    if wp_last_idx < len(wp_list):
                        current_wp = wp_list[wp_last_idx]
                        current_wp_x, current_wp_y = float(current_wp[0]), float(current_wp[1])
                        if wp_last_idx != old_wp_idx:
                            # Initialize waypoint progress tracking
                            if not hasattr(self, '_waypoints_passed'):
                                self._waypoints_passed = 0
                            self._waypoints_passed += 1
                            progress_pct = ((self._waypoints_passed % len(wp_list)) / len(wp_list)) * 100.0 if len(wp_list) > 0 else 0.0
                            sim = simulation()
                            ctrl_dt = getattr(sim, 'control_dt', None)
                            if ctrl_dt is None or ctrl_dt <= 0:
                                ctrl_dt = getattr(sim, 'control_period', None)
                            if ctrl_dt is None or ctrl_dt <= 0:
                                ctrl_dt = float(getattr(sim, 'timestep', 0.05))
                            step_wp = getattr(self, '_behavior_step_count', 0)
                            t_wp = step_wp * ctrl_dt
                            _smap_w = getattr(self, '_waypoint_segment_map', None)
                            _raw_w = get_segment_at_waypoint(wp_last_idx, _smap_w) if _smap_w else None
                            _path_w = "pit" if (_raw_w and _raw_w[1] and _raw_w[1].startswith("pit ")) else "main"
                            _lid_w = getattr(self, '_last_valid_segment_id', None)
                            _lname_w = getattr(self, '_last_valid_segment_name', "") or ""
                            _eid, _ename, _ = get_segment_at_waypoint_ring_strict(wp_last_idx, _smap_w, _path_w, _lid_w, _lname_w)
                            self._last_valid_segment_id = _eid
                            self._last_valid_segment_name = _ename or ""
                            seg_str = f" {get_segment_label(_eid, _ename)}" if (_eid is not None or _ename) else " segment ?"
                            print(f"{_fbhv} t={t_wp:.2f}s WAYPOINT HIT: index {old_wp_idx} -> {wp_last_idx} at ({current_wp_x:.2f}, {current_wp_y:.2f}), distance={current_wp_dist:.2f}m{seg_str}")
                            print(f"{_fbhv} t={t_wp:.2f}s Progress: {self._waypoints_passed} waypoints passed ({progress_pct:.1f}% of {len(wp_list)} total waypoints)")

                except Exception as e:
                    print(f"{_fbhv} Warning: Waypoint finder error: {e}")
                if _bt is not None:
                    _bt.end_section('path_progress')
            
            # --- Longitudinal Control (MPC) ---
            # To-Do D: Use projection-based CTE (same geometry as MPCC) everywhere. CTE magnitude for speed
            # comes from previous step's waypoint-based e_y (MPCC uses same polyline and projection).
            cte_mag_for_speed = getattr(self, '_last_waypoint_cte_for_speed', 0.0)
            # Legacy CTE: fallback when MPC does not provide e_y (e.g. no waypoints or exception)
            self._legacy_cte_this_tick = None
            if use_waypoints and wp_list and len(wp_list) >= 2:
                try:
                    nearest_idx = wp_last_idx
                    best_d2 = 1e18
                    search_window = 50
                    for i in range(max(0, wp_last_idx - search_window), min(len(wp_list), wp_last_idx + search_window)):
                        wx, wy = float(wp_list[i][0]), float(wp_list[i][1])
                        d2 = (px - wx)**2 + (py - wy)**2
                        if d2 < best_d2:
                            best_d2 = d2
                            nearest_idx = i
                    if abs(nearest_idx - wp_last_idx) > search_window * 0.8:
                        for i in range(max(0, wp_last_idx - 2*search_window), min(len(wp_list), wp_last_idx + 2*search_window)):
                            wx, wy = float(wp_list[i][0]), float(wp_list[i][1])
                            d2 = (px - wx)**2 + (py - wy)**2
                            if d2 < best_d2:
                                best_d2 = d2
                                nearest_idx = i
                    next_idx = (nearest_idx + 1) % len(wp_list)
                    if len(wp_list) >= 2:
                        x0, y0 = float(wp_list[nearest_idx][0]), float(wp_list[nearest_idx][1])
                        x1, y1 = float(wp_list[next_idx][0]), float(wp_list[next_idx][1])
                        seg_dx = x1 - x0
                        seg_dy = y1 - y0
                        seg_len = (seg_dx*seg_dx + seg_dy*seg_dy) ** 0.5
                        if seg_len > 1e-6:
                            u_proj = max(0.0, min(1.0, ((px - x0)*seg_dx + (py - y0)*seg_dy) / (seg_len*seg_len)))
                            proj_x = x0 + u_proj * seg_dx
                            proj_y = y0 + u_proj * seg_dy
                            nx = -seg_dy / seg_len
                            ny = seg_dx / seg_len
                            if car_heading is not None:
                                seg_heading = math.atan2(seg_dy, seg_dx)
                                hd = math.atan2(math.sin(seg_heading - car_heading), math.cos(seg_heading - car_heading))
                                if abs(hd) > math.pi / 2:
                                    nx, ny = -nx, -ny
                            self._legacy_cte_this_tick = (px - proj_x)*nx + (py - proj_y)*ny
                except Exception:
                    pass
            if self._legacy_cte_this_tick is None and hasattr(self, 'ttl') and self.ttl is not None and hasattr(self.ttl, 'signedDistanceTo'):
                try:
                    self._legacy_cte_this_tick = self.ttl.signedDistanceTo(self.position)
                except Exception:
                    self._legacy_cte_this_tick = 0.0
            if self._legacy_cte_this_tick is None:
                self._legacy_cte_this_tick = 0.0
            if _bt is not None:
                _bt.start_section('waypoint_speed_grade')
            # Universal max speed limit: Reduced for better robustness without elevation data
            MAX_SPEED_LIMIT_MS = 62.58  # 140 mph in m/s (~225 km/h) for IAC vehicle capability
            PIT_MAX_SPEED_MS = 15.646   # 35 mph in m/s for pit lane (cap only; we coast in pit)
            # Pit lane: coast at 35 mph; only add throttle when "too slow" to avoid brake/throttle oscillation
            PIT_COAST_TOO_SLOW_MS = 10.0   # below this speed we add a bit of throttle
            PIT_COAST_MIN_TARGET_MS = 15.646  # 35 mph — pit target and coast threshold (match PIT_MAX_SPEED_MS)
            # Segment (and thus pit vs main): use raw segment at waypoint so we never rely on ModelDesk route for control.
            _smap = getattr(self, '_waypoint_segment_map', None)
            _raw_seg = get_segment_at_waypoint(wp_last_idx, _smap) if _smap else None
            _seg_name_raw = (_raw_seg[1] or "") if _raw_seg else ""
            in_pit_by_segment = _seg_name_raw.startswith("pit ")
            _path = "pit" if in_pit_by_segment else "main"
            _lid = getattr(self, '_last_valid_segment_id', None)
            _lname = getattr(self, '_last_valid_segment_name', "") or ""
            _eff_id, _eff_name, _transition = get_segment_at_waypoint_ring_strict(wp_last_idx, _smap, _path, _lid, _lname)
            self._last_valid_segment_id = _eff_id
            self._last_valid_segment_name = _eff_name or ""
            # Pit exit/enter: update ModelDesk route and _route (for logging/transition only); control uses segment, not route
            if _transition == "pit_exit" and self is simulation().scene.egoObject:
                self._route = RACING_MODE_MAIN
                if hasattr(simulation(), "request_ego_route"):
                    simulation().request_ego_route(self, RACING_MODE_MAIN)
            elif _transition == "pit_enter" and self is simulation().scene.egoObject:
                self._route = RACING_MODE_PIT
                if hasattr(simulation(), "request_ego_route"):
                    simulation().request_ego_route(self, RACING_MODE_PIT)
            segment_name_current = _eff_name
            # Pit mode for speed limit and throttle: segment-based only (pit road = pit limit; main = no limit)
            pit_mode = in_pit_by_segment
            self._in_pit_by_segment = pit_mode
            
            # --- Curvature-based speed gate (from suggestion.md) ---
            # Formula: v_max(s) = sqrt(a_y_max / (|κ(s)| + ε))
            # v_ref = min(v_desired, min_{s∈[s_0, s_0+L]} v_max(s))
            # This ensures vehicle enters turns at appropriate speed (Laguna Seca: see turns early to avoid run-off)
            curvature_speed_limit = target_speed  # Default: no reduction
            curvature_ahead_max = 0.0  # Max curvature magnitude over lookahead (for speed limits)
            curvature_ahead_max_signed = 0.0  # Task 3: signed kappa at apex (left > 0, right < 0) for downstream
            max_lateral_accel = 8.0  # m/s² (conservative for indoor sim, can be configured)
            curvature_epsilon = 0.001  # Small epsilon to avoid division by zero
            curvature_speed_margin = 0.92  # Use 92% of theoretical v_max (generic: less over-braking, less throttle compensation; works for any TTL)
            
            if use_waypoints and wp_list and len(wp_list) >= 3:
                try:
                    # Compute curvature-based speed limit over MPC horizon (waypoint indices wrap at end of lap)
                    # Look ahead: ensure we see turns early enough to brake (Laguna Seca Corkscrew/hairpins)
                    horizon = _lon_controller.config.mpc_prediction_horizon if hasattr(_lon_controller, 'config') else 35
                    dt_mpc = _lon_controller.config.mpc_prediction_dt if hasattr(_lon_controller, 'config') else 0.05
                    lookahead_dist = current_speed * horizon * dt_mpc  # Distance over MPC horizon
                    if lookahead_dist < 10.0:
                        lookahead_dist = 25.0  # Minimum at very low speed
                    # At high speed, need enough distance to brake before turn (avoid run-off).
                    # Braking from v to v_turn at slew_down m/s takes (v - v_turn)/slew_down seconds; distance ~ v * T.
                    # At 46 m/s, 7 m/s/s slew: need ~5.5 s -> ~250 m to see turn and slow to ~8 m/s for sharp bend.
                    min_lookahead_for_braking = 85.0   # m when 15 < speed <= 25
                    if current_speed > 40.0:
                        min_lookahead_for_braking = 250.0  # m at very high speed: see sharp turn in time (e.g. k=0.1) without capping max speed
                    elif current_speed > 25.0:
                        min_lookahead_for_braking = 120.0  # m at high speed
                    if lookahead_dist < min_lookahead_for_braking and current_speed > 15.0:
                        lookahead_dist = min_lookahead_for_braking
                    # When speed is high, use at least 120 m lookahead so sharp bends (e.g. segment 43) are fully seen and we slow in time.
                    if current_speed > 20.0 and lookahead_dist < 120.0:
                        lookahead_dist = max(lookahead_dist, 120.0)
    
                    lookahead_idx = wp_last_idx
                    accumulated_dist = 0.0
                    min_v_max = target_speed  # Track minimum v_max over horizon
                    n_wp = len(wp_list)
                    
                    # Sample curvature along horizon; wrap waypoints so near end-of-lap we see the straight after the loop
                    sample_points = []
                    while accumulated_dist < lookahead_dist:
                        next_idx = (lookahead_idx + 1) % n_wp
                        x0, y0 = float(wp_list[lookahead_idx][0]), float(wp_list[lookahead_idx][1])
                        x1, y1 = float(wp_list[next_idx][0]), float(wp_list[next_idx][1])
                        seg_dx = x1 - x0; seg_dy = y1 - y0
                        seg_len = (seg_dx*seg_dx + seg_dy*seg_dy) ** 0.5
                        if seg_len < 1e-6:
                            lookahead_idx = next_idx
                            continue
                        sample_points.append((lookahead_idx, accumulated_dist))
                        accumulated_dist += seg_len
                        lookahead_idx = next_idx
                        if lookahead_idx == wp_last_idx and len(sample_points) > 1:
                            break  # wrapped full lap
                    
                    # Compute curvature at each sample point (use modulo so indices 0 and n_wp-1 are valid)
                    for sample_idx, sample_dist in sample_points:
                        i0 = (sample_idx - 1) % n_wp
                        i1 = sample_idx % n_wp
                        i2 = (sample_idx + 1) % n_wp
                        p0 = (float(wp_list[i0][0]), float(wp_list[i0][1]))
                        p1 = (float(wp_list[i1][0]), float(wp_list[i1][1]))
                        p2 = (float(wp_list[i2][0]), float(wp_list[i2][1]))
                        # Compute curvature (3-point method)
                        v1x = p1[0] - p0[0]; v1y = p1[1] - p0[1]
                        v2x = p2[0] - p1[0]; v2y = p2[1] - p1[1]
                        cross = v1x * v2y - v1y * v2x
                        len1 = (v1x*v1x + v1y*v1y) ** 0.5
                        len2 = (v2x*v2x + v2y*v2y) ** 0.5
                        if len1 > 1e-6 and len2 > 1e-6:
                            avg_len = (len1 + len2) / 2.0
                            if avg_len > 1e-6:
                                # Task 3: keep signed curvature (cross2D: left turn > 0, right turn < 0); magnitude for speed formulas
                                kappa_signed = 2.0 * cross / (len1 * len2 * avg_len)
                                abs_kappa = abs(kappa_signed)
                                if abs_kappa > curvature_ahead_max:
                                    curvature_ahead_max = abs_kappa
                                    curvature_ahead_max_signed = kappa_signed  # apex signed kappa
                                # Apply speed gate formula: v_max = sqrt(a_y_max / (|κ| + ε)); apply safety margin
                                v_max_at_kappa = curvature_speed_margin * (max_lateral_accel / (abs_kappa + curvature_epsilon)) ** 0.5
                                if v_max_at_kappa < min_v_max:
                                    min_v_max = v_max_at_kappa
                    
                    # Expose signed curvature at apex for downstream (commit/approach logic)
                    self._curvature_ahead_max_signed = curvature_ahead_max_signed
                    # Apply minimum v_max over horizon
                    curvature_speed_limit = min_v_max
                    # Slow-in for sharp turns: when any significant curvature is ahead, cap speed more aggressively
                    # so we are already slow when the turn tightens (avoids "too fast, didn't turn in time").
                    # Very sharp turns (κ > 0.08): use 74% margin to reduce high CTE (e.g. segment 43 run).
                    if curvature_ahead_max > 0.015:
                        if curvature_ahead_max > 0.08:
                            slow_in_margin = 0.74   # Very sharp: stricter so we enter segment 43-type curves slower
                        elif curvature_ahead_max > 0.05:
                            slow_in_margin = 0.82
                        else:
                            slow_in_margin = 0.88
                        v_max_slow_in = slow_in_margin * (max_lateral_accel / (curvature_ahead_max + curvature_epsilon)) ** 0.5
                        if v_max_slow_in < curvature_speed_limit:
                            curvature_speed_limit = v_max_slow_in
                    curvature_speed_cap = curvature_speed_limit  # Task 4: single curvature-based speed cap
                except Exception as e:
                    curvature_speed_cap = target_speed
                    pass
            else:
                curvature_speed_cap = target_speed
            
            # --- CTE-based speed reduction (for MPC speed reference) ---
            # Generic: no TTL or segment IDs; only |CTE| so we slow when off-line on any track.
            # When CTE is large, cap target speed so the car can recover instead of running off.
            if cte_mag_for_speed >= 10.0:
                # 10m+ CTE: strong decel and hard cap so we don't maintain high speed off-track
                cte_target_speed = min(3.0, max(0.0, current_speed - 4.0))
            elif cte_mag_for_speed >= 5.0:
                # 5-10m CTE: cap speed (was current_speed -> run-off). Cap at 5 m/s so MPC brakes.
                cte_target_speed = min(5.0, current_speed)
            elif cte_mag_for_speed >= 3.0:
                # 3-5m CTE: limit to 5 m/s (earlier recovery, any track)
                cte_target_speed = 5.0
            elif cte_mag_for_speed >= 2.0:
                # 2-3m CTE: limit to 6 m/s
                cte_target_speed = 6.0
            elif cte_mag_for_speed >= 1.5:
                # 1.5-2m CTE: limit to 6 m/s (early intervention)
                cte_target_speed = 6.0
            elif cte_mag_for_speed >= 1.0:
                # 1.0-1.5m CTE: limit to 7 m/s
                cte_target_speed = 7.0
            elif cte_mag_for_speed >= 0.5:
                # 0.5-1.0m CTE: limit to 8 m/s
                cte_target_speed = 8.0
            elif cte_mag_for_speed >= cte_stop_threshold:
                # At 50m+ CTE: aim for very low speed (encourages heavy braking)
                cte_target_speed = target_speed * 0.1
            elif cte_mag_for_speed >= cte_slowdown_threshold:
                # Between 15-50m: aim for 30% of target speed (encourages braking)
                factor = 0.3
                cte_target_speed = target_speed * factor
            elif cte_mag_for_speed >= cte_throttle_reduction_max:
                # Between 10-15m: linear from 50% to 30% of target speed
                factor = 0.5 - ((cte_mag_for_speed - cte_throttle_reduction_max) / (cte_slowdown_threshold - cte_throttle_reduction_max)) * 0.2
                cte_target_speed = target_speed * factor
            else:
                # CTE < 0.5m: use full target speed (but still respect max speed limit)
                cte_target_speed = target_speed
            
            # --- Combine CTE and curvature speed limits (take minimum) ---
            # Task 4: curvature-based speed cap applied here; use single cap from lookahead curvature
            effective_target_speed = min(cte_target_speed, curvature_speed_cap)
            
            # Apply universal max speed limit
            effective_target_speed = min(effective_target_speed, MAX_SPEED_LIMIT_MS)
            # Pit mode (racing library): fixed coast target, no slew — drive differently than main
            if pit_mode:
                if current_speed < PIT_COAST_TOO_SLOW_MS:
                    effective_target_speed = min(PIT_MAX_SPEED_MS, PIT_COAST_MIN_TARGET_MS, curvature_speed_cap)
                else:
                    effective_target_speed = min(PIT_COAST_MIN_TARGET_MS, PIT_MAX_SPEED_MS, curvature_speed_cap)
                self._last_effective_target_speed = float(effective_target_speed)
            else:
                # --- Slew-rate limit on speed reference (smooth ramps into/out of turns) ---
                # When CTE is large, allow faster slew-down so the CTE speed cap takes effect quickly (generic run-off fix).
                dt_slew = _lon_controller.config.mpc_prediction_dt if hasattr(_lon_controller, 'config') else 0.05
                slew_down_ms = 12.0 if cte_mag_for_speed >= 3.0 else 7.0   # faster ramp-down when off-line
                slew_up_ms = 5.0     # max speed increase per second
                if not hasattr(self, '_last_effective_target_speed'):
                    self._last_effective_target_speed = float(effective_target_speed)
                last_eff = float(self._last_effective_target_speed)
                effective_target_speed = max(last_eff - slew_down_ms * dt_slew, min(last_eff + slew_up_ms * dt_slew, float(effective_target_speed)))
                self._last_effective_target_speed = float(effective_target_speed)
            
            # --- Build speed reference profile for MPC ---
            horizon = _lon_controller.config.mpc_prediction_horizon
            # Pit mode: constant profile at coast target (no curvature lookahead)
            if pit_mode:
                v_ref_profile = [float(effective_target_speed)] * horizon
                v_ref_profile = [min(v, PIT_MAX_SPEED_MS) for v in v_ref_profile]
            else:
                # Main mode: profile from effective_target_speed, then reduce for upcoming turns
                v_ref_profile = [float(effective_target_speed)] * horizon
            # If we have waypoints (main mode only; pit already has constant profile), build a speed profile that reduces speed for upcoming turns (cached + vectorized)
            if not pit_mode and use_waypoints and wp_list and len(wp_list) >= 2:
                # Stable cache key (shape + first/last waypoint) so cache hits when racing line is unchanged
                wp_cache_key = (len(wp_list), (float(wp_list[0][0]), float(wp_list[0][1])), (float(wp_list[-1][0]), float(wp_list[-1][1])))
                self._wp_cache_tick = getattr(self, '_wp_cache_tick', 0) + 1
                # Build/reuse curvature cache (2D cumdist + per-waypoint curve_vmax)
                if getattr(self, '_wp_curve_cache_key', None) != wp_cache_key:
                    nwp = len(wp_list)
                    if nwp >= 2:
                        # Closed-loop segment lengths (including last->first)
                        seg_len_xy = np.zeros(nwp, dtype=np.float64)
                        for i in range(nwp):
                            j = (i + 1) % nwp
                            x0, y0 = float(wp_list[i][0]), float(wp_list[i][1])
                            x1, y1 = float(wp_list[j][0]), float(wp_list[j][1])
                            dx, dy = x1 - x0, y1 - y0
                            seg_len_xy[i] = max((dx*dx + dy*dy) ** 0.5, 1e-9)
                        cum = np.zeros(nwp + 1, dtype=np.float64)
                        cum[1:] = np.cumsum(seg_len_xy)
                        self._wp_seg_len_xy = seg_len_xy
                        self._wp_cumdist_xy = cum
                        self._wp_total_len_xy = float(cum[-1])
                        # Curvature vmax per waypoint (closed-loop: modulo neighbors)
                        curve_vmax_list = []
                        for i in range(nwp):
                            i0 = (i - 1) % nwp
                            i1 = i
                            i2 = (i + 1) % nwp
                            p0 = (float(wp_list[i0][0]), float(wp_list[i0][1]))
                            p1 = (float(wp_list[i1][0]), float(wp_list[i1][1]))
                            p2 = (float(wp_list[i2][0]), float(wp_list[i2][1]))
                            v1x = p1[0] - p0[0]; v1y = p1[1] - p0[1]
                            v2x = p2[0] - p1[0]; v2y = p2[1] - p1[1]
                            cross = v1x * v2y - v1y * v2x
                            len1 = (v1x*v1x + v1y*v1y) ** 0.5
                            len2 = (v2x*v2x + v2y*v2y) ** 0.5
                            if len1 > 1e-6 and len2 > 1e-6:
                                avg_len = (len1 + len2) / 2.0
                                abs_kappa = abs(2.0 * cross / (len1 * len2 * avg_len))
                                v_max_at_kappa = curvature_speed_margin * (max_lateral_accel / (abs_kappa + curvature_epsilon)) ** 0.5
                                curve_vmax_list.append(v_max_at_kappa)
                            else:
                                curve_vmax_list.append(1e6)
                        self._wp_curve_vmax = np.asarray(curve_vmax_list, dtype=np.float64)
                        self._wp_curve_cache_key = wp_cache_key
                        self._wp_curve_cache_rebuilds = getattr(self, '_wp_curve_cache_rebuilds', 0) + 1
                else:
                    self._wp_curve_cache_hits = getattr(self, '_wp_curve_cache_hits', 0) + 1
                try:
                    if getattr(self, '_wp_curve_vmax', None) is not None:
                        # Vectorized: dist_vec, modulo lookahead for closed loop, gather curve_vmax, apply min
                        dt = _lon_controller.config.mpc_prediction_dt
                        dist_vec = current_speed * (np.arange(1, horizon + 1, dtype=np.float64)) * dt
                        nwp = len(wp_list)
                        base_idx = int(wp_last_idx) % nwp
                        base_cum = self._wp_cumdist_xy[base_idx]
                        L = self._wp_total_len_xy
                        target_abs = base_cum + dist_vec
                        target_mod = np.mod(target_abs, L)
                        seg_idx = np.searchsorted(self._wp_cumdist_xy, target_mod, side='right') - 1
                        seg_idx = np.clip(seg_idx, 0, nwp - 1)
                        cap_profile = self._wp_curve_vmax[seg_idx]
                        v_ref_profile = np.minimum(np.asarray(v_ref_profile, dtype=np.float64), cap_profile).tolist()
                except Exception as e:
                    # If profile building fails, use constant speed
                    pass
            
            # Task 4: enforce curvature-based speed cap on entire profile (main mode; pit already constant)
            if not pit_mode:
                v_ref_profile = [min(v, curvature_speed_cap) for v in v_ref_profile]
            # Apply max speed limit to profile (pit mode: already capped above; main: 140 mph cap)
            if not pit_mode:
                v_ref_profile = [min(v, MAX_SPEED_LIMIT_MS) for v in v_ref_profile]
            
            # --- Build grade profile for longitudinal MPC (if 3D waypoints available) ---
            grade_profile = None
            if use_waypoints and wp_list and len(wp_list) >= 2:
                # --- Build/reuse waypoint geometry cache --- (same stable key as curvature block)
                wp_cache_key = (len(wp_list), (float(wp_list[0][0]), float(wp_list[0][1])), (float(wp_list[-1][0]), float(wp_list[-1][1])))
                if getattr(self, '_wp_grade_cache_key', None) != wp_cache_key:
                    nwp = len(wp_list)
                    if nwp >= 2:
                        # Closed-loop grade segments (including last->first)
                        seg_len_3d = []
                        seg_grade = []
                        for i in range(nwp):
                            j = (i + 1) % nwp
                            wp0 = wp_list[i]
                            wp1 = wp_list[j]
                            x0, y0 = float(wp0[0]), float(wp0[1])
                            x1, y1 = float(wp1[0]), float(wp1[1])
                            z0 = float(wp0[2]) if len(wp0) >= 3 else 0.0
                            z1 = float(wp1[2]) if len(wp1) >= 3 else 0.0

                            dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
                            L3 = (dx*dx + dy*dy + dz*dz) ** 0.5
                            Lxy = (dx*dx + dy*dy) ** 0.5

                            seg_len_3d.append(max(L3, 1e-9))
                            seg_grade.append(math.atan2(dz, Lxy) if Lxy > 1e-6 else 0.0)

                        self._wp_seg_len_3d = np.asarray(seg_len_3d, dtype=np.float64)
                        self._wp_seg_grade = np.asarray(seg_grade, dtype=np.float64)

                        cum = np.zeros(nwp + 1, dtype=np.float64)
                        cum[1:] = np.cumsum(self._wp_seg_len_3d)
                        self._wp_cumdist_3d = cum
                        self._wp_total_len_3d = float(cum[-1])

                        self._wp_grade_cache_key = wp_cache_key
                        self._wp_grade_cache_rebuilds = getattr(self, '_wp_grade_cache_rebuilds', 0) + 1
                else:
                    self._wp_grade_cache_hits = getattr(self, '_wp_grade_cache_hits', 0) + 1
                if getattr(self, '_wp_cache_tick', 0) % 100 == 0:
                    ch = getattr(self, '_wp_curve_cache_hits', 0); cr = getattr(self, '_wp_curve_cache_rebuilds', 0)
                    gh = getattr(self, '_wp_grade_cache_hits', 0); gr = getattr(self, '_wp_grade_cache_rebuilds', 0)
                    print(f"[Waypoint cache] tick={getattr(self, '_wp_cache_tick', 0)}  curve: hits={ch} rebuilds={cr}  grade: hits={gh} rebuilds={gr}")
                if len(wp_list[0]) >= 3:
                    horizon = _lon_controller.config.mpc_prediction_horizon
                    dt_mpc = _lon_controller.config.mpc_prediction_dt

                    dist_vec = current_speed * (np.arange(1, horizon + 1, dtype=np.float64)) * dt_mpc

                    nwp = len(wp_list)
                    base_idx = int(wp_last_idx) % nwp
                    base_cum = self._wp_cumdist_3d[base_idx]
                    L = self._wp_total_len_3d

                    target_abs = base_cum + dist_vec
                    target_mod = np.mod(target_abs, L)

                    seg_idx = np.searchsorted(self._wp_cumdist_3d, target_mod, side='right') - 1
                    seg_idx = np.clip(seg_idx, 0, nwp - 1)

                    grade_profile = self._wp_seg_grade[seg_idx].astype(np.float64, copy=False)
            if _bt is not None:
                _bt.end_section('waypoint_speed_grade')
            
            # --- Use MPC for throttle/brake control ---
            # Get current acceleration (estimate from speed if not available)
            dt = getattr(simulation(), 'control_dt', None)
            if dt is None:
                dt = simulation().timestep * max(1, getattr(simulation(), '_control_interval', 1))
            current_accel = 0.0
            if hasattr(self, '_prev_speed_mpc'):
                current_accel = (current_speed - self._prev_speed_mpc) / dt
                current_accel = max(-15.0, min(20.0, current_accel))  # Clamp to reasonable range
            self._prev_speed_mpc = current_speed
            
            # Build vehicle state for MPC
            vehicle_state_mpc = {
                'speed': current_speed,
                'acceleration': current_accel,
                'gear': getattr(self, 'gear', 0) if manage_gears and hasattr(self, 'setGear') else None
            }
            
            # Compute throttle/brake using MPC (with grade compensation if available)
            try:
                throttle_mpc, brake_mpc = _lon_controller.run_step(
                    vehicle_state_mpc,
                    v_ref_profile,
                    None,  # curvature_profile not used in simplified model
                    grade_profile  # Road grade profile for gravity compensation
                )
                throttle_mpc = float(throttle_mpc)
                brake_mpc = float(brake_mpc)
            except Exception as ex:
                print(f"{_fbhv} MPC longitudinal error: {ex}, using fallback")
                # Fallback: simple proportional control
                speed_error = effective_target_speed - current_speed
                if speed_error > 0:
                    throttle_mpc = min(1.0, speed_error * 0.1)
                    brake_mpc = 0.0
                else:
                    throttle_mpc = 0.0
                    brake_mpc = min(1.0, abs(speed_error) * 0.1)
            
            # Use MPC outputs (will be processed by CTE-aware safety envelope below)
            throttle_pid = throttle_mpc  # Keep variable name for compatibility with existing code
            
            # -------------------------------
            # CTE-aware longitudinal safety
            # -------------------------------
    
            # Hysteresis state tracking
            if not hasattr(self, '_was_cte_large'):
                self._was_cte_large = False
    
            _sim_cte = simulation()
            _is_non_ego_fellow = (
                getattr(_sim_cte.scene, 'egoObject', None) is not None
                and self is not _sim_cte.scene.egoObject
            )
            # Fellows (v,d plant): heavy CTE brake kills speed -> weak bicycle yaw -> lateral MPC cannot recover
            _cte_enter_large = 7.5 if _is_non_ego_fellow else 4.5
            _cte_exit_large = 6.5 if _is_non_ego_fellow else 3.5
            _cte_very_large_m = 14.0 if _is_non_ego_fellow else 10.0
    
            if cte_mag_for_speed >= _cte_enter_large:
                self._was_cte_large = True
            elif cte_mag_for_speed < _cte_exit_large:
                self._was_cte_large = False
    
            SPEED_THRESHOLD_FOR_BRAKE = 2.0
            MIN_THROTTLE_WHEN_STOPPED = 0.10
    
            # Output knobs (generic: no TTL/segment; avoid run-off by braking sooner when |CTE| is large)
            MAX_BRAKE_NORMAL = 0.25        # MPC brake cap when CTE is small
            MAX_BRAKE_LARGE_CTE = 0.65     # 4.5-10m CTE: allow stronger brake to recover
            MAX_BRAKE_VERY_LARGE = 0.90    # 10m+ CTE: strong brake, still not instant 1.0
            BRAKE_SLEW = 0.15              # per step
    
            cte_brake = 0.0
            throttle_override = None
    
            very_large = (cte_mag_for_speed >= _cte_very_large_m)
            large = (self._was_cte_large or cte_mag_for_speed >= _cte_enter_large)
    
            if very_large:
                # Far off-track: strong brake so we don't maintain speed (generic fix for run-off)
                if _is_non_ego_fellow:
                    if current_speed > 4.0:
                        throttle_override = 0.0
                        cte_brake = 0.28
                    elif current_speed > SPEED_THRESHOLD_FOR_BRAKE:
                        throttle_override = 0.0
                        cte_brake = 0.18
                    else:
                        throttle_override = MIN_THROTTLE_WHEN_STOPPED
                        cte_brake = 0.0
                elif current_speed > 4.0:
                    throttle_override = 0.0
                    cte_brake = 0.50
                elif current_speed > SPEED_THRESHOLD_FOR_BRAKE:
                    throttle_override = 0.0
                    cte_brake = 0.30
                else:
                    throttle_override = MIN_THROTTLE_WHEN_STOPPED
                    cte_brake = 0.0
            elif large:
                # 4.5-10m CTE (ego): meaningful brake at higher speed. Fellow: lighter so MPC keeps authority.
                if _is_non_ego_fellow:
                    if current_speed > 8.0:
                        throttle_override = None
                        cte_brake = 0.06
                    elif current_speed > 5.0:
                        throttle_override = None
                        cte_brake = 0.04
                    elif current_speed > 3.0:
                        throttle_override = None
                        cte_brake = 0.02
                    else:
                        throttle_override = None
                        cte_brake = 0.0
                elif current_speed > 8.0:
                    throttle_override = 0.0
                    cte_brake = 0.25
                elif current_speed > 5.0:
                    throttle_override = 0.0
                    cte_brake = 0.15
                elif current_speed > 3.0:
                    throttle_override = 0.0
                    cte_brake = 0.05
                else:
                    throttle_override = None
                    cte_brake = 0.0
    
            # Apply throttle override if set
            if throttle_override is not None:
                throttle_mpc = throttle_override
    
            # -------------------------------
            # Brake cap + merge + SLEW-LIMIT (drop-in replacement)
            # -------------------------------
    
            # Compute a brake cap depending on CTE regime
            if very_large:
                brake_cap = MAX_BRAKE_VERY_LARGE
            elif large:
                brake_cap = MAX_BRAKE_LARGE_CTE
            else:
                brake_cap = MAX_BRAKE_NORMAL
    
            # Merge CTE brake + MPC brake (both capped)
            raw_brake = max(float(cte_brake), float(brake_mpc))
            raw_brake = min(float(raw_brake), float(brake_cap))
    
            # ---- Guard: do not "slam brake" at (near) standstill ----
            STOP_SPEED = 0.6  # m/s (tune: 0.3~1.0)
            if current_speed <= STOP_SPEED:
                raw_brake = 0.0
    
            # ---- Slew-limit brake to avoid spikes (apply slower, release faster) ----
            BRAKE_SLEW_UP = 0.12     # max increase per step (0..1 scale)
            BRAKE_SLEW_DOWN = 0.20   # max decrease per step
    
            if not hasattr(self, "_last_raw_brake"):
                self._last_raw_brake = float(raw_brake)
    
            prev_brake = float(self._last_raw_brake)
            db = float(raw_brake) - prev_brake
            limited = False
    
            if db > BRAKE_SLEW_UP:
                raw_brake = prev_brake + BRAKE_SLEW_UP
                limited = True
            elif db < -BRAKE_SLEW_DOWN:
                raw_brake = prev_brake - BRAKE_SLEW_DOWN
                limited = True
    
            raw_brake = max(0.0, min(float(brake_cap), float(raw_brake)))
            self._last_raw_brake = float(raw_brake)
            final_brake = max(0.0, min(1.0, raw_brake))
    
            # --- Lateral Control (MPC) ---
            sim = simulation()
            is_fellow_mpc_virt = (
                getattr(sim.scene, "egoObject", None) is not None
                and self is not sim.scene.egoObject
                and getattr(sim, "mpc_config", None) is not None
            )
            _ctrl_dt_virt = getattr(sim, "control_dt", None)
            if _ctrl_dt_virt is None or _ctrl_dt_virt <= 0:
                _ctrl_dt_virt = float(sim.timestep) * max(1, getattr(sim, "_control_interval", 1))

            # Build vehicle state for MPC (assembly + read_state_from_controldesk)
            vehicle_state = {
                'x': px,
                'y': py,
                'yaw': car_heading if car_heading is not None else 0.0,
                'speed': current_speed,
            }

            # Scenic: bare names / importlib -> self.* ; use builtin __import__ only.
            if is_fellow_mpc_virt:
                __import__(
                    "scenic.domains.racing.mpc.fellow_virtual_mpc_state",
                    fromlist=["fellow_virt_prepare_for_scenic"],
                ).fellow_virt_prepare_for_scenic(
                    self,
                    vehicle_state,
                    wp_list,
                    use_waypoints,
                    wp_last_idx,
                    car_heading,
                )
    
            # Add gear information for MPC (check before gear change logic)
            if manage_gears and hasattr(self, 'setGear'):
                current_gear = getattr(self, 'gear', 0)
                vehicle_state['gear'] = current_gear
    
            # Add optional yaw_rate if available (fellow: plant rate often not meaningful for MPC)
            if not is_fellow_mpc_virt and hasattr(self, 'angularVelocity') and self.angularVelocity is not None:
                try:
                    vehicle_state['yaw_rate'] = float(self.angularVelocity.z) if hasattr(self.angularVelocity, 'z') else 0.0
                except Exception:
                    pass
    
            # Ego: steering feedback from ControlDesk. Fellow: virtual steer (no Vesi steer path).
            if not is_fellow_mpc_virt and hasattr(sim, 'mpc_config') and sim.mpc_config:
                from scenic.domains.racing.mpc.io_adapter import read_state_from_controldesk
                try:
                    cd_state = read_state_from_controldesk(sim, self)
                    if 'steer_actual' in cd_state:
                        vehicle_state['steer_actual'] = cd_state['steer_actual']   # match the same sign convention as command
    
                except Exception:
                    # If reading fails, MPC will use previous state estimate
                    pass
    
            # Convert waypoints to list of tuples for MPC (preserve 3D if available); cache to avoid rebuild every tick
            _wp_src = wp_list
            _wp_src_id = id(_wp_src)
            _wp_len = len(_wp_src) if _wp_src else 0

            cache_ok = (
                hasattr(self, '_waypoints_for_mpc_cache_id') and
                self._waypoints_for_mpc_cache_id == _wp_src_id and
                getattr(self, '_waypoints_for_mpc_cache_len', -1) == _wp_len
            )

            if cache_ok:
                waypoints_for_mpc = self._waypoints_for_mpc_cache
            else:
                if _wp_src and len(_wp_src) >= 2:
                    is_3d_waypoints = len(_wp_src[0]) >= 3
                    if is_3d_waypoints:
                        waypoints_for_mpc = tuple((float(wp[0]), float(wp[1]), float(wp[2])) for wp in _wp_src)
                    else:
                        waypoints_for_mpc = tuple((float(wp[0]), float(wp[1])) for wp in _wp_src)
                else:
                    waypoints_for_mpc = None

                self._waypoints_for_mpc_cache = waypoints_for_mpc
                self._waypoints_for_mpc_cache_id = _wp_src_id
                self._waypoints_for_mpc_cache_len = _wp_len

            # Compute steering using MPC (mpc_total in [LoopOther] from record_lateral_mpc_ms + record_longitudinal_mpc_ms)
            # Pass behavior's progress index (wp_last_idx) so MPC searches locally; MPC owns chosen segment (last_seg_idx), we do not sync back.
            try:
                steer_mpc = _lat_controller.run_step(
                    vehicle_state,
                    waypoints_for_mpc,
                    wp_last_idx if (use_waypoints and wp_list and len(wp_list) >= 2) else None,
                    cte_magnitude=cte_mag_for_speed,
                    v_ref_profile=v_ref_profile,  # Same trajectory as longitudinal: smooth turns, avoid over-steer then correct
                    curvature_ahead_max=curvature_ahead_max  # For deadzone eligibility: never deadzone in moderate curvature (curv_ahead_max < curv_deadzone_max)
                )
                steer_mpc = float(steer_mpc)
    
            except Exception as e:
                print(f"{_fbhv} MPC error: {e}, using fallback")
                steer_mpc = 0.0
            if is_fellow_mpc_virt:
                __import__(
                    "scenic.domains.racing.mpc.fellow_virtual_mpc_state",
                    fromlist=["fellow_virt_step_for_scenic"],
                ).fellow_virt_step_for_scenic(
                    self,
                    float(steer_mpc),
                    current_speed,
                    getattr(_lat_controller, "_log_kappa_ref_at_proj", None),
                    float(_ctrl_dt_virt),
                    float(sim.mpc_config.wheel_base),
                    float(sim.mpc_config.steer_tau),
                )
            if _bt is not None:
                _bt.start_section('cmd_post')
            # Keep waypoint-based CTE (e_y_mpc) for mismatch fallback (used when behavior CTE disagrees)
            if getattr(_lat_controller, '_log_mpc_e_y', None) is not None:
                self._last_waypoint_cte_for_speed = abs(getattr(_lat_controller, '_log_mpc_e_y'))
    
            # --- CTE-aware safety envelope (for throttle/brake) ---
            # Single definition: MPC e_y (projection onto chosen segment) when available; legacy only as fallback when MPC did not run.
            _ey = getattr(_lat_controller, '_log_mpc_e_y', None)
            if _ey is not None:
                cte = float(_ey)
            else:
                cte = float(getattr(self, '_legacy_cte_this_tick', 0.0))
            cte_mag = abs(cte)
            # One ff_log line per MPC tick (feedforward + command side: u_norm, delta_cmd_rad post-clamp, current_delta_max)
            _lc_ff = _lat_controller
            _seg_id = getattr(_lc_ff, '_log_segment_id', None)
            _v_ff = getattr(_lc_ff, '_log_v', None)
            _kap_ff = getattr(_lc_ff, '_log_kappa_ref_at_proj', None)
            _dff = getattr(_lc_ff, '_log_delta_ff', None)
            _dfb = getattr(_lc_ff, '_log_delta_fb', None)
            _dtot = getattr(_lc_ff, '_log_delta_total', None)
            _dcmd = getattr(_lc_ff, '_log_delta_cmd_rad', None)
            _dmax = getattr(_lc_ff, '_log_current_delta_max', None)
            _sraw = getattr(_lc_ff, '_log_steer_mpc_raw', None)
            _slpf = getattr(_lc_ff, '_log_steer_after_lpf', None)
            _seg_s = str(_seg_id) if _seg_id is not None else "?"
            _v_s = f"{_v_ff:.3f}" if _v_ff is not None else "?"
            _kap_s = f"{_kap_ff:.4f}" if _kap_ff is not None else "?"
            _dff_s = f"{_dff:.4f}" if _dff is not None else "?"
            _dfb_s = f"{_dfb:.4f}" if _dfb is not None else "?"
            _dtot_s = f"{_dtot:.4f}" if _dtot is not None else "?"
            _dcmd_s = f"{_dcmd:.4f}" if _dcmd is not None else "?"
            _dmax_s = f"{_dmax:.4f}" if _dmax is not None else "?"
            _sraw_s = f"{_sraw:.3f}" if _sraw is not None else "?"
            _slpf_s = f"{_slpf:.3f}" if _slpf is not None else "?"
            _log_step = getattr(self, '_behavior_step_count', 0) + 1
            if _log_step % 50 == 0:
                print(f"{_fl_mpc} ff_log segment_id={_seg_s} v={_v_s} kappa_ref_at_proj={_kap_s} delta_ff={_dff_s} delta_fb={_dfb_s} delta_total={_dtot_s} delta_cmd_rad={_dcmd_s} current_delta_max={_dmax_s} u_norm_mpc={_sraw_s} u_norm_lpf={_slpf_s}")
                # Task 1: signed curvature sanity log (every 50 steps)
                _kappa_ahead = getattr(_lc_ff, '_log_kappa_ref_ahead_signed', None)
                _s_ref = getattr(_lc_ff, '_log_s_ref', None)
                _kappa_ahead_s = f"{_kappa_ahead:.4f}" if _kappa_ahead is not None else "?"
                _s_ref_s = f"{_s_ref:.3f}" if _s_ref is not None else "?"
                print(f"{_fl_mpc} CURV_SANITY kappa_ref_at_proj={_kap_s} kappa_ref_ahead_signed={_kappa_ahead_s} segment_id={_seg_s} s_ref={_s_ref_s}")
            # Heading diff for steering conditioning (segment direction vs vehicle heading, wrapped to [-pi, pi])
            heading_diff = 0.0
            if car_heading is not None and use_waypoints and wp_list and len(wp_list) >= 2:
                n_wp = len(wp_list)
                next_idx = (wp_last_idx + 1) % n_wp
                x0, y0 = float(wp_list[wp_last_idx][0]), float(wp_list[wp_last_idx][1])
                x1, y1 = float(wp_list[next_idx][0]), float(wp_list[next_idx][1])
                seg_heading = math.atan2(y1 - y0, x1 - x0)
                heading_diff = math.atan2(math.sin(seg_heading - car_heading), math.cos(seg_heading - car_heading))
            local_throttle_limit = throttle_limit
            final_brake = 0.0
    
            # Plan: steering = road wheel angle (rad). Single source of truth delta_max (racing.constants).
            # Progressive throttle reduction based on CTE magnitude
            if cte_mag >= cte_stop_threshold:
                local_throttle_limit = 0.0
                final_brake = 1.0
                steer_mpc = (-0.5 * DELTA_MAX_RAD) if cte > 0 else (0.5 * DELTA_MAX_RAD)
            elif cte_mag >= cte_slowdown_threshold:
                local_throttle_limit = min(local_throttle_limit, 0.3)
                final_brake = min(1.0, (cte_mag - cte_slowdown_threshold) / (cte_stop_threshold - cte_slowdown_threshold))
            elif cte_mag >= cte_throttle_reduction_max:
                throttle_factor = 1.0 - ((cte_mag - cte_throttle_reduction_max) / (cte_slowdown_threshold - cte_throttle_reduction_max)) * 0.7
                local_throttle_limit = min(local_throttle_limit, throttle_limit * throttle_factor)
                local_throttle_limit = max(local_throttle_limit, 0.3)
            elif cte_mag >= cte_throttle_reduction_start:
                throttle_factor = 1.0 - ((cte_mag - cte_throttle_reduction_start) / (cte_throttle_reduction_max - cte_throttle_reduction_start)) * (1.0 - min_throttle_at_large_cte / throttle_limit)
                local_throttle_limit = min(local_throttle_limit, throttle_limit * throttle_factor)
                local_throttle_limit = max(local_throttle_limit, min_throttle_at_large_cte)
    
            # Speed-based throttle reduction when CTE is large
            if cte_mag >= cte_throttle_reduction_start and current_speed > 3.0:
                if current_speed <= 8.0:
                    speed_penalty = (current_speed - 3.0) / 10.0
                else:
                    speed_penalty = 0.5 + ((current_speed - 8.0) / 10.0) * 0.3
                speed_penalty = min(0.8, speed_penalty)
                local_throttle_limit = local_throttle_limit * (1.0 - speed_penalty)
    
            # Additional throttle reduction for moderate CTE (2-4m) when speed is high
            if cte_mag >= 2.0 and cte_mag < 5.0 and current_speed > 4.0:
                if current_speed <= 6.0:
                    moderate_cte_penalty = (current_speed - 4.0) / 4.0
                else:
                    moderate_cte_penalty = 0.5 + ((current_speed - 6.0) / 4.0) * 0.3
                moderate_cte_penalty = min(0.8, moderate_cte_penalty)
                local_throttle_limit = local_throttle_limit * (1.0 - moderate_cte_penalty)
    
            
            # --- Steering: MPC is single owner of clamp/rate limit (mpc_lateral.py). Safety backup below. ---
            final_steer = max(-DELTA_MAX_RAD, min(DELTA_MAX_RAD, float(steer_mpc)))
            final_steer_ds = final_steer   # rad; IO adapter converts to dSPACE steering_wheel_deg in simulator
            # fix.md Option 1: heading-based yaw rate (avoids yaw-rate channel stuck at 0)
            _psi_rad = float(car_heading) if car_heading is not None else None
            _dt = float(simulation().timestep) if hasattr(simulation(), 'timestep') else 0.05
            _psi_prev = getattr(self, '_psi_prev_steer_cal', None)
            _yaw_rps_est = None
            if _psi_rad is not None and _psi_prev is not None and _dt > 1e-6:
                _dpsi = _psi_rad - _psi_prev
                _dpsi = math.atan2(math.sin(_dpsi), math.cos(_dpsi))  # wrapToPi
                _yaw_rps_est = _dpsi / _dt
            self._psi_prev_steer_cal = _psi_rad
            # Unit-inference analysis: prefer yaw_rps_est when channel yaw_rate is zero/missing (fix.md)
            _L_wb = getattr(_lat_controller.config, 'wheel_base', 2.97)
            _delta_cmd_rad = float(final_steer)
            _kappa_pred = math.tan(_delta_cmd_rad) / _L_wb if _L_wb > 1e-6 else 0.0
            _speed_mps = float(current_speed) if current_speed is not None else 0.0
            _yaw_rate_ch = vehicle_state.get('yaw_rate', None) if vehicle_state else None
            _speed_floor = 0.5
            _yaw_rps_use = _yaw_rps_est if (_yaw_rate_ch is None or abs(float(_yaw_rate_ch)) < 1e-6) else _yaw_rate_ch
            if _yaw_rps_use is not None:
                _yaw_rps_use = float(_yaw_rps_use)
            _kappa_meas = (_yaw_rps_use / max(_speed_mps, _speed_floor)) if _yaw_rps_use is not None else None
            _curv_err = (_kappa_meas - _kappa_pred) if _kappa_meas is not None else None
            _kappa_ratio = (_kappa_meas / _kappa_pred) if (_kappa_meas is not None and abs(_kappa_pred) > 1e-3) else None
            _psi_s = f"{_psi_rad:.4f}" if _psi_rad is not None else "?"
            _yaw_est_s = f"{_yaw_rps_est:.4f}" if _yaw_rps_est is not None else "?"
            _kmeas_s = f"{_kappa_meas:.5f}" if _kappa_meas is not None else "?"
            _cerr_s = f"{_curv_err:.5f}" if _curv_err is not None else "?"
            _krat_s = f"{_kappa_ratio:.4f}" if _kappa_ratio is not None else "?"
            _steer_cal_tick = getattr(self, '_steer_cal_log_count', 0)
            _log_step_plant = getattr(self, '_behavior_step_count', 0) + 1
            if _log_step_plant % 50 == 0:
                print(f"[STEER_CAL] v={_speed_mps:.3f} psi={_psi_s} yaw_rps_est={_yaw_est_s} delta_cmd_rad={_delta_cmd_rad:.4f} L={_L_wb:.3f} kappa_meas={_kmeas_s} kappa_pred={_kappa_pred:.5f} curv_err={_cerr_s} kappa_ratio={_krat_s}")
                print(f"[PLANT] speed_mps={_speed_mps:.3f} kappa_pred={_kappa_pred:.4f} kappa_meas={_kmeas_s} curvature_error={_cerr_s}")
            self._steer_cal_log_count = _steer_cal_tick + 1
    
            # Ensure local_throttle_limit doesn't exceed base throttle_limit
            local_throttle_limit = min(local_throttle_limit, throttle_limit)
            
            # Use MPC throttle/brake outputs (already processed by CTE-aware safety above)
            final_throttle = max(0.0, min(local_throttle_limit, throttle_mpc))
            final_brake = raw_brake  # Already merged and capped above
            
            # Hard speed limit: pit mode vs main (racing library mode drives which limit applies)
            current_speed_limit_ms = PIT_MAX_SPEED_MS if pit_mode else MAX_SPEED_LIMIT_MS
            SPEED_LIMIT_DEADBAND = 0.5  # m/s: trigger brake when speed > limit+deadband; release when speed < limit-deadband
            speed_limit_applied_this_step = False
            was_limit_active = getattr(self, '_speed_limit_active', False)
            if current_speed > current_speed_limit_ms + SPEED_LIMIT_DEADBAND:
                self._speed_limit_active = True
            if current_speed < current_speed_limit_ms - SPEED_LIMIT_DEADBAND:
                self._speed_limit_active = False
            # Pit mode: when over limit we only coast (no brake); throttle is zeroed in pit block when speed >= 35 mph
            apply_pit_limit = pit_mode and current_speed > PIT_MAX_SPEED_MS
            apply_main_limit = (not pit_mode) and self._speed_limit_active and current_speed > current_speed_limit_ms - SPEED_LIMIT_DEADBAND
            if apply_main_limit:
                speed_limit_applied_this_step = True
                speed_excess = current_speed - current_speed_limit_ms
                if speed_excess > 2.0:
                    final_throttle = 0.0
                    final_brake = max(final_brake, 0.5)
                elif speed_excess > 1.0:
                    final_throttle = 0.0
                    final_brake = max(final_brake, 0.3)
                else:
                    final_throttle = max(0.0, final_throttle * 0.5)
                    final_brake = max(final_brake, 0.1)
                step_for_log = getattr(self, '_behavior_step_count', 0) + 1
                print(f"[Speed Limit] step={step_for_log} Speed {current_speed:.2f}m/s exceeds limit {current_speed_limit_ms:.1f}m/s (main), applying brake={final_brake:.3f}")
            elif apply_pit_limit:
                # Pit: over 35 mph — coast only (no brake); throttle is zeroed below when speed >= PIT_MAX_SPEED_MS
                speed_limit_applied_this_step = False
                step_for_log = getattr(self, '_behavior_step_count', 0) + 1
                print(f"[Speed Limit] step={step_for_log} Speed {current_speed:.2f}m/s exceeds limit {current_speed_limit_ms:.1f}m/s (pit 35mph, coast only)")
    
            # Global mutual exclusion: never command throttle and brake at the same time.
            # When slowing (e.g. for a turn), lift throttle and brake only—no simultaneous throttle+brake.
            BRAKE_THROTTLE_EXCLUSION_THRESHOLD = 0.05  # treat as "active" above this
            if final_brake > BRAKE_THROTTLE_EXCLUSION_THRESHOLD:
                final_throttle = 0.0
            elif final_throttle > BRAKE_THROTTLE_EXCLUSION_THRESHOLD:
                final_brake = 0.0
    
            # Throttle ramp after brake release: avoid jumping from full brake to full throttle (generic, any TTL)
            THROTTLE_RAMP_STEPS = 12   # ~0.6 s at 0.05 s/step
            THROTTLE_RAMP_START = 0.2  # throttle cap at first step after brake
            was_braking = getattr(self, '_last_brake_heavy', False)
            self._last_brake_heavy = (final_brake > 0.2)
            ramp_remaining = getattr(self, '_throttle_ramp_steps_remaining', 0)
            if was_braking and final_brake < BRAKE_THROTTLE_EXCLUSION_THRESHOLD:
                ramp_remaining = THROTTLE_RAMP_STEPS
            if ramp_remaining > 0:
                # Linear ramp: cap throttle so it increases smoothly over THROTTLE_RAMP_STEPS
                step_in_ramp = THROTTLE_RAMP_STEPS - ramp_remaining
                throttle_cap = THROTTLE_RAMP_START + (1.0 - THROTTLE_RAMP_START) * (step_in_ramp / THROTTLE_RAMP_STEPS)
                final_throttle = min(final_throttle, throttle_cap)
                self._throttle_ramp_steps_remaining = ramp_remaining - 1
            else:
                self._throttle_ramp_steps_remaining = 0
            # Pit mode: cap throttle at 50%; when speed >= 35 mph use zero throttle to coast (don't push past limit)
            PIT_MAX_THROTTLE = 0.5
            if pit_mode:
                if current_speed >= PIT_COAST_MIN_TARGET_MS:
                    final_throttle = 0.0
                else:
                    final_throttle = min(final_throttle, PIT_MAX_THROTTLE)
                # Debug: log pit mode every 50 steps (speed, throttle, target 35 mph)
                _step_num = getattr(self, '_behavior_step_count', 0) + 1
                if _step_num % 50 == 0 or _step_num == 1:
                    _t_log = _step_num * (getattr(simulation(), 'control_dt', None) or getattr(simulation(), 'timestep', 0.05))
                    _reason = "coast (speed>=35mph)" if current_speed >= PIT_COAST_MIN_TARGET_MS else f"cap {PIT_MAX_THROTTLE}"
                    print(f"[Pit] step={_step_num} t={_t_log:.2f}s pit_mode=True speed={current_speed:.2f}m/s target=35mph throttle={final_throttle:.2f} ({_reason})")
            self._last_final_steer = final_steer
            self._last_final_throttle = final_throttle
            self._last_final_brake = final_brake
            self._last_curvature_ahead_max = curvature_ahead_max

        # Store CTE for debugging
        self._current_cte = cte

        # ---- Detailed drive logging (heavy brake / near stop / speed drop) ----
        # Use step_for_log (not _step) to avoid shadowing the behavior's _step() method
        # Timestamp t = step * control_dt for systematic comparison across runs
        sim = simulation()
        ctrl_dt = getattr(sim, 'control_dt', None)
        if ctrl_dt is None or ctrl_dt <= 0:
            ctrl_dt = getattr(sim, 'control_period', None)
        if ctrl_dt is None or ctrl_dt <= 0:
            ctrl_dt = float(getattr(sim, 'timestep', 0.05))
        step_for_log = getattr(self, '_behavior_step_count', 0) + 1
        t_log = step_for_log * ctrl_dt
        _last_speed = getattr(self, '_last_speed', None)
        if final_brake > 0.25:
            cte_show = float(cte) if cte is not None else 0.0
            print(f"[Drive] t={t_log:.2f}s step={step_for_log} Heavy brake: speed={current_speed:.2f}m/s brake={final_brake:.3f} throttle={final_throttle:.3f} | speed_limit={speed_limit_applied_this_step} cte={cte_show:.2f}m brake_mpc={brake_mpc:.3f}")
        if current_speed is not None and current_speed < 6.0:
            print(f"[Drive] t={t_log:.2f}s step={step_for_log} Low speed: speed={current_speed:.2f}m/s brake={final_brake:.3f} throttle={final_throttle:.3f}")
        if _last_speed is not None and current_speed is not None and (current_speed - _last_speed) < -4.0:
            print(f"[Drive] t={t_log:.2f}s step={step_for_log} Speed drop: from {_last_speed:.2f} to {current_speed:.2f} m/s (delta={current_speed - _last_speed:.2f})")
        self._last_speed = float(current_speed) if current_speed is not None else 0.0
        
        # Build Action List 
        actions_to_take = [
            SetSteerAction(final_steer), 
            SetThrottleAction(final_throttle), 
            SetBrakeAction(final_brake)
        ]

        # Gear Logic: proactive downshift before turns + speed-based shifts
        gear_changed = False
        new_gear = None
        if manage_gears and hasattr(self, 'setGear'):
            current_gear = getattr(self, 'gear', 0) 
            
            if current_gear < 1:
                actions_to_take.append(SetGearAction(1))
                self.gear = 1
                gear_changed = True
                new_gear = 1
                print(f"  [Gear] Shifting from {current_gear} to 1 (starting from neutral)")
            
            elif current_speed is not None:
                # Proactive downshift before turns (curvature-ahead aware)
                curvature_very_tight = 0.08   # 1/m, tight turn -> prefer gear 1
                curvature_tight = 0.05        # 1/m, turn -> prefer one gear lower
                proactive_downshift = None
                if curvature_ahead_max >= curvature_very_tight and current_gear >= 2 and current_speed < 12.0:
                    proactive_downshift = 1  # 2->1 before very tight turn
                elif curvature_ahead_max >= curvature_tight and current_gear >= 3 and current_speed < 20.0:
                    proactive_downshift = current_gear - 1  # 3->2 (or 4->3, etc.) before turn
                if proactive_downshift is not None:
                    new_gear = proactive_downshift
                    actions_to_take.append(SetGearAction(new_gear))
                    self.gear = new_gear
                    gear_changed = True
                    print(f"  [Gear] Proactive downshift from {current_gear} to {new_gear} (curvature_ahead={curvature_ahead_max:.3f} 1/m, speed={current_speed:.2f} m/s)")
                elif current_gear < 6 and current_speed > gear_up_thresholds[min(current_gear, 5)]:
                    new_gear = current_gear + 1
                    actions_to_take.append(SetGearAction(new_gear))
                    self.gear = new_gear
                    gear_changed = True
                    print(f"  [Gear] Shifting up from {current_gear} to {new_gear} (speed={current_speed:.2f} m/s)")
                elif current_gear > 1 and current_speed < gear_down_thresholds[min(current_gear - 1, 5)]:
                    new_gear = current_gear - 1
                    actions_to_take.append(SetGearAction(new_gear))
                    self.gear = new_gear
                    gear_changed = True
                    print(f"  [Gear] Shifting down from {current_gear} to {new_gear} (speed={current_speed:.2f} m/s)")

        # Debug Print
        if hasattr(self, '_behavior_step_count'):
            self._behavior_step_count += 1
        else:
            self._behavior_step_count = 0
        
        gear_val = getattr(self, 'gear', 0)
        self._last_final_gear = gear_val if gear_val >= 1 else 1
        # Log step summary every 50 steps; include t= and OpenDRIVE-based segment for segment performance analysis
        if self._behavior_step_count % 50 == 0:
            sim = simulation()
            ctrl_dt = getattr(sim, 'control_dt', None)
            if ctrl_dt is None or ctrl_dt <= 0:
                ctrl_dt = getattr(sim, 'control_period', None)
            if ctrl_dt is None or ctrl_dt <= 0:
                ctrl_dt = float(getattr(sim, 'timestep', 0.05))
            t_log = self._behavior_step_count * ctrl_dt
            _eid_log = getattr(self, '_last_valid_segment_id', None)
            _ename_log = getattr(self, '_last_valid_segment_name', "") or ""
            segment_str = f" {get_segment_label(_eid_log, _ename_log)}" if (_eid_log is not None or _ename_log) else " segment ?"
            print(f"{_fl_mpc} t={t_log:.2f}s Step {self._behavior_step_count}: pos=({px:.2f},{py:.2f}) speed={current_speed:.2f}m/s CTE={cte:.3f}m steer={final_steer:.3f} throttle={final_throttle:.3f} brake={final_brake:.3f} gear={gear_val} curv_ahead={curvature_ahead_max:.3f}{segment_str}")
            # Phase 0 telemetry (baseline visibility): active TTL, planner mode, ego s/speed,
            # nearest-opponent relative metrics, and event markers (switch/near-miss/collision/off-track).
            _ttl_label = None
            _ttl_file = getattr(self, 'ttlFileName', None)
            _ttl_sel = getattr(self, 'ttl_selection', None)
            if isinstance(_ttl_sel, str) and _ttl_sel:
                _ttl_label = _ttl_sel
            elif isinstance(_ttl_file, str) and _ttl_file:
                _tf = _ttl_file.lower()
                if "left" in _tf:
                    _ttl_label = "left"
                elif "right" in _tf:
                    _ttl_label = "right"
                elif "pit" in _tf:
                    _ttl_label = "pit"
                else:
                    _ttl_label = "optimal"
            else:
                _ttl_label = "unknown"
            _planner_mode = getattr(self, 'strategy_type', None) or "follow_mpc"
            # Use MPC controller progress estimate if available; keep None-safe for early ticks.
            _ego_s = getattr(_lat_controller, '_log_s_ref', None)

            # Detect TTL switch event.
            _last_ttl_label = getattr(self, '_phase0_last_ttl_label', None)
            if _last_ttl_label is None:
                self._phase0_last_ttl_label = _ttl_label
            elif _last_ttl_label != _ttl_label:
                print(f"[Phase0Event] t={t_log:.2f}s type=ttl_switch from={_last_ttl_label} to={_ttl_label}")
                self._phase0_last_ttl_label = _ttl_label

            # Nearest-opponent visibility.
            nearest_obj = None
            nearest_dist = None
            nearest_rel_speed = None
            nearest_rel_longitudinal = None
            try:
                _objs = getattr(simulation().scene, 'objects', [])
                _ego_h = car_heading if car_heading is not None else 0.0
                _ego_fx = math.cos(_ego_h)
                _ego_fy = math.sin(_ego_h)
                _ego_speed = float(current_speed if current_speed is not None else 0.0)
                _best_d2 = None
                for _obj in _objs:
                    if _obj is self:
                        continue
                    if not hasattr(_obj, 'position') or _obj.position is None:
                        continue
                    _ox = float(_obj.position.x); _oy = float(_obj.position.y)
                    _dx = _ox - px
                    _dy = _oy - py
                    _d2 = _dx*_dx + _dy*_dy
                    if _best_d2 is None or _d2 < _best_d2:
                        _best_d2 = _d2
                        nearest_obj = _obj
                        nearest_dist = _d2 ** 0.5
                        _ov = float(getattr(_obj, 'speed', 0.0) or 0.0)
                        nearest_rel_speed = _ov - _ego_speed
                        # Positive means opponent is ahead along ego heading.
                        nearest_rel_longitudinal = _dx * _ego_fx + _dy * _ego_fy
            except Exception:
                nearest_obj = None

            _opp_dist_s = f"{nearest_dist:.2f}" if nearest_dist is not None else "na"
            _opp_rel_v_s = f"{nearest_rel_speed:.2f}" if nearest_rel_speed is not None else "na"
            _opp_rel_s_s = f"{nearest_rel_longitudinal:.2f}" if nearest_rel_longitudinal is not None else "na"
            _ego_s_s = f"{_ego_s:.2f}" if _ego_s is not None else "na"
            print(
                f"[Phase0] t={t_log:.2f}s ttl={_ttl_label} planner_mode={_planner_mode} "
                f"ego_s={_ego_s_s} ego_speed={current_speed:.2f} "
                f"nearest_opp_ds={_opp_rel_s_s} nearest_opp_rel_speed={_opp_rel_v_s} nearest_opp_dist={_opp_dist_s}"
            )
            # Event thresholds for baseline KPI extraction.
            if nearest_dist is not None and nearest_dist <= 3.0:
                print(f"[Phase0Event] t={t_log:.2f}s type=near_miss distance_m={nearest_dist:.2f}")
            if nearest_dist is not None and nearest_dist <= 1.0:
                print(f"[Phase0Event] t={t_log:.2f}s type=collision distance_m={nearest_dist:.2f}")
            if cte is not None and abs(float(cte)) >= 10.0:
                print(f"[Phase0Event] t={t_log:.2f}s type=off_track cte_m={float(cte):.3f}")
            # Ref continuity / gate logging (todo1: match_dist, gate ACCEPT/REJECT, s_ref/dS_ref/s_jump_flag, segment_prev->new, stick_blocked, e_y_mpc vs cte_behavior)
            _lc = _lat_controller
            _md = getattr(_lc, '_log_match_dist_m', None)
            _gs = getattr(_lc, '_log_gate_status', '?')
            _gr = getattr(_lc, '_log_gate_reason', None)
            _sr = getattr(_lc, '_log_s_ref', None)
            _ds = getattr(_lc, '_log_delta_s_ref', None)
            _sj = getattr(_lc, '_log_s_jump_flag', False)
            _sp = getattr(_lc, '_log_segment_prev', None)
            _sn = getattr(_lc, '_log_segment_new', None)
            _st = getattr(_lc, '_log_stick_blocked', False)
            _ey = getattr(_lc, '_log_mpc_e_y', None)
            _md_s = f"{_md:.3f}" if _md is not None else "?"
            _gr_s = f" reason={_gr}" if _gr else ""
            _sr_s = f"{_sr:.2f}" if _sr is not None else "?"
            _ds_s = f"{_ds:.3f}" if _ds is not None else "?"
            _seg_s = f"{_sp}->{_sn}" if _sp is not None else f"->{_sn}"
            _ey_s = f"{_ey:.3f}" if _ey is not None else "?"
            cte_b = float(cte) if cte is not None else 0.0
            _legacy_s = f"{getattr(self, '_legacy_cte_this_tick', 0):.3f}" if getattr(self, '_legacy_cte_this_tick', None) is not None else "?"
            print(f"{_fl_mpc} ref_log match_dist_m={_md_s} gate={_gs}{_gr_s} s_ref={_sr_s} dS_ref={_ds_s} s_jump_flag={1 if _sj else 0} seg={_seg_s} stick={1 if _st else 0} e_y_mpc={_ey_s} cte_behavior={cte_b:.3f} legacy_cte={_legacy_s}")
            # Task 4: Quick projection check — match_dist_m, proj_xy, ego_xy, segment_id progression (stuck = segment_id stops advancing or match_dist spikes at left-right transition)
            _rp = getattr(_lc, '_log_ref_point', None)
            _ego = getattr(_lc, '_log_ego', None)
            _proj_s = f"({_rp[0]:.3f},{_rp[1]:.3f})" if _rp is not None and len(_rp) >= 2 else "?"
            _ego_xy_s = f"({_ego[0]:.3f},{_ego[1]:.3f})" if _ego is not None and len(_ego) >= 2 else "?"
            print(f"{_fl_mpc} projection_check match_dist_m={_md_s} proj_xy={_proj_s} veh_xy={_ego_xy_s} segment_id={_seg_s}")
            # Task 1: projection continuity — s_ref, segment_id, proj_xy, match_dist; ensure s doesn't jump and projection doesn't hop
            _s_ok = 1 if getattr(_lc, '_log_s_ref_continuous', True) else 0
            _hop_ok = 1 if not getattr(_lc, '_log_proj_hop', False) else 0
            _cont_ok = 1 if getattr(_lc, '_log_projection_continuity_ok', True) else 0
            print(f"{_fl_mpc} projection_continuity s_ref={_sr_s} segment_id={_seg_s} proj_xy={_proj_s} match_dist_m={_md_s} s_ok={_s_ok} proj_hop_ok={_hop_ok} continuity_ok={_cont_ok}")
            _stuck_hint = False
            if _md is not None and _md > 5.0:
                _stuck_hint = True  # match_dist spike
            if _sp is not None and _sn is not None and _sp == _sn and _ds is not None and abs(_ds) < 0.01 and _md is not None and _md > 2.0:
                _stuck_hint = True  # segment not advancing with significant match_dist
            if _stuck_hint:
                print(f"{_fl_mpc} projection_check STUCK? (segment_id not advancing or match_dist spike)")
            # CTE cross-check log: cte_to_waypoints = e_y_mpc (single source); cte_behavior is same when MPC ran
            _rp = getattr(_lc, '_log_ref_point', None)
            _ego = getattr(_lc, '_log_ego', None)
            _cte_wp_s = _ey_s
            _rp_s = f"({_rp[0]:.3f},{_rp[1]:.3f})" if _rp is not None and len(_rp) >= 2 else "?"
            _ego_s = f"({_ego[0]:.3f},{_ego[1]:.3f})" if _ego is not None and len(_ego) >= 2 else "?"
            print(f"{_fl_mpc} ct_crosscheck cte_to_waypoints={_cte_wp_s} cte_behavior={cte_b:.3f} ref_point={_rp_s} vehicle={_ego_s}")
            if _ey is not None:
                self._last_waypoint_cte_for_speed = abs(_ey)
            # To-Do C: Polyline identity (behavior CTE, MPCC waypoints, segment map) — n_pts, first/last, total length, id
            _pts_b = wp_list if (use_waypoints and wp_list) else None
            if _pts_b is None or len(_pts_b) == 0:
                _pw = "behavior_cte_polyline: n=0"
            else:
                _n_b = len(_pts_b)
                _f_b = (float(_pts_b[0][0]), float(_pts_b[0][1])) if len(_pts_b[0]) >= 2 else (0, 0)
                _l_b = (float(_pts_b[-1][0]), float(_pts_b[-1][1])) if len(_pts_b[-1]) >= 2 else (0, 0)
                _len_b = 0.0
                for _i in range(len(_pts_b) - 1):
                    _a, _b = _pts_b[_i], _pts_b[_i + 1]
                    _len_b += ((float(_b[0]) - float(_a[0]))**2 + (float(_b[1]) - float(_a[1]))**2) ** 0.5
                _pw = f"behavior_cte_polyline: id={id(_pts_b)} n={_n_b} first=({_f_b[0]:.2f},{_f_b[1]:.2f}) last=({_l_b[0]:.2f},{_l_b[1]:.2f}) length={_len_b:.2f}m"
            _pts_m = waypoints_for_mpc if waypoints_for_mpc else None
            if _pts_m is None or len(_pts_m) == 0:
                _pm = "mpcc_waypoint_polyline: n=0"
            else:
                _n_m = len(_pts_m)
                _f_m = (float(_pts_m[0][0]), float(_pts_m[0][1])) if len(_pts_m[0]) >= 2 else (0, 0)
                _l_m = (float(_pts_m[-1][0]), float(_pts_m[-1][1])) if len(_pts_m[-1]) >= 2 else (0, 0)
                _len_m = 0.0
                for _i in range(len(_pts_m) - 1):
                    _a, _b = _pts_m[_i], _pts_m[_i + 1]
                    _len_m += ((float(_b[0]) - float(_a[0]))**2 + (float(_b[1]) - float(_a[1]))**2) ** 0.5
                _pm = f"mpcc_waypoint_polyline: id={id(_pts_m)} n={_n_m} first=({_f_m[0]:.2f},{_f_m[1]:.2f}) last=({_l_m[0]:.2f},{_l_m[1]:.2f}) length={_len_m:.2f}m"
            _smap = getattr(self, '_waypoint_segment_map', None)
            if _smap is not None and len(_smap) > 0:
                _seg_first = _smap[0]
                _seg_last = _smap[-1]
                _sm_s = f"segment_map: id={id(_smap)} n={len(_smap)} first_seg=({_seg_first[0]},{_seg_first[1]}) last_seg=({_seg_last[0]},{_seg_last[1]})"
            else:
                _sm_s = "segment_map: (none)"
            print(f"{_fl_mpc} polyline_check {_pw} | {_pm} | {_sm_s}")
            # ff_log is printed every MPC tick (see above) with segment_id, v, kappa_ref_at_proj, delta_ff, delta_fb, delta_total, steer_mpc_raw, steer_after_lpf

        # Supplement log (Todo2): deadzone decision, association, curvature, steering — every 10 ticks or when deadzone state changes
        _lc2 = _lat_controller
        _dz_app = getattr(_lc2, '_log_deadzone_applied', False)
        _last_dz = getattr(self, '_last_deadzone_applied', None)
        _dz_changed = (_last_dz is not None and _dz_app != _last_dz)
        self._last_deadzone_applied = _dz_app
        if (self._behavior_step_count % 10 == 0) or _dz_changed:
            _dz_m = getattr(_lc2, '_log_dz_cte_m', None)
            _cte_used = getattr(_lc2, '_log_cte_used_for_control', None)
            _cte_raw = getattr(_lc2, '_log_cte_raw', None)
            _reason = getattr(_lc2, '_log_deadzone_reason', '?')
            _match_d = getattr(_lc2, '_log_match_dist_m', None)
            _gate_ok = getattr(_lc2, '_log_gate_accept', None)
            _seg_id = getattr(_lc2, '_log_segment_id', None)
            _kappa = getattr(_lc2, '_log_kappa_ref_at_proj', None)
            _creg = getattr(_lc2, '_log_curv_regime', '?')
            _s_raw = getattr(_lc2, '_log_steer_mpc_raw', None)
            _s_caps = getattr(_lc2, '_log_steer_after_caps', None)
            _s_lpf = getattr(_lc2, '_log_steer_after_lpf', None)
            _s_rate = getattr(_lc2, '_log_steer_rate', None)
            _curv_ahd = curvature_ahead_max
            _dz_m_s = f"{_dz_m:.3f}" if _dz_m is not None else "?"
            _cte_used_s = f"{_cte_used:.3f}" if _cte_used is not None else "?"
            _cte_raw_s = f"{_cte_raw:.3f}" if _cte_raw is not None else "?"
            _match_s = f"{_match_d:.3f}" if _match_d is not None else "?"
            _kappa_s = f"{_kappa:.3f}" if _kappa is not None else "?"
            _curv_ahd_s = f"{_curv_ahd:.3f}" if _curv_ahd is not None else "?"
            _s_raw_s = f"{_s_raw:.3f}" if _s_raw is not None else "?"
            _s_caps_s = f"{_s_caps:.3f}" if _s_caps is not None else "?"
            _s_lpf_s = f"{_s_lpf:.3f}" if _s_lpf is not None else "?"
            _s_rate_s = f"{_s_rate:.3f}" if _s_rate is not None else "?"
            print(f"{_fl_mpc} deadzone_log deadzone_applied={_dz_app} dz_cte_m={_dz_m_s} cte_used_for_control={_cte_used_s} cte_raw={_cte_raw_s} deadzone_reason={_reason} match_dist_m={_match_s} gate_accept={_gate_ok} segment_id={_seg_id} curv_ahead_max={_curv_ahd_s} curv_regime={_creg} kappa_ref_at_proj={_kappa_s} steer_mpc_raw={_s_raw_s} steer_after_caps={_s_caps_s} steer_after_lpf={_s_lpf_s} steer_rate={_s_rate_s}")

        if _bt is not None:
            _bt.end_section('cmd_post')
        # Execute all actions together
        take actions_to_take

behavior PitStopBehavior(manage_gears=True):
    """Execute a pit stop using racing-specific systems.
    
    This behavior demonstrates the use of racing-specific actions like
    pit limiter and ERS deployment.
    """
    
    # Enter pit lane with speed limiter
    take PitLimiterAction(activate=True)
    do FollowRacingLineMPCBehavior(target_speed=20, manage_gears=manage_gears)
    
    # Stop for pit stop
    take SetBrakeAction(1.0)
    wait  # Simulate pit stop time
    
    # Exit pit lane
    take PitLimiterAction(activate=False)

behavior OvertakingBehavior(target_car, aggressive=False):
    """Attempt to overtake target car using racing systems.
    
    This behavior uses DRS and ERS systems for overtaking maneuvers.
    
    Args:
        target_car: The car to overtake
        aggressive: If True, use all available systems (DRS, ERS)
    """
    
    # Close the gap
    while (distance from self to target_car) > 5:
        do FollowRacingLineMPCBehavior(target_speed=35)
    
    # Execute overtake with racing systems
    if aggressive:
        take ERSDeployAction(mode='overtake', amount=1.0)
        take DRSAction(activate=True)
    
    # Move to side and accelerate
    take SetThrottleAction(1.0)
    
    # Complete overtake
    do FollowRacingLineMPCBehavior() until (distance from self to target_car) > 10
    
    # Return to racing line
    do FollowRacingLineMPCBehavior()

behavior DefensiveBehavior():
    """Defend position using racing-specific systems.
    
    This behavior uses traction control and brake bias adjustments
    for defensive driving.
    """
    
    # Adjust racing systems for defense
    take TractionControlAction(level=8)  # More conservative TC
    take BrakeBiasAction(bias=0.6)  # More front bias for stability
    
    # Follow racing line defensively
    do FollowRacingLineMPCBehavior(target_speed=25)


## Decision tree behaviors (for race decision engine integration)

behavior FlagBasedSpeedBehavior(speed_type="green", speed_limit=None, manage_gears=True):
    """Set speed based on flag type (decision tree behavior).
    
    This behavior sets the speed limit based on race flags (yellow, green, etc.)
    and applies it to the vehicle.
    
    Args:
        speed_type: Speed type string - "yellow", "double_yellow", "green", "round", etc.
        speed_limit: Speed limit in m/s (if None, uses default for speed_type)
    """
    
    # Set speed limit based on type
    if speed_limit is None:
        # Default speeds (can be overridden with params)
        speed_limits = {
            "pit_crawl": 10.0,
            "pit_lane": 20.0,
            "pit_road": 25.0,
            "yellow": 40.0,
            "double_yellow": 90.0,
            "green": 120.0,
            "round": 120.0,
            "stop": 0.0
        }
        speed_limit = speed_limits.get(speed_type, 120.0)
    
    take SetSpeedLimitAction(speed_limit=speed_limit, speed_type=speed_type)
    
    # Apply speed limit via FollowRacingLineMPCBehavior
    do FollowRacingLineMPCBehavior(target_speed=speed_limit, manage_gears=manage_gears)


behavior LaneSelectionBehavior(ttl_selection="race", manage_gears=True):
    """Select TTL based on attacker/defender flags (decision tree behavior).
    
    This behavior selects the appropriate TTL (left for defender, right for attacker,
    race for optimal) and sets the speed accordingly.
    
    Args:
        ttl_selection: TTL selection string - "left", "right", "race", "optimal", or "pit"
    """
    
    take SetTTLSelectionAction(selection=ttl_selection)
    
    # Set speed based on selection (green speed for racing, slower for pit)
    if ttl_selection == "pit":
        do FlagBasedSpeedBehavior(speed_type="pit_lane", speed_limit=20.0, manage_gears=manage_gears)
    else:
        do FlagBasedSpeedBehavior(speed_type="green", speed_limit=120.0, manage_gears=manage_gears)


behavior ARTStackControlBehavior():
    """Wrapper behavior: do not control the vehicle; let the ART stack control it.
    
    Use this when the ego is driven by the ART (Automated Racing Technology) stack
    (e.g. VKS, race decision engine) instead of Scenic controllers. The behavior
    runs every step but does not send throttle, brake, or steering commands, so
    Scenic does not overwrite ART's control outputs.
    
    Similar in spirit to other shell/wrapper behaviors (e.g. LaneSelectionBehavior
    that delegates to FlagBasedSpeedBehavior).
    """
    while True:
        wait


behavior StopBehavior(stop_type="safe"):
    """Stop car with specified stop type (decision tree behavior).
    
    This behavior implements emergency, immediate, or safe stop behavior.
    
    Args:
        stop_type: Stop type string - "emergency", "immediate", or "safe"
    """
    
    take StopCarAction(stop_type=stop_type)
    take SetTargetGapAction(gap=0.0, gap_type="no_gap")


behavior FollowModeBehavior(target_car, target_gap=31.0, manage_gears=True, use_waypoints=True, lookahead=20.0):
    """Follow another car maintaining target gap (decision tree behavior).
    
    This behavior implements follow mode strategy where the car maintains
    a target gap distance to the car ahead.
    
    Args:
        target_car: The car to follow
        target_gap: Target gap distance in meters
    """
    
    take SetStrategyAction(strategy_type="follow_mode")
    take SetTargetGapAction(gap=target_gap, gap_type="attacker_preparing")
    
    # Get controllers
    _lon_controller, _lat_controller = simulation().getRacingControllers(self)
    past_steer_angle = 0
    gear_up_thresholds = [0.0, 12.0, 22.0, 32.0, 42.0, 52.0]
    gear_down_thresholds = [0.0, 9.0, 18.0, 28.0, 38.0, 48.0]
    
    # Waypoint state (nearest index), used only if waypoints are available
    wp_last_idx = 0
    
    while True:
        # Compute gap to target car
        current_gap = distance from self to target_car
        
        # Compute speed error based on gap
        gap_error = current_gap - target_gap
        
        # Adjust speed to maintain gap
        if gap_error > 5.0:  # Too far, speed up
            target_speed = (target_car.speed if target_car.speed is not None else 0) + 2.0
        elif gap_error < -5.0:  # Too close, slow down
            target_speed = (target_car.speed if target_car.speed is not None else 0) - 2.0
        else:
            target_speed = target_car.speed if target_car.speed is not None else 0
        
        # Clamp to max speed
        target_speed = min(target_speed, self.maxSpeed if hasattr(self, 'maxSpeed') else 120.0)
        
        # Get TTL to follow
        line = (self.ttl if hasattr(self, 'ttl') and self.ttl is not None else (track.racingLine if hasattr(track, 'racingLine') and track.racingLine else mainRacingRoad))
        
        # Cross-track error (waypoint-targeted if available)
        cte = None
        wp_list = (self.waypoints if hasattr(self, 'waypoints') else None)
        if use_waypoints and wp_list and len(wp_list) >= 2:
            px = float(self.position.x); py = float(self.position.y)
            nearest_idx = 0; best_d2 = 1e18
            for i in range(max(0, wp_last_idx - 25), min(len(wp_list), wp_last_idx + 26)):
                wx, wy = float(wp_list[i][0]), float(wp_list[i][1])
                dx = px - wx; dy = py - wy
                d2 = dx*dx + dy*dy
                if d2 < best_d2:
                    best_d2 = d2; nearest_idx = i
            wp_last_idx = nearest_idx
            Ld = float(lookahead)
            tgt_idx = nearest_idx; rem = Ld; j = nearest_idx
            n_wp_la = len(wp_list)
            steps = 0
            while rem > 0.0 and steps < n_wp_la:
                j_next = (j + 1) % n_wp_la
                x0, y0 = float(wp_list[j][0]), float(wp_list[j][1])
                x1, y1 = float(wp_list[j_next][0]), float(wp_list[j_next][1])
                seg_dx = x1 - x0; seg_dy = y1 - y0
                seg_len = (seg_dx*seg_dx + seg_dy*seg_dy) ** 0.5
                if seg_len <= 1e-6:
                    j = j_next; steps += 1; continue
                if rem <= seg_len:
                    u = rem / seg_len
                    # projection for signed error
                    wx = px - x0; wy = py - y0
                    u_proj = (wx*seg_dx + wy*seg_dy) / (seg_len*seg_len)
                    if u_proj < 0.0: u_proj = 0.0
                    if u_proj > 1.0: u_proj = 1.0
                    qx = x0 + u_proj * seg_dx; qy = y0 + u_proj * seg_dy
                    nx = -seg_dy / seg_len; ny = seg_dx / seg_len
                    cte = (px - qx)*nx + (py - qy)*ny
                    break
                else:
                    rem -= seg_len; j = j_next; tgt_idx = j; steps += 1
            if cte is None:
                # Closed loop: last segment index is len(wp_list)-1; next wp is (k0+1)%n_wp
                n_wp_fb = len(wp_list)
                k0 = max(0, min(n_wp_fb - 1, wp_last_idx))
                j1 = (k0 + 1) % n_wp_fb
                x0, y0 = float(wp_list[k0][0]), float(wp_list[k0][1])
                x1, y1 = float(wp_list[j1][0]), float(wp_list[j1][1])
                seg_dx = x1 - x0; seg_dy = y1 - y0
                seg_len = (seg_dx*seg_dx + seg_dy*seg_dy) ** 0.5
                if seg_len <= 1e-6:
                    cte = 0.0
                else:
                    wx = px - x0; wy = py - y0
                    u_proj = (wx*seg_dx + wy*seg_dy) / (seg_len*seg_len)
                    if u_proj < 0.0: u_proj = 0.0
                    if u_proj > 1.0: u_proj = 1.0
                    qx = x0 + u_proj * seg_dx; qy = y0 + u_proj * seg_dy
                    nx = -seg_dy / seg_len; ny = seg_dx / seg_len
                    cte = (px - qx)*nx + (py - qy)*ny
        if cte is None:
            cte = line.signedDistanceTo(self.position)
        current_speed = (self.speed if self.speed is not None else 0)
        speed_error = target_speed - current_speed

        if manage_gears and hasattr(self, 'setGear'):
            current_gear = getattr(self, 'gear', None)
            if current_gear is None or current_gear < 1:
                take SetGearAction(1)
                self.gear = 1
                current_gear = 1
            elif current_speed is not None:
                if current_gear < 6 and current_speed > gear_up_thresholds[current_gear]:
                    take SetGearAction(current_gear + 1)
                    self.gear = current_gear + 1
                    current_gear = self.gear
                elif current_gear > 1 and current_speed < gear_down_thresholds[current_gear - 1]:
                    take SetGearAction(current_gear - 1)
                    self.gear = current_gear - 1
                    current_gear = self.gear
        
        throttle = _lon_controller.run_step(speed_error)
        steer = _lat_controller.run_step(cte)
        
        take RegulatedControlAction(throttle, steer, past_steer_angle)
        past_steer_angle = steer


behavior PitLaneBehavior(manage_gears=True):
    """Handle pit lane speeds (decision tree behavior).
    
    This behavior implements pit lane speed limits: pit crawl (10 m/s),
    pit lane (20 m/s), and pit road (25 m/s).
    """
    
    # Determine which pit zone we're in (simplified - would need location detection)
    # For now, use pit_lane speed as default
    take SetSpeedLimitAction(speed_limit=20.0, speed_type="pit_lane")
    take SetTTLSelectionAction(selection="pit")
    take SetTargetGapAction(gap=0.0, gap_type="no_gap")
    
    # Apply pit lane speed
    do FollowRacingLineMPCBehavior(target_speed=20.0, manage_gears=manage_gears)


behavior SimpleRaceBehavior(manage_gears=True, use_waypoints=True,
                           out_of_bounds_tolerance=5.0):
    """Simplified race decision tree behavior.
    
    Priority-based decision making:
    1. Emergency stop (if out of bounds)
    2. Pit lane behavior (if in pit lane)
    3. Green flag behavior (normal racing)
    
    Args:
        manage_gears: Whether to automatically manage gears
        use_waypoints: Whether to use waypoint-based steering for :obj:`FollowRacingLineMPCBehavior`
        out_of_bounds_tolerance: Distance tolerance for out-of-bounds check (meters)
    """
    
    while True:
        # ============================================================
        # PRIORITY 1: Emergency Stop Check (Out of Bounds)
        # ============================================================
        
        # Check if car is still within track bounds
        # Option 1: Check if position is in road region
        is_in_bounds = road.contains(self.position) if hasattr(road, 'contains') else True
        
        # Option 2: Check distance to road (more lenient)
        if not is_in_bounds:
            # Check if we're close enough to road (within tolerance)
            distance_to_road = road.distanceTo(self.position) if hasattr(road, 'distanceTo') else 0.0
            is_in_bounds = distance_to_road <= out_of_bounds_tolerance
        
        # Emergency stop if out of bounds
        if not is_in_bounds:
            take StopCarAction(stop_type="emergency")
            take SetTargetGapAction(gap=0.0, gap_type="no_gap")
            # Emergency stop - exit behavior
            break
        
        # ============================================================
        # PRIORITY 2: Pit Lane vs Green Flag
        # ============================================================
        
        # Check if we're in pit lane
        in_pit_lane = False
        if hasattr(track, 'pitLaneRoad') and track.pitLaneRoad:
            in_pit_lane = track.pitLaneRoad.contains(self.position)
        
        if in_pit_lane:
            # PIT LANE BEHAVIOR
            take SetSpeedLimitAction(speed_limit=20.0, speed_type="pit_lane")
            take SetTTLSelectionAction(selection="pit")
            take SetTargetGapAction(gap=0.0, gap_type="no_gap")
            take SetStrategyAction(strategy_type="cruise_control")
            
            # Execute pit lane behavior
            do FollowRacingLineMPCBehavior(target_speed=20.0, manage_gears=manage_gears,
                                       use_waypoints=use_waypoints)
        else:
            # GREEN FLAG BEHAVIOR (Normal Racing)
            green_speed = 120.0  # Default green speed (m/s)
            take SetSpeedLimitAction(speed_limit=green_speed, speed_type="green")
            take SetTTLSelectionAction(selection="race")  # Use race TTL
            take SetStrategyAction(strategy_type="cruise_control")
            
            # Execute green flag behavior
            do FollowRacingLineMPCBehavior(target_speed=green_speed, manage_gears=manage_gears,
                                       use_waypoints=use_waypoints)
        
        wait  # Wait one timestep before re-evaluating