"""Vehicle controller for dSPACE simulator.

This module handles the application of control commands to vehicles in the
dSPACE simulation environment, including both ego and fellow vehicles.
"""

import logging
import math

from scenic.domains.racing.fellow.plant import is_fellow_vd_plant_behavior

from ..vehicle.physics import VehiclePhysicsState

logger = logging.getLogger(__name__)
from ..modeldesk.placement import t_for_dspace_lateral

# Fellow plant: log first N control ticks, then every Kth (INFO).
_FELLOW_LOG_INITIAL = 3
_FELLOW_LOG_INTERVAL = 50


def _fellow_plant_outputs_ready(obj) -> bool:
    st = getattr(obj, "_fellow_plant_state", None)
    return (
        isinstance(st, dict)
        and st.get("v_kmh") is not None
        and st.get("d_m") is not None
    )


class VehicleController:
    """Controller for applying vehicle commands to ControlDesk.

    This class handles the translation from Scenic control commands to
    ControlDesk variable writes, supporting both:
    - Ego vehicle: VesiInterface physics-based control
    - Fellow vehicles: Kinematic control via external signals

    Attributes:
        simulation: Reference to parent DSpaceSimulation instance
        cd: ControlDesk connection object
    """

    # ControlDesk paths for ego manual inputs
    KEY_THROTTLE = (
        "Platform()://ASM_Traffic/Model Root/VesiInterface/"
        "VESIResultData_Manual/vehicle_inputs/Const_throttle_cmd/Value"
    )
    KEY_BRAKE_FRONT = (
        "Platform()://ASM_Traffic/Model Root/VesiInterface/"
        "VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_front/Value"
    )
    KEY_BRAKE_REAR = (
        "Platform()://ASM_Traffic/Model Root/VesiInterface/"
        "VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_rear/Value"
    )
    KEY_STEERING = (
        "Platform()://ASM_Traffic/Model Root/VesiInterface/"
        "VESIResultData_Manual/vehicle_inputs/Const_steering_cmd/Value"
    )
    KEY_GEAR = (
        "Platform()://ASM_Traffic/Model Root/VesiInterface/"
        "VESIResultData_Manual/vehicle_inputs/Const_gear_cmd/Value"
    )
    KEY_CLUTCH = (
        "Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel/"
        "ExternalUserData/Pos_ClutchPedal[%]/Value"
    )

    def __init__(self, simulation):
        """Initialize the vehicle controller.

        Args:
            simulation: The parent DSpaceSimulation instance
        """
        self.simulation = simulation
        # Variable read/write: MAPort if available, else ControlDesk COM
        self.cd = getattr(simulation, "_var_access", None) or simulation._cd

        # --- Dedup / COM-write optimization state ---
        self._write_tick = 0
        self._last_written_cmd = {}

        # path -> {"attempts": int, "skips": int, "execs": int}
        self._dedup_stats = {}
        self._dedup_stats_print_every = 50  # log every N ego control ticks

        # Epsilons for dedup comparison (after scaling into CD units)
        self._eps_throttle = 1e-4
        self._eps_brake = 1e-6
        self._eps_steer = 1e-4

        # Force refresh every N ego control ticks even if value unchanged
        self._dedup_heartbeat_every = 20

    # -------------------------------------------------------------------------
    # Dedup helper methods
    # -------------------------------------------------------------------------
    def _dedup_stat_inc(self, path, key, inc=1):
        if path not in self._dedup_stats:
            self._dedup_stats[path] = {"attempts": 0, "skips": 0, "execs": 0}
        self._dedup_stats[path][key] += inc

    def _normalize_for_dedup(self, value, *, zero_snap=1e-6, round_digits=5):
        """Normalize values before dedup comparison to reduce float jitter."""
        v = float(value)
        if abs(v) < zero_snap:
            v = 0.0
        # Rounding helps avoid tiny noise preventing dedup
        return round(v, round_digits)

    def _maybe_write_cd(self, path, value, eps, heartbeat=False):
        """Write a value to ControlDesk only if changed or heartbeat forces it.

        Returns:
            bool: True if write executed, False if skipped by dedup.
        """
        self._dedup_stat_inc(path, "attempts", 1)

        v = self._normalize_for_dedup(value)
        last = self._last_written_cmd.get(path, None)

        force_write = bool(heartbeat)
        skip = False
        if (last is not None) and (not force_write):
            if abs(v - last) <= float(eps):
                skip = True

        if skip:
            self._dedup_stat_inc(path, "skips", 1)
            return False

        self.cd.set_var(path, v)
        self._last_written_cmd[path] = v
        self._dedup_stat_inc(path, "execs", 1)
        return True

    def _print_dedup_stats(self):
        """Print periodic dedup stats for key ego command paths."""
        key_order = [
            self.KEY_THROTTLE,
            self.KEY_STEERING,
            self.KEY_BRAKE_FRONT,
            self.KEY_BRAKE_REAR,
        ]
        labels = {
            self.KEY_THROTTLE: "throttle",
            self.KEY_STEERING: "steer",
            self.KEY_BRAKE_FRONT: "brake_front",
            self.KEY_BRAKE_REAR: "brake_rear",
        }

        parts = []
        printed = set()
        for p in key_order:
            if p in self._dedup_stats:
                st = self._dedup_stats[p]
                parts.append(
                    f"{labels.get(p, p)}: a={st['attempts']} s={st['skips']} e={st['execs']}"
                )
                printed.add(p)

        # Print any additional paths too (unlikely, but useful)
        for p, st in self._dedup_stats.items():
            if p in printed:
                continue
            parts.append(f"{p}: a={st['attempts']} s={st['skips']} e={st['execs']}")

        if parts:
            logger.debug("[DedupStats] %s", " | ".join(parts))

    @staticmethod
    def _safe_float(v, default=0.0):
        try:
            return float(v)
        except Exception:
            return float(default)

    # -------------------------------------------------------------------------
    # Ego control
    # -------------------------------------------------------------------------
    def apply_ego_control(self, obj):
        """Apply VesiInterface control for ego vehicle.

        Ego uses physics-based control:
            throttle/brake/steering -> VesiInterface -> physics engine.

        Control inputs are written to VesiInterface manual control paths which
        feed into the VEOS vehicle dynamics model.
        """
        control = getattr(obj, "_control_state", None)

        # Dedup heartbeat tick increments once per ego-control update
        self._write_tick += 1
        heartbeat = (self._write_tick % self._dedup_heartbeat_every == 0)

        # Track for debug
        if not hasattr(obj, "_ego_control_count"):
            obj._ego_control_count = 0
        obj._ego_control_count += 1

        try:
            throttle_scenic = control.get("throttle", 0.0) if control else 0.0
            brake_scenic = control.get("braking", 0.0) if control else 0.0
            steer_scenic = control.get("steering", 0.0) if control else 0.0

            # -----------------------------------------------------------------
            # Throttle: [0,1] -> [0,100]
            # -----------------------------------------------------------------
            if control and ("throttle" in control) and (control["throttle"] is not None):
                throttle_val = float(max(0.0, min(1.0, control["throttle"])) * 100.0)
                self._maybe_write_cd(
                    self.KEY_THROTTLE,
                    throttle_val,
                    eps=self._eps_throttle,
                    heartbeat=heartbeat,
                )

            # -----------------------------------------------------------------
            # Brake: [0,1] -> [0,10000] to front + rear
            # ControlDesk expects 0..10000, not 0..100
            # -----------------------------------------------------------------
            if control and ("braking" in control) and (control["braking"] is not None):
                brake_val = float(max(0.0, min(1.0, control["braking"])) * 10000.0)

                # Dedup front and rear independently so stats reflect each path
                self._maybe_write_cd(
                    self.KEY_BRAKE_FRONT,
                    brake_val,
                    eps=self._eps_brake,
                    heartbeat=heartbeat,
                )
                self._maybe_write_cd(
                    self.KEY_BRAKE_REAR,
                    brake_val,
                    eps=self._eps_brake,
                    heartbeat=heartbeat,
                )

            # -----------------------------------------------------------------
            # Steering:
            # - MPC path: _racing_steer_units == 'rad'  => road wheel angle [rad]
            # - PID path: normalized [-1,1]             => convert to rad first
            # -----------------------------------------------------------------
            if control and ("steering" in control) and (control["steering"] is not None):
                from ..steer_io import road_rad_to_dspace_value
                from scenic.domains.racing.constants import DELTA_MAX_RAD

                steer_raw = float(control["steering"])
                if getattr(obj, "_racing_steer_units", None) == "rad":
                    delta_rad = steer_raw
                else:
                    delta_rad = max(
                        -DELTA_MAX_RAD,
                        min(DELTA_MAX_RAD, steer_raw * DELTA_MAX_RAD),
                    )

                steer_val = road_rad_to_dspace_value(delta_rad)
                self._maybe_write_cd(
                    self.KEY_STEERING,
                    steer_val,
                    eps=self._eps_steer,
                    heartbeat=heartbeat,
                )

            # Debug every 50 ego control ticks (control-time + sim-time alignment)
            if obj._ego_control_count % 50 == 0:
                # Control-time (prefer actual control period; support controlPeriod or control_period)
                ctrl_period = getattr(self.simulation, "control_period", None)
                if ctrl_period is None:
                    ctrl_period = getattr(self.simulation, "controlPeriod", None)
                if ctrl_period is None or float(ctrl_period) <= 0:
                    ctrl_period = 0.05  # fallback for your current experiment
                t_ctrl = obj._ego_control_count * float(ctrl_period)

                # Sim-time from simulation step index
                sim_step_idx = int(getattr(self.simulation, "currentTime", 0))
                t_sim = sim_step_idx * float(getattr(self.simulation, "timestep", 0.0))

                # Recompute steering conversion for debug readability
                from ..steer_io import road_rad_to_dspace_value
                from scenic.domains.racing.constants import DELTA_MAX_RAD

                steer_raw_dbg = self._safe_float(steer_scenic, 0.0)
                if getattr(obj, "_racing_steer_units", None) == "rad":
                    delta_rad_dbg = steer_raw_dbg
                else:
                    delta_rad_dbg = max(
                        -DELTA_MAX_RAD,
                        min(DELTA_MAX_RAD, steer_raw_dbg * DELTA_MAX_RAD),
                    )
                steer_deg_dbg = road_rad_to_dspace_value(delta_rad_dbg)

                throttle_scenic_f = self._safe_float(throttle_scenic, 0.0)
                brake_scenic_f = self._safe_float(brake_scenic, 0.0)

                print(
                    f"[VesiPlant ego] t_ctrl={t_ctrl:.2f}s sim_t={t_sim:.2f}s "
                    f"step={sim_step_idx} #{obj._ego_control_count} Writing: "
                    f"throttle={throttle_scenic_f:.3f}->{throttle_scenic_f*100:.1f}, "
                    f"brake={brake_scenic_f:.3f}->{brake_scenic_f*100:.1f}, "
                    f"steer_rad={delta_rad_dbg:.4f}->{steer_deg_dbg:.1f}deg"
                )

                # Dedup stats summary
                if obj._ego_control_count % max(1, int(self._dedup_stats_print_every)) == 0:
                    self._print_dedup_stats()

        except Exception as e:
            print(f"[VehicleController:VesiPlant ego] Error: {e}")
            import traceback
            traceback.print_exc()

        # ---------------------------------------------------------------------
        # One-shot actions (gear, clutch)
        # ---------------------------------------------------------------------
        if hasattr(obj, "_oneshot_actions") and obj._oneshot_actions:
            for action_type, value in obj._oneshot_actions:
                try:
                    if action_type == "gear":
                        gear_int = max(0, min(6, int(value)))
                        if self._maybe_write_cd(self.KEY_GEAR, gear_int, 0.0):
                            if obj._ego_control_count <= 5 or obj._ego_control_count % 50 == 0:
                                print(f"[VesiPlant ego] Setting gear to {gear_int}")
                    elif action_type == "clutch":
                        clutch_pct = float(value * 100.0)
                        if self._maybe_write_cd(self.KEY_CLUTCH, clutch_pct, 1e-6):
                            print(f"[VesiPlant ego] Setting clutch to {clutch_pct}%")
                except Exception as e:
                    print(f"[VehicleController:VesiPlant ego] {action_type} error: {e}")
                    import traceback
                    traceback.print_exc()
            obj._oneshot_actions.clear()

    # -------------------------------------------------------------------------
    # Fellow control
    # -------------------------------------------------------------------------
    def _write_fellow_plant_external_signals(self, obj, fellow_index, eff_index):
        """Write fellow plant commands to Const_v / Const_d bulk arrays from ``_fellow_plant_state``."""
        st = getattr(obj, "_fellow_plant_state", None)
        if not isinstance(st, dict) or st.get("v_kmh") is None or st.get("d_m") is None:
            if not getattr(obj, "_fellow_plant_incomplete_warned", False):
                logger.warning(
                    "FellowControl: expected _fellow_plant_state with v_kmh and d_m on %s; skip write",
                    obj,
                )
                obj._fellow_plant_incomplete_warned = True
            return
        v_value = st["v_kmh"]
        d_cmd = st["d_m"]

        v_value = float(v_value)
        d_cmd = float(d_cmd)

        base_ext = (
            "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/"
            "FellowMovement/External_Signals"
        )
        v_path_bulk = f"{base_ext}/Const_v_Fellows_External[km|h]/Value"
        d_path_bulk = f"{base_ext}/Const_d_Fellows_External[m]/Value"

        try:
            v_arr = list(self.cd.get_var(v_path_bulk) or [])
            d_arr = list(self.cd.get_var(d_path_bulk) or [])
        except Exception:
            v_arr = []
            d_arr = []

        need_len = eff_index + 1
        if len(v_arr) < need_len:
            v_arr.extend([0.0] * (need_len - len(v_arr)))
        if len(d_arr) < need_len:
            d_arr.extend([0.0] * (need_len - len(d_arr)))

        v_arr[eff_index] = v_value
        d_arr[eff_index] = t_for_dspace_lateral(d_cmd)

        self.cd.set_var(v_path_bulk, v_arr)
        self.cd.set_var(d_path_bulk, d_arr)
        obj._fellow_plant_incomplete_warned = False

        if not hasattr(obj, "_fellow_control_count"):
            obj._fellow_control_count = 0
        obj._fellow_control_count += 1
        c = obj._fellow_control_count
        if c <= _FELLOW_LOG_INITIAL or c % _FELLOW_LOG_INTERVAL == 0:
            mode = getattr(obj, "_fellow_plant_log_mode", "plant")
            wp_i = int(getattr(obj, "_fellow_geo_wp_last_idx", -1))
            if mode == "placement_t":
                logger.info(
                    "[Fellow %s] constant_plant v=%.1f km/h d=%.3f m (placement t)",
                    fellow_index,
                    v_value,
                    d_cmd,
                )
            else:
                logger.info(
                    "[Fellow %s] geometric_ttl %s v=%.1f km/h d_cmd=%.3f m wp_idx=%s",
                    fellow_index,
                    mode,
                    v_value,
                    d_cmd,
                    wp_i,
                )

    def apply_fellow_control(self, obj):
        """Apply control for fellow vehicle via v and d only.

        Fellows are controlled by (v, d) on External_Signals. Longitudinal v
        follows throttle/brake integration (same as before).

        **Lateral (Lap + optimal TTL)**: d commands the racing line expressed in
        centerline coordinates: δ(s) from the optimal polyline vs main
        centerline, plus feedback so the plant converges to that offset—aligned
        with MPC's reference. Legacy bicycle-from-steering is used for Pit route
        or when the delta table cannot be built; set
        ``obj._fellow_force_bicycle_lateral = True`` to force bicycle on Lap.

        Fellow* plant behaviors (class name starts with ``Fellow``): read staged
        ``_fellow_plant_state`` (``v_kmh``, ``d_m``) from
        :class:`~scenic.domains.racing.actions.SetFellowPlantAction` and write fellow
        External_Signals only—no per-behavior Python updaters in the controller.
        """
        # Ensure fellow arrays are initialized before attempting to write
        from ..controldesk.arrays import ensure_fellow_arrays_initialized

        ensure_fellow_arrays_initialized(self.simulation)

        # Get fellow index
        fellow_index = self.get_fellow_index(obj)
        if fellow_index is None:
            logger.warning("FellowControl: could not determine array index for %s", obj)
            return

        # Adjust for base (0-based vs 1-based arrays) for writing
        eff_index = fellow_index + (self.simulation._fellow_index_base or 0)

        # Fellow (v, d) plant: values staged by Scenic actions; controller writes External_Signals only.
        if is_fellow_vd_plant_behavior(obj):
            if _fellow_plant_outputs_ready(obj):
                self._write_fellow_plant_external_signals(obj, fellow_index, eff_index)
            elif not getattr(obj, "_fellow_plant_incomplete_warned", False):
                logger.warning(
                    "FellowControl: incomplete _fellow_plant_state for %s; skip plant write "
                    "(behavior should take SetFellowPlantAction each step)",
                    obj,
                )
                obj._fellow_plant_incomplete_warned = True
            return

        # Kinematic path requires _control_state from behavior
        if not hasattr(obj, "_control_state") or not obj._control_state:
            return

        control = obj._control_state

        # Extract controls (default to 0 if not present)
        throttle = float(control.get("throttle", 0.0))
        brake = float(control.get("braking", 0.0))
        steering = float(control.get("steering", 0.0))

        # When MPC path supplies steering in rad, pass it to physics for bicycle model.
        steering_rad = None
        if getattr(obj, "_racing_steer_units", None) == "rad":
            from scenic.domains.racing.constants import DELTA_MAX_RAD

            steering_rad = steering  # behavior already in rad
            steering = max(-1.0, min(1.0, steering / DELTA_MAX_RAD))  # normalized for legacy

        # CRITICAL: Get the actual CTE (cross-track error) from the behavior
        _ = getattr(obj, "_current_cte", None)  # retained for debugging / future use

        # Track control calls for debug
        if not hasattr(obj, "_fellow_control_count"):
            obj._fellow_control_count = 0
        obj._fellow_control_count += 1

        try:
            # Update physics model
            actor = obj.dspaceActor

            # Ensure physics model exists for kinematic update
            if getattr(actor, "physics", None) is None:
                actor.physics = VehiclePhysicsState(initial_velocity=0.0, initial_deviation=0.0)
                logger.info("[Fellow %s] physics model created (v=0)", fellow_index)

            # Sync physics model with actual velocity from ControlDesk
            actual_speed = 0.0
            try:
                base_path = (
                    "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/"
                    "FellowMovement/FELLOW_POS_VEL/FellowTrailer"
                )
                v_arr = self.cd.get_var(f"{base_path}/v_Fellows")
                if isinstance(v_arr, (list, tuple)) and eff_index < len(v_arr):
                    v_value = v_arr[eff_index]
                    if v_value is not None:
                        # Assuming km/h feedback here (matching your current behavior)
                        actual_speed = float(v_value) / 3.6
            except Exception:
                pass

            # Sync physics velocity
            old_physics_velocity = actor.physics.velocity
            actor.physics.velocity = actual_speed

            if obj._fellow_control_count <= _FELLOW_LOG_INITIAL:
                logger.debug(
                    "[Fellow %s] sync plant v %.2f -> %.2f m/s",
                    fellow_index,
                    old_physics_velocity,
                    actual_speed,
                )

            # Initialize deviation on first step to maintain continuity
            if obj._fellow_control_count == 1:
                initial_deviation = 0.0
                try:
                    base_ext = (
                        "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/"
                        "FellowMovement/External_Signals"
                    )
                    d_path_bulk = f"{base_ext}/Const_d_Fellows_External[m]/Value"
                    d_arr = self.cd.get_var(d_path_bulk)
                    if isinstance(d_arr, (list, tuple)) and eff_index < len(d_arr):
                        initial_deviation = (
                            float(d_arr[eff_index]) if d_arr[eff_index] is not None else 0.0
                        )
                except Exception:
                    initial_deviation = 0.0

                actor.physics.deviation = initial_deviation

            control_interval = getattr(self.simulation, "_control_interval", 1)
            dt = float(self.simulation.timestep) * max(1, control_interval)
            if obj._fellow_control_count == 1:
                # First step: maintain current deviation
                new_velocity = actual_speed
                new_deviation = actor.physics.deviation
            else:
                actor.physics.path_curvature = 0.0
                new_velocity = actual_speed
                new_deviation = actor.physics.deviation
                used_racing_servo = False
                plant_base = (
                    "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/"
                    "FellowMovement/FELLOW_POS_VEL/FellowTrailer"
                )
                try:
                    x_arr = self.cd.get_var(f"{plant_base}/x")
                    y_arr = self.cd.get_var(f"{plant_base}/y")
                    x_rd = x_arr[eff_index] if isinstance(x_arr, (list, tuple)) and eff_index < len(x_arr) else None
                    y_rd = y_arr[eff_index] if isinstance(y_arr, (list, tuple)) and eff_index < len(y_arr) else None

                    if x_rd is None or y_rd is None:
                        actor.physics.update_longitudinal_only(throttle, brake, dt)
                        new_velocity = actor.physics.velocity
                    else:
                        pos_xy = (float(x_rd), float(y_rd))
                        road_index = getattr(self.simulation, "_road_index_ttl", None) or self.simulation._road_index
                        route_pref = getattr(obj, "_route", None)
                        scene = getattr(self.simulation, "scene", None)
                        scene_params = getattr(scene, "params", None) or {}
                        ttl_folder = getattr(obj, "ttlFolder", None) or scene_params.get("ttlFolder")
                        optimal_csv = getattr(obj, "ttlFileName", None) or scene_params.get("ttlFileName")

                        # Racing-line servo: Const_d is centerline lateral; MPC tracks optimal line.
                        # Command d toward delta(s) on centerline that matches the racing line.
                        if (
                            road_index
                            and route_pref == "Lap"
                            and ttl_folder
                            and optimal_csv
                            and not getattr(obj, "_fellow_force_bicycle_lateral", False)
                        ):
                            from .fellow_racing_line_lateral import (
                                _road_index_main_track_only,
                                get_or_build_delta_table,
                                lookup_delta,
                            )

                            tbl = get_or_build_delta_table(
                                self.simulation,
                                str(ttl_folder),
                                str(optimal_csv),
                                road_index,
                            )
                            idx_main = _road_index_main_track_only(road_index)
                            if tbl is not None and idx_main is not None:
                                from ..utils.legacy import project_world_to_st

                                s_meas, t_meas = project_world_to_st(idx_main, pos_xy)
                                s_filt = getattr(obj, "_fellow_s_meas_filtered", None)
                                if s_filt is None:
                                    s_use = float(s_meas)
                                else:
                                    s_use = 0.42 * float(s_meas) + 0.58 * float(s_filt)
                                obj._fellow_s_meas_filtered = s_use
                                s_arr, d_arr, track_len = tbl
                                d_cmd, delta_ref, e_lat = lookup_delta(
                                    s_use, t_meas, s_arr, d_arr, track_len
                                )
                                prev_d = getattr(obj, "_fellow_d_cmd_prev", None)
                                max_slew = 0.32 * max(1.0, dt / 0.05)
                                if prev_d is not None:
                                    d_cmd = max(
                                        prev_d - max_slew,
                                        min(prev_d + max_slew, d_cmd),
                                    )
                                obj._fellow_d_cmd_prev = float(d_cmd)
                                actor.physics.update_longitudinal_only(throttle, brake, dt)
                                new_velocity = actor.physics.velocity
                                new_deviation = d_cmd
                                actor.physics.deviation = d_cmd
                                used_racing_servo = True
                                if obj._fellow_control_count <= _FELLOW_LOG_INITIAL or obj._fellow_control_count % _FELLOW_LOG_INTERVAL == 0:
                                    logger.debug(
                                        "[Fellow %s] racing_servo s=%.1f t=%.3f d_ref=%.3f e=%.3f d_cmd=%.3f",
                                        fellow_index,
                                        s_meas,
                                        t_meas,
                                        delta_ref,
                                        e_lat,
                                        d_cmd,
                                    )

                        if not used_racing_servo and road_index:
                            from ..geometry.route_projection import (
                                path_curvature_at_pos_route_aware,
                                project_world_to_st_route_specific,
                            )

                            actor.physics.path_curvature = path_curvature_at_pos_route_aware(
                                road_index, pos_xy, route_pref
                            )
                            if route_pref:
                                _, t_actual = project_world_to_st_route_specific(
                                    road_index,
                                    pos_xy,
                                    route_preference=route_pref,
                                )
                            else:
                                from ..utils.legacy import project_world_to_st

                                _, t_actual = project_world_to_st(
                                    road_index,
                                    pos_xy,
                                )
                            actor.physics.deviation = float(t_actual)
                            new_velocity, new_deviation = actor.physics.update(
                                throttle=throttle,
                                brake=brake,
                                steering=steering,
                                dt=dt,
                                steering_rad=steering_rad,
                            )
                except Exception:
                    if not used_racing_servo:
                        try:
                            new_velocity, new_deviation = actor.physics.update(
                                throttle=throttle,
                                brake=brake,
                                steering=steering,
                                dt=dt,
                                steering_rad=steering_rad,
                            )
                        except Exception:
                            actor.physics.update_longitudinal_only(throttle, brake, dt)
                            new_velocity = actor.physics.velocity

            # --- WRITE TO EXTERNAL SIGNALS ---
            base_ext = (
                "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/"
                "FellowMovement/External_Signals"
            )
            v_path_bulk = f"{base_ext}/Const_v_Fellows_External[km|h]/Value"
            d_path_bulk = f"{base_ext}/Const_d_Fellows_External[m]/Value"

            # Prepare values
            v_value = float(new_velocity * 3.6)  # m/s -> km/h
            d_value = float(new_deviation)       # m

            # Read-Modify-Write (Bulk Array)
            try:
                v_arr = list(self.cd.get_var(v_path_bulk) or [])
                d_arr = list(self.cd.get_var(d_path_bulk) or [])
            except Exception:
                v_arr = []
                d_arr = []

            # Extend arrays if too short
            need_len = eff_index + 1
            if len(v_arr) < need_len:
                v_arr.extend([0.0] * (need_len - len(v_arr)))
            if len(d_arr) < need_len:
                d_arr.extend([0.0] * (need_len - len(d_arr)))

            # Update specific index
            v_arr[eff_index] = v_value
            d_arr[eff_index] = d_value

            # Write back
            self.cd.set_var(v_path_bulk, v_arr)
            self.cd.set_var(d_path_bulk, d_arr)

            step = obj._fellow_control_count
            log_this_step = step <= _FELLOW_LOG_INITIAL or step % _FELLOW_LOG_INTERVAL == 0
            if log_this_step:
                psi_e_rad = getattr(actor.physics, "heading_error", 0.0)
                psi_e_deg = math.degrees(psi_e_rad)
                steer_rad_str = f"{steering_rad:.3f} rad" if steering_rad is not None else "norm"
                logger.info(
                    "[Fellow %s] mpc_plant step=%s thr=%.2f brk=%.2f %s v=%.2f m/s d=%.3f m psi_e=%.1f deg",
                    fellow_index,
                    step,
                    throttle,
                    brake,
                    steer_rad_str,
                    new_velocity,
                    new_deviation,
                    psi_e_deg,
                )
                if logger.isEnabledFor(logging.DEBUG):
                    delta_rad = getattr(actor.physics, "_last_delta_rad", None)
                    yaw_rate = getattr(actor.physics, "_last_yaw_rate", None)
                    d_dot = getattr(actor.physics, "_last_d_dot", None)
                    acc = getattr(actor.physics, "_last_acceleration", None)
                    if delta_rad is not None and yaw_rate is not None and d_dot is not None:
                        logger.debug(
                            "[Fellow %s] bicycle delta=%.3f r=%.3f d_dot=%.3f a=%.2f",
                            fellow_index,
                            delta_rad,
                            yaw_rate,
                            d_dot,
                            acc or 0.0,
                        )

        except Exception as e:
            error_msg = str(e)
            if "Index was outside the bounds" not in error_msg:
                logger.exception("FellowControl: %s", e)

    def get_fellow_index(self, obj):
        """Get the array index for a fellow vehicle (0-based)."""
        return self.simulation._getFellowIndex(obj)