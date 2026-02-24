"""Vehicle controller for dSPACE simulator.

This module handles the application of control commands to vehicles in the
dSPACE simulation environment, including both ego and fellow vehicles.
"""

from ..vehicle.physics import VehiclePhysicsState

print(f"[PatchID] controller.py loaded from {__file__}")


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
        self.cd = simulation._cd

        # --- Dedup / COM-write optimization state ---
        self._write_tick = 0
        self._last_written_cmd = {}

        # path -> {"attempts": int, "skips": int, "execs": int}
        self._dedup_stats = {}
        self._dedup_stats_print_every = 50  # print every N ego control ticks
        self._dedup_banner_printed = False

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
        if not self._dedup_banner_printed:
            self._dedup_banner_printed = True
            print(f"[PatchID] controller dedup active from {__file__}")

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
            print("[DedupStats] " + " | ".join(parts))

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
            throttle/brake/steering → VesiInterface → physics engine.

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
                # Control-time (based on configured control period if available)
                ctrl_period = getattr(self.simulation, "control_period", None)
                if ctrl_period is None or self._safe_float(ctrl_period, 0.0) <= 0.0:
                    ctrl_period = float(getattr(self.simulation, "timestep", 0.0))
                else:
                    ctrl_period = float(ctrl_period)
                t_ctrl = obj._ego_control_count * ctrl_period

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
                    f"[EgoControl] t_ctrl={t_ctrl:.2f}s sim_t={t_sim:.2f}s "
                    f"step={sim_step_idx} #{obj._ego_control_count} Writing: "
                    f"throttle={throttle_scenic_f:.3f}->{throttle_scenic_f*100:.1f}, "
                    f"brake={brake_scenic_f:.3f}->{brake_scenic_f*100:.1f}, "
                    f"steer_rad={delta_rad_dbg:.4f}->{steer_deg_dbg:.1f}deg"
                )

                # Dedup stats summary
                if obj._ego_control_count % max(1, int(self._dedup_stats_print_every)) == 0:
                    self._print_dedup_stats()

        except Exception as e:
            print(f"[VehicleController:EgoControl] Error: {e}")
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
                        self.cd.set_var(self.KEY_GEAR, gear_int)
                        if obj._ego_control_count <= 5 or obj._ego_control_count % 50 == 0:
                            print(f"[EgoControl] Setting gear to {gear_int}")
                    elif action_type == "clutch":
                        clutch_pct = float(value * 100.0)
                        self.cd.set_var(self.KEY_CLUTCH, clutch_pct)
                        print(f"[EgoControl] Setting clutch to {clutch_pct}%")
                except Exception as e:
                    print(f"[VehicleController:EgoControl] {action_type} error: {e}")
                    import traceback
                    traceback.print_exc()

    # -------------------------------------------------------------------------
    # Fellow control
    # -------------------------------------------------------------------------
    def apply_fellow_control(self, obj):
        """Apply kinematic control for fellow vehicle using physics model.

        Fellows use kinematic control: throttle/brake/steering → physics model
        → velocity/deviation. The physics model computes realistic motion, then
        velocity and lateral deviation are written to ControlDesk External_Signals.
        """
        if not hasattr(obj, "_control_state") or not obj._control_state:
            return

        # Ensure fellow arrays are initialized before attempting to write
        from ..controldesk.arrays import ensure_fellow_arrays_initialized

        ensure_fellow_arrays_initialized(self.simulation)

        # Get fellow index
        fellow_index = self.get_fellow_index(obj)
        if fellow_index is None:
            print(f"[VehicleController:FellowControl] Could not determine index for {obj}")
            return

        # Adjust for base (0-based vs 1-based arrays) for writing
        eff_index = fellow_index + (self.simulation._fellow_index_base or 0)
        control = obj._control_state

        # Extract controls (default to 0 if not present)
        throttle = float(control.get("throttle", 0.0))
        brake = float(control.get("braking", 0.0))
        steering = float(control.get("steering", 0.0))

        # Fellow physics expects steering in [-1, 1]. Convert from rad if MPC path.
        if getattr(obj, "_racing_steer_units", None) == "rad":
            from scenic.domains.racing.constants import DELTA_MAX_RAD

            steering = max(-1.0, min(1.0, steering / DELTA_MAX_RAD))

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
                print(f"[Fellow {fellow_index}] Physics model created (initial velocity=0.0 m/s)")

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

            if obj._fellow_control_count <= 3:
                print(
                    f"[Fellow {fellow_index} Physics] Synced velocity: "
                    f"{old_physics_velocity:.2f} → {actual_speed:.2f} m/s"
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

            if obj._fellow_control_count == 1:
                # First step: maintain current deviation
                new_velocity = actual_speed
                new_deviation = actor.physics.deviation
            else:
                # Sync deviation with actual position before update
                try:
                    plant_base = (
                        "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/"
                        "FellowMovement/FELLOW_POS_VEL/FellowTrailer"
                    )
                    x_arr = self.cd.get_var(f"{plant_base}/x")
                    y_arr = self.cd.get_var(f"{plant_base}/y")
                    x_rd = x_arr[eff_index] if isinstance(x_arr, (list, tuple)) and eff_index < len(x_arr) else None
                    y_rd = y_arr[eff_index] if isinstance(y_arr, (list, tuple)) and eff_index < len(y_arr) else None

                    if x_rd is not None and y_rd is not None and self.simulation._road_index:
                        from ..utils.legacy import project_world_to_st

                        _, t_actual = project_world_to_st(
                            self.simulation._road_index,
                            (float(x_rd), float(y_rd)),
                        )
                        actor.physics.deviation = float(t_actual)
                except Exception:
                    pass

                # Physics Update
                new_velocity, new_deviation = actor.physics.update(
                    throttle=throttle,
                    brake=brake,
                    steering=steering,
                    dt=self.simulation.timestep,
                )

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

            # Debug log
            if obj._fellow_control_count % 50 == 0:
                print(f"[Fellow {fellow_index}] Step {obj._fellow_control_count}")
                print(f"  Physics: v={v_value:.1f} km/h, d={d_value:.2f} m")
                print("  Written to: ...External_Signals/Const_v... and .../Const_d...")

        except Exception as e:
            error_msg = str(e)
            if "Index was outside the bounds" not in error_msg:
                print(f"[VehicleController:FellowControl] Error: {e}")
                import traceback
                traceback.print_exc()

    def get_fellow_index(self, obj):
        """Get the array index for a fellow vehicle (0-based)."""
        return self.simulation._getFellowIndex(obj)