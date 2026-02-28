# -*- coding: utf-8 -*-
# Scenic → dSPACE (ModelDesk) absolute placement:
# - SaveAs/Activate first (desired)
# - Build XODR reference index from Scenic param `map`
# - For each Scenic object: (x,y) → (s,t), then seg0 uses absolute Position/Deviation

import math
import os
import time
import pythoncom
from win32com.client import Dispatch

from scenic.domains.racing.simulators import RacingSimulator, RacingSimulation
from scenic.core.simulators import SimulationCreationError

from .utils import legacy as dutils
from .controldesk.per_tick_control import ExternalControlManager
from .vehicle import VehiclePhysicsState, VehicleController
from .ttl.loader import get_ttl_config, load_ttl_region, attach_to_ego, attach_ttl
from .controldesk.arrays import ensure_fellow_arrays_initialized, probe_external_index_base
from .controldesk.readback import read_ego_state, read_fellow_state
from .vehicle.actor import ensure_actor, DSpaceVehicleActor
from .vehicle.indexing import get_fellow_index as indexing_get_fellow_index
from .geometry.pipeline import build_road_index_and_transform
from .modeldesk.authoring import author_scenario, configure_fellow
from .modeldesk.placement import place_ego, place_fellow
from .geometry.route_mapping import detect_track_segment, assign_route_for_segment
from .modeldesk.routes import set_route as routes_set_route
from .controldesk import session as cd_session
from .geometry.params import get_map_path
from .steer_io import road_rad_to_dspace_value, log_startup_once, DELTA_MAX_RAD, THETA_SW_MAX_DEG, R
print(f"[PatchID] simulator.py loaded from {__file__}")

# dSPACE path for simulated time (read on each control step and logged)
SIMULATED_TIME_PATH = "Platform()://ASM_Traffic/Simulation and RTOS/Simulation/SimulationTime"


class DSpaceSimulator(RacingSimulator):
    def __init__(self, *, scenario_src="LagunaSeca_ExternalControl",
                 scenario_name=None, timestep=1, control_period=None, light_step=False, save_as=True):
        super().__init__()
        self.scenario_src = scenario_src
        self.scenario_name = scenario_name
        self.timestep = float(timestep)
        # control_period: seconds between control/readback updates (None = every step)
        self.control_period = float(control_period) if control_period is not None else None
        self.light_step = bool(light_step)
        self.save_as = bool(save_as)

    def createSimulation(self, scene, **kwargs):
        return DSpaceSimulation(scene, self, **kwargs)

class DSpaceSimulation(RacingSimulation):
    def __init__(self, scene, sim: DSpaceSimulator, **kwargs):
        self.sim = sim
        self._wall_process_start = time.perf_counter()  # for setup-time and total process time in destroy()
        self.exp = None
        self.ts  = None
        self._road_index = None   # parsed from XODR or RD
        self._coordinate_transform = None  # XODR→RD transformation if needed
        self._ego_created = False  # Track if ego vehicle was created
        
        # dSPACE two-phase architecture
        self._modeldesk_app = None  # ModelDesk COM application
        self._fellow_vehicles = {}  # Track fellow vehicles for runtime control
        self._cd = None  # ControlDesk app (session, stepping only)
        self._maport = None  # MAPort app (variable read/write when used)
        self._var_access = None  # get_var/set_var: MAPort if available, else ControlDesk
        self._var_access_backend_logged = False  # one-time runtime log of which backend is used
        self._vehicle_controller = None  # VehicleController (initialized after ControlDesk connection)
        self._fellow_arrays_initialized = False
        self._initializing_fellow_arrays = False
        self._fellow_index_base = 0  # 0 for 0-based arrays, 1 for 1-based arrays
        # External signals (write-back) probing cache
        self._ext_probe_done = False
        self._ext_v_path = None
        self._ext_d_path = None
        self._ext_index_base = 0
        
        ts = kwargs.pop("timestep", None) or sim.timestep
        # Timing and control period: init before super().__init__() because parent runs the whole simulation inside __init__
        self._timing_interval = 50
        self._timing_sums = {"apply_actions": 0.0, "com_writes": 0.0, "step_time": 0.0, "com_reads": 0.0, "loop_other": 0.0, "get_properties": 0.0}
        self._timing_n = 0
        self._timing_last = {}
        self._loop_end = None   # wall time at end of last getProperties (for loop_other)
        self._wall_start = None  # wall time at start of first executeActions (for total wall/sim ratio)
        # Per-step state cache to avoid duplicate ControlDesk readback in same tick
        self._state_cache = {}   # key: (currentTime, id(obj)) -> state dict
        # control_period (seconds) -> steps; must be divisible by timestep
        _control_period = getattr(sim, 'control_period', None)
        if _control_period is None or _control_period <= 0:
            self._control_interval = 1
        else:
            steps_float = _control_period / ts
            if abs(steps_float - round(steps_float)) > 1e-9 or steps_float < 0.99:
                raise ValueError(
                    f"control_period ({_control_period}s) must be a positive multiple of timestep ({ts}s). "
                    f"Got {steps_float} steps (must be an integer)."
                )
            self._control_interval = max(1, int(round(steps_float)))
        self.control_dt = float(ts) * max(1, self._control_interval)
        # Light-step mode: disable COM read/write to test step_time in isolation (param light_step or env SCENIC_DSPACE_LIGHT_STEP)
        _light = os.environ.get("SCENIC_DSPACE_LIGHT_STEP", "").strip().lower()
        self._light_step = getattr(sim, "light_step", False) or _light in ("1", "true", "yes")
        if self._light_step:
            self._light_step_times = []  # for per-step step_time logging

        # Behavior timing for [LoopOther] breakdown (waypoint_speed_grade, after_mpc).
        # MUST be set before super().__init__() because the parent runs the entire simulation inside __init__.
        try:
            from scenic.domains.racing.mpc.timing import BehaviorTiming, set_behavior_timing
            self.behavior_timing = BehaviorTiming()
            set_behavior_timing(self.behavior_timing)
            print("[Timing] behavior_timing active (waypoint_speed_grade, after_mpc in [LoopOther] every 50 steps)")
        except Exception as e:
            self.behavior_timing = None
            print(f"[Timing] behavior_timing disabled: {e}")
        print(f"[DSpaceSimulation] timestep: {ts}, control_interval: {self._control_interval} steps (read/control every {self._control_interval} steps), timing every {self._timing_interval} steps")
        if self._light_step:
            print("[LightStep] *** COM DISABLED *** set_var and get_var are skipped to test step_time only. Vehicle will not move. Set SCENIC_DSPACE_LIGHT_STEP=0 or unset to restore.")
        # Pass only parameters that Simulation.__init__ accepts (avoids TypeError from
        # extra kwargs or int/callable confusion).
        parent_kwargs = {
            "maxSteps": kwargs.get("maxSteps"),
            "name": kwargs.get("name"),
            "timestep": ts,
            "replay": kwargs.get("replay"),
            "enableReplay": kwargs.get("enableReplay", True),
            "allowPickle": kwargs.get("allowPickle", False),
            "enableDivergenceCheck": kwargs.get("enableDivergenceCheck", False),
            "divergenceTolerance": kwargs.get("divergenceTolerance", 0),
            "continueAfterDivergence": kwargs.get("continueAfterDivergence", False),
            "verbosity": kwargs.get("verbosity", 0),
        }
        super().__init__(scene, **parent_kwargs)

    @property
    def is_control_step(self):
        """True when this step is a control step (COM read/write and heavy control run)."""
        return (self.currentTime % self._control_interval) == 0

    # --- TTL (Target Trajectory Line) loading utilities ---
    def _load_ttl_region(self):
        """Load a TTL CSV as a PolylineRegion, applying global transform (dx,dy)."""
        ttl_folder, ttl_index, dx, dy = get_ttl_config(getattr(self.scene, "params", {}) or {})
        try:
            region, pts = load_ttl_region(ttl_folder, ttl_index, dx, dy)
            if region is None:
                return None
            print(f"[TTL] Loaded {len(pts)} points from ttl_{ttl_index}.csv with offset ({dx}, {dy})")
            self._ttl_points_loaded = pts
            return region
        except Exception as e:
            print(f"[TTL] Error loading TTL: {e}")
            return None

    def _get_scx_map_path(self):
        return get_map_path(getattr(self.scene, "params", {}) or {})

    def setup(self):
        """SaveAs/Activate first, then create Fellows, then Save/Download/Reset/Start."""
        pythoncom.CoInitialize()
        app = Dispatch("ModelDesk.Application")
        proj = app.ActiveProject
        if proj is None:
            raise SimulationCreationError("Open a ModelDesk project first.")
        exp = proj.ActiveExperiment
        if exp is None:
            raise SimulationCreationError("Activate an experiment in ModelDesk.")

        # 1) Switch to source, then SaveAs a working copy
        try:
            exp.ActivateTrafficScenario(self.sim.scenario_src)
        except Exception:
            pass

        name = self.sim.scenario_name or time.strftime("Scenic_%Y%m%d_%H%M%S")
        if self.sim.save_as:
            try:
                exp.TrafficScenario.SaveAs(name, True)
            except Exception:
                editor = exp.EditTrafficScenario()
                try:
                    editor.SaveAs(name, True)
                finally:
                    try:
                        editor.Close(False)
                    except Exception:
                        pass
            try:
                exp.ActivateTrafficScenario(name)
            except Exception:
                pass

        # 2) Rebind fresh handles
        pythoncom.PumpWaitingMessages()
        time.sleep(0.2)
        proj = app.ActiveProject
        self.exp = proj.ActiveExperiment
        self.ts  = self.exp.TrafficScenario
        if self.ts is None:
            raise SimulationCreationError("Active experiment has no TrafficScenario.")

        # 3) Clear existing Fellows on the new copy
        try:
            dutils.clear_collection(self.ts.Fellows)
        except Exception:
            pass

        # 4) Build road geometry index and coordinate transformation
        # Strategy: Scenic gives us coordinates in XODR system, but ModelDesk/Aurelion
        # uses RD system. We need to transform between them.
        map_path = self._get_scx_map_path()
        if map_path:
            self._road_index, self._coordinate_transform = build_road_index_and_transform(map_path, dutils)
        else:
            print("[Map] No Scenic `map` param found; will fall back to (0,0).")

        # 5) Let Scenic create objects (calls createObjectInSimulator)
        super().setup()

        # 6) Phase 1: Author scenario in ModelDesk (if dynamic control is needed)
        if self._needsDynamicControl():
            print("[dSPACE] Dynamic control detected - authoring scenario in ModelDesk")
            author_scenario(self)

        # 7) Save and Download All to simulator
        try:
            self.ts.Save()
            self.ts.Download()  # Download all to simulator
        except Exception as e:
            print(f"[ModelDesk] Save/Download failed: {e}")

        # 8) Reset maneuver (but do NOT start yet)
        try:
            mc = self.exp.ManeuverControl
            try: 
                mc.Stop()
            except Exception: 
                pass
            time.sleep(0.2)
            mc.Reset()  # Reset to initial state
            time.sleep(0.2)
        except Exception:
            pass

        # 9) Connect ControlDesk and initialize VesiInterface BEFORE starting maneuver
        self._cd = cd_session.connect_and_prepare(self)
        if self._cd:
            self._var_access = self._cd  # default fallback
            # 9b) Prefer MAPort for variable read/write (faster); fall back to ControlDesk COM
            try:
                from .maport import session as maport_session
                self._maport = maport_session.connect_and_prepare_maport(self, start_if_needed=False)
                if self._maport:
                    self._var_access = self._maport
                    try:
                        from .controldesk.readback import EGO_READ_PATHS
                        from .vehicle.controller import VehicleController as _VC
                        _fellow_trailer_base = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/FELLOW_POS_VEL/FellowTrailer"
                        _fellow_ext_base = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/External_Signals"
                        hot_paths = list(EGO_READ_PATHS) + [
                            _VC.KEY_THROTTLE,
                            _VC.KEY_BRAKE_FRONT,
                            _VC.KEY_BRAKE_REAR,
                            _VC.KEY_STEERING,
                            _VC.KEY_GEAR,
                            _VC.KEY_CLUTCH,
                        ] + [
                            f"{_fellow_trailer_base}/x",
                            f"{_fellow_trailer_base}/y",
                            f"{_fellow_trailer_base}/z",
                            f"{_fellow_trailer_base}/yaw_deg_out",
                            f"{_fellow_trailer_base}/v_Fellows",
                            f"{_fellow_trailer_base}/w_Fellows",
                            f"{_fellow_ext_base}/Const_v_Fellows_External[km|h]/Value",
                            f"{_fellow_ext_base}/Const_d_Fellows_External[m]/Value",
                            SIMULATED_TIME_PATH,
                        ]
                        if hasattr(self._maport, "precache"):
                            self._maport.precache(hot_paths)
                            print(f"[MAPort] Precached {len(hot_paths)} hot refs")
                    except Exception as e:
                        print(f"[MAPort] [WARN] Ref precache skipped: {e}")
                    print("[VarAccess] backend=MAPort (session/step = ControlDesk COM)")
                else:
                    print("[VarAccess] backend=COM (MAPort unavailable/failed)")
            except Exception as e:
                print("[VarAccess] backend=COM (MAPort unavailable/failed): %s" % e)
            # Initialize VehicleController for applying controls (uses _var_access for get_var/set_var)
            self._vehicle_controller = VehicleController(self)
        else:
            self._var_access = None
            self._vehicle_controller = None
            print("\n" + "="*70)
            print("[WARNING] CRITICAL: ControlDesk Connection Failed!")
            print("="*70)
            print("The dSPACE simulator requires ControlDesk to control vehicles.")
            print("\nTo fix this:")
            print("  1. Start ControlDesk application")
            print("  2. Load an experiment with a platform/device")
            print("  3. Ensure the platform is ready for online calibration")
            print("  4. Re-run the Scenic script")
            print("\nWithout ControlDesk, Fellow vehicles CANNOT move.")
            print("="*70 + "\n")

        # 10) NOW start the maneuver via ControlDesk (AFTER VesiInterface is fully initialized)
        if self._cd:
            cd_session.start_maneuver(self._cd)
        
        # 11) Initialize fellow arrays and verify readback before pausing
        print("[Setup] Initializing fellow arrays...")
        ensure_fellow_arrays_initialized(self)
        
        # 12) Verify we can read back values from ControlDesk for all vehicles
        print("[Setup] Verifying ControlDesk readback for all vehicles...")
        readback_success = self._verifyControlDeskReadback()
        
        if readback_success:
            print("[Setup] [OK] ControlDesk readback verified successfully")
        else:
            print("[Setup] [WARN] Warning: Some ControlDesk readbacks failed, but continuing...")
        
        # 13) Pause simulation for step-by-step control (after readback verification)
        print("[Setup] Pausing simulation for step-by-step control...")
        if self._cd and cd_session.pause(self._cd):
            print("[Setup] [OK] Simulation paused successfully")
        else:
            print("[Setup] [WARN] Could not pause simulation")

    def createObjectInSimulator(self, obj):
        """Place car (ego or fellow) by absolute (s,t) computed from (x,y) and XODR.
        
        This function automatically transforms Scenic world coordinates (x,y,z) 
        to the corresponding (s,t) coordinates that will map correctly to Aurelion's
        coordinate system through the dSPACE simulator.
        
        The ego vehicle is handled through the Maneuver API, while other vehicles
        are created as Fellows.
        """
        # Initialize dSPACE actor representation for this vehicle
        self._initializeDSpaceActor(obj)
        
        # Check if this is the ego object
        is_ego = (obj is self.scene.egoObject)
        
        if is_ego:
            return self.createEgoInSimulator(obj)
        else:
            return self.createFellowInSimulator(obj)
    
    def createEgoInSimulator(self, obj):
        """Create/configure the ego vehicle using the Maneuver API.
        
        Unlike Fellows which are added to the Fellows collection, the ego vehicle
        is accessed through TrafficScenario.Maneuver.Item(0) and configured.
        """
        result = place_ego(self, obj)
        # Assign TTL to ego if available (delegated)
        attach_to_ego(self, obj)
        return result
    
    def createFellowInSimulator(self, obj):
        """Create a Fellow vehicle (non-ego) using the Fellows API."""
        result = place_fellow(self, obj)
        # Assign TTL to fellow if available (same as ego)
        attach_ttl(self, obj, vehicle_type="fellow")
        return result
    
    def _needsDynamicControl(self):
        """Check if any Scenic objects need dynamic control (have behaviors with dSPACE actions)."""
        try:
            # Check if any objects have behaviors that use dSPACE actions
            for obj in self.scene.objects:
                if hasattr(obj, 'behavior'):
                    behavior = obj.behavior
                    if behavior:
                        behavior_name = behavior.__class__.__name__
                        # Check if it's a racing behavior that uses dSPACE actions
                        if 'Racing' in behavior_name or 'Pit' in behavior_name:
                            return True
            return False
        except Exception:
            return False
    
    def _connectModelDesk(self):
        """Connect to ModelDesk COM application for scenario authoring."""
        try:
            from win32com.client import Dispatch
            self._modeldesk_app = Dispatch("ModelDesk.Application")
            print("[ModelDesk] Connected to ModelDesk application")
            return True
        except Exception as e:
            print(f"[ModelDesk] Failed to connect: {e}")
            return False

    def _authorScenarioInModelDesk(self):
        """Author scenario in ModelDesk using COM automation.
        
        This implements Phase 1 of the dSPACE architecture:
        - Use existing save_as logic to create working copy
        - Configure fellows with routes and initial properties
        - Set external control flags for fellow vehicles
        - Download scenario to VEOS
        """
        try:
            # Use existing save_as logic from setup()
            self._setupModelDeskScenario()
            
            # Configure fellows based on Scenic objects (skip ego and already-created fellows)
            for scenic_obj in self.scene.objects:
                # Skip EGO: ego is configured via Maneuver API, not Fellows
                if scenic_obj is self.scene.egoObject:
                    continue
                # Skip if this Scenic object already has a Fellow created during placement
                try:
                    exists = any(v.get('scenic_object') is scenic_obj for v in self._fellow_vehicles.values())
                except Exception:
                    exists = False
                if exists:
                    continue
                if hasattr(scenic_obj, 'raceNumber'):  # It's a racing car
                    self._configureFellowInModelDesk(scenic_obj)
            
            # Set external control flags for fellow vehicles (required for ControlDesk control)
            ExternalControlManager.enableExternalControlViaScript(self.scene.objects)
            
            # Check consistency and download
            if self.ts.CheckConsistency():
                print("[ModelDesk] Scenario is consistent")
                if self.ts.Download():
                    print("[ModelDesk] Scenario downloaded to VEOS successfully")
                    return True
                else:
                    print("[ModelDesk] Failed to download scenario to VEOS")
                    return False
            else:
                print("[ModelDesk] Scenario is inconsistent - check configuration")
                return False
                
        except Exception as e:
            print(f"[ModelDesk] Error during scenario authoring: {e}")
            return False
    
    def _setupModelDeskScenario(self):
        """Setup ModelDesk scenario using existing save_as logic."""
        # This integrates with your existing setup() logic
        # The scenario is already saved and activated in setup()
        print(f"[ModelDesk] Using existing scenario: {self.ts.Name}")

    def _configureFellowInModelDesk(self, scenic_obj):
        """Configure a fellow vehicle in ModelDesk scenario.
        
        Args:
            scenic_obj: Scenic RacingCar object
        """
        try:
            # Get fellow name (use raceNumber for identification)
            fellow_name = f"F{scenic_obj.raceNumber}"
            
            # Try to get existing fellow or create new one
            try:
                fellow = self.ts.Fellows.Item(fellow_name)
                print(f"[ModelDesk] Using existing fellow: {fellow_name}")
            except:
                # Create new fellow if it doesn't exist
                fellow = self.ts.Fellows.Add()
                fellow.Name = fellow_name
                print(f"[ModelDesk] Created new fellow: {fellow_name}")
            
            # Get first sequence
            sequences = fellow.Sequences
            if sequences.Count == 0:
                seq = sequences.Add()
            else:
                seq = sequences.Item(1)
            
            # Configure route based on track segment
            route_sel = seq.Route
            
            # Determine track segment robustly
            try:
                pos_xy = (float(scenic_obj.position.x), float(scenic_obj.position.y))
            except Exception:
                pos_xy = None
            track_segment = None
            try:
                if pos_xy is not None:
                    track_segment = self.detectTrackSegment(pos_xy)
            except Exception:
                track_segment = None

            # Default to mainRacing when detection fails
            if track_segment not in ('pitLane', 'mainRacing'):
                track_segment = 'mainRacing'

            # Map to desired route type and pick from available elements
            desired_pref = (self.assignRoute(scenic_obj, track_segment) or 'Lap')
            desired_is_pit = (desired_pref.lower().startswith('pit'))

            chosen_route = None
            available_names = []
            try:
                available = list(route_sel.AvailableElements)
                # Coerce to strings for matching
                available_names = [str(x) for x in available]
                # Prefer names containing 'pit' for pit lane, otherwise avoid them
                if desired_is_pit:
                    pit_candidates = [n for n in available_names if 'pit' in n.lower()]
                    if pit_candidates:
                        chosen_route = pit_candidates[0]
                else:
                    non_pit = [n for n in available_names if 'pit' not in n.lower()]
                    if non_pit:
                        chosen_route = non_pit[0]
                # Fallbacks
                if chosen_route is None and available_names:
                    chosen_route = available_names[0]
            except Exception as e:
                print(f"[ModelDesk] Could not enumerate AvailableElements: {e}")

            # If still nothing, fallback to preference string
            if not chosen_route:
                chosen_route = desired_pref

            # Activate route
            try:
                route_sel.Activate(chosen_route)
                print(f"[ModelDesk] Set route '{chosen_route}' (from pref '{desired_pref}') for {fellow_name}")
            except Exception as e:
                print(f"[ModelDesk] Failed to set route '{chosen_route}' for {fellow_name}: {e}")
                if available_names:
                    print(f"[ModelDesk] Available routes: {available_names}")
            
            # Set direction (forward)
            try:
                route_sel.Direction = 1
                route_sel.UseExternal = True  # Enable external control for fellow vehicles
            except Exception:
                pass
            
            # Store fellow reference for runtime control
            self._fellow_vehicles[fellow_name] = {
                'fellow_object': fellow,
                'scenic_object': scenic_obj,
                'sequence': seq
            }
            
        except Exception as e:
            print(f"[ModelDesk] Error configuring fellow: {e}")
    
    def setVehicleControl(self, vehicle_name, throttle=None, brake=None, steering=None, velocity=None):
        """Set dynamic control inputs for a vehicle using VesiInterface manual control."""
        print(f"\n[setVehicleControl] Called for {vehicle_name}: throttle={throttle}, brake={brake}, steering={steering}, velocity={velocity}")

        # Write directly via variable access (MAPort or ControlDesk) using VesiInterface if available
        if self._var_access:
            print(f"[setVehicleControl] Writing to VesiInterface...")
            # VesiInterface manual control keys (Platform_2)
            KEY_THROTTLE = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_throttle_cmd/Value"
            KEY_BRAKE_FRONT = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_front/Value"
            KEY_BRAKE_REAR = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_rear/Value"
            KEY_STEERING = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_steering_cmd/Value"
            
            try:
                # Throttle: Scenic uses 0-1, ControlDesk expects 0-100 command range
                if throttle is not None:
                    throttle_val = float(max(0.0, min(1.0, throttle)) * 100.0)
                    print(f"  [VesiInterface] Setting throttle: {throttle} -> {throttle_val}")
                    self._var_access.set_var(KEY_THROTTLE, throttle_val)
                    print(f"  [VesiInterface] OK - Throttle written successfully")
                
                # Brake: Scenic uses 0-1, ControlDesk expects 0-10000 command range
                # Apply same value to both front and rear
                if brake is not None:
                    brake_val = float(max(0.0, min(1.0, brake)) * 10000.0)
                    print(f"  [VesiInterface] Setting brake: {brake} -> front={brake_val}, rear={brake_val}")
                    self._var_access.set_var(KEY_BRAKE_FRONT, brake_val)
                    self._var_access.set_var(KEY_BRAKE_REAR, brake_val)
                    print(f"  [VesiInterface] OK - Brake (front/rear) written successfully")
                
                # Steering: plan — steering is road wheel angle (rad). Single conversion here (only place 240 appears).
                if steering is not None:
                    log_startup_once()
                    delta_cmd_rad = float(steering)
                    theta_sw_deg_sent = road_rad_to_dspace_value(delta_cmd_rad)
                    self._var_access.set_var(KEY_STEERING, theta_sw_deg_sent)
                    u_norm = delta_cmd_rad / DELTA_MAX_RAD
                    steer_norm = u_norm
                    sat_io = abs(theta_sw_deg_sent) >= 0.99 * THETA_SW_MAX_DEG
                    cmd_value_sent = theta_sw_deg_sent  # exact float written to Const_steering_cmd/Value (fix.md)
                    # fix.md checklist: cmd_value_sent, theta_sw_deg_sent, steer_norm, delta_cmd_rad, delta_max, R
                    print(f"[STEER_IO] u_norm={u_norm:.4f} delta_cmd_rad={delta_cmd_rad:.4f} steer_norm={steer_norm:.4f} theta_sw_deg_sent={theta_sw_deg_sent:.2f} cmd_value_sent={cmd_value_sent:.2f} delta_max={DELTA_MAX_RAD:.4f} R={R:.2f}")
                    print(f"[IO] theta_sw_deg_sent={theta_sw_deg_sent:.2f} u_norm={u_norm:.4f} delta_cmd_rad={delta_cmd_rad:.4f} R={R:.2f} sat_io={sat_io}")
                    print(f"  [VesiInterface] Setting steering: {delta_cmd_rad:.4f} rad -> {theta_sw_deg_sent:.2f} deg")
                    print(f"  [VesiInterface] OK - Steering written successfully")
                
                # Note: Velocity control not available in VesiInterface manual control
                if velocity is not None:
                    print(f"  [VesiInterface] WARNING - Velocity control not supported in VesiInterface")
                    
            except Exception as e:
                print(f"[VesiInterface] ERROR - Write failed for {vehicle_name}: {e}")
                import traceback
                traceback.print_exc()
                return False
        
        return True
    
    def setVehicleGear(self, vehicle_name, gear):
        """Set gear for a vehicle using VesiInterface manual control (one-shot action).
        
        Gear range: 0 (neutral) to 6.
        """
        print(f"\n[setVehicleGear] Called for {vehicle_name}: gear={gear}")
        
        if self._var_access:
            # Use VesiInterface gear control
            KEY_GEAR = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_gear_cmd/Value"
            try:
                gear_int = int(gear)
                # Clamp gear to valid range: 0 (neutral) to 6
                gear_int = max(0, min(6, gear_int))
                print(f"  [VesiInterface] Setting gear: {gear_int}")
                self._var_access.set_var(KEY_GEAR, gear_int)
                print(f"  [VesiInterface] OK - Gear written successfully")
            except Exception as e:
                print(f"[VesiInterface] ERROR - Gear write failed for {vehicle_name}: {e}")
                import traceback
                traceback.print_exc()
    
    def setVehicleClutch(self, vehicle_name, clutch):
        """Set clutch pedal position for a vehicle (one-shot action).
        
        Note: VesiInterface does not support clutch control, so we use ExternalUserData.
        Clutch is only used for starting from neutral.
        """
        print(f"\n[setVehicleClutch] Called for {vehicle_name}: clutch={clutch}")
        
        if self._var_access:
            # VesiInterface doesn't have clutch, use ExternalUserData
            KEY_CLUTCH = "Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel/ExternalUserData/Pos_ClutchPedal[%]/Value"
            try:
                # Convert 0-1 to 0-100%
                clutch_pct = float(clutch * 100.0)
                print(f"  [VesiInterface] Setting clutch: {clutch} -> {clutch_pct}% (using ExternalUserData)")
                self._var_access.set_var(KEY_CLUTCH, clutch_pct)
                print(f"  [VesiInterface] OK - Clutch written successfully")
            except Exception as e:
                print(f"[VesiInterface] ERROR - Clutch write failed for {vehicle_name}: {e}")
                import traceback
                traceback.print_exc()
    
    def getVehicleState(self, vehicle_name):
        """Get current state of a fellow vehicle.
        
        Args:
            vehicle_name: Name of the fellow vehicle
            
        Returns:
            Dictionary with vehicle state information
        """
        if vehicle_name not in self._fellow_vehicles:
            return None
        
        try:
            vehicle_data = self._fellow_vehicles[vehicle_name]
            fellow_obj = vehicle_data['fellow_object']
            
            state = {
                'name': vehicle_name,
                'position': None,
                'velocity': None,
                'heading': None
            }
            
            # Try to get position and velocity from fellow object
            if hasattr(fellow_obj, 'Position'):
                state['position'] = fellow_obj.Position
            if hasattr(fellow_obj, 'Velocity'):
                state['velocity'] = fellow_obj.Velocity
            if hasattr(fellow_obj, 'Heading'):
                state['heading'] = fellow_obj.Heading
                
            return state
            
        except Exception as e:
            print(f"State retrieval error for {vehicle_name}: {e}")
            return None

    def executeActions(self, allActions):
        """Execute actions selected by agents and apply accumulated control state.

        This method is called by Scenic's simulation framework after behaviors
        determine what actions to take. We:
        1. Ensure fellow arrays are initialized before executing behaviors; if not, skip this tick
        2. Call super() to apply actions via their applyTo() methods (stores in _control_state)
        3. Apply the accumulated control state to ControlDesk variables via VehicleController
           - Ego: VesiInterface (throttle/brake/steering)
           - Fellows: External signals (velocity/deviation computed from physics model)
        4. Clear the control state for next timestep
        """
        if not hasattr(self, '_execute_count'):
            self._execute_count = 0
        self._execute_count += 1

        # --- Loop-other: time between end of previous getProperties and start of this executeActions (core Scenic loop) ---
        _t_loop_start = time.perf_counter()
        if self._wall_start is None:
            self._wall_start = _t_loop_start
        if self._loop_end is not None:
            self._timing_last['loop_other'] = _t_loop_start - self._loop_end
        else:
            self._timing_last['loop_other'] = 0.0

        # --- Timing: flush previous step and init this step (only record when all buckets present) ---
        _timing_keys = ('apply_actions', 'com_writes', 'step_time', 'com_reads', 'loop_other', 'get_properties')
        _last = self._timing_last
        if all(k in _last for k in _timing_keys):
            for k in self._timing_sums:
                self._timing_sums[k] += _last.get(k, 0.0)
            self._timing_n += 1
            n = self._timing_n
            if n % self._timing_interval == 0:
                s = self._timing_sums
                var_backend = "MAPort" if (self._maport is not None and self._var_access is self._maport) else "COM"
                print(
                    f"[Timing] steps={n} mean(s): "
                    f"apply_actions={s['apply_actions']/n:.4f} "
                    f"var_writes({var_backend})={s['com_writes']/n:.4f} "
                    f"step_time={s['step_time']/n:.4f} "
                    f"var_reads({var_backend})={s['com_reads']/n:.4f} "
                    f"loop_other={s['loop_other']/n:.4f} "
                    f"get_properties={s['get_properties']/n:.4f}"
                )
        # Reset only com_reads and get_properties so previous step's apply_actions/com_writes/step_time/loop_other are kept for next flush
        self._timing_last['com_reads'] = 0.0
        self._timing_last['get_properties'] = 0.0

        if not self._fellow_arrays_initialized:
            ensure_fellow_arrays_initialized(self)
            if not self._fellow_arrays_initialized:
                # Skip this tick to give ControlDesk time to produce fellow arrays
                # Behaviors will run again next tick once arrays are ready
                # (avoids losing one-shot actions)
                if self._execute_count % 50 == 1:
                    print(f"[executeActions #{self._execute_count}] Waiting for fellow arrays...")
                return

        if self._execute_count % 50 == 1:
            step_idx = int(getattr(self, "currentTime", 0))
            sim_t = step_idx * float(self.timestep)
            print(f"[executeActions] step={step_idx} sim_t={sim_t:.2f}s #{self._execute_count} Executing actions for {len(self.scene.objects)} objects")

        # First, let actions apply themselves (this calls setThrottle, setSteering, etc.)
        _t0 = time.perf_counter()
        super().executeActions(allActions)
        self._timing_last['apply_actions'] = time.perf_counter() - _t0

        # Apply control only every control_interval steps. In light-step mode skip variable writes.
        if not getattr(self, "_light_step", False) and self._vehicle_controller and (self.currentTime % self._control_interval) == 0:
            _t0 = time.perf_counter()
            for obj in self.scene.objects:
                # Determine if this is ego or fellow
                is_ego = (obj is self.scene.egoObject)

                if is_ego:
                    # Ego: Use VesiInterface physics-based control
                    self._vehicle_controller.apply_ego_control(obj)
                else:
                    # Fellow: Only drive if a behavior exists; otherwise leave stationary
                    if getattr(obj, 'behavior', None):
                        self._vehicle_controller.apply_fellow_control(obj)

                # Clear control state after applying
                if hasattr(obj, '_control_state'):
                    obj._control_state = {}
                if hasattr(obj, '_oneshot_actions'):
                    obj._oneshot_actions = []
            self._timing_last['com_writes'] = time.perf_counter() - _t0
        elif getattr(self, "_light_step", False):
            self._timing_last['com_writes'] = 0.0
            if self._execute_count == 1:
                print("[LightStep] COM writes disabled (set_var skipped) for this run.")
        else:
            if self._execute_count == 1:
                print(f"\n{'='*70}")
                print(f"[WARNING] CRITICAL ERROR: VehicleController is None!")
                print(f"{'='*70}")
                print(f"Control commands are being generated but CANNOT be applied.")
                print(f"This means ControlDesk is not connected to the simulator.")
                print(f"\nBehaviors are running and generating throttle/brake/steering,")
                print(f"but these values are NEVER written to the dSPACE simulator.")
                print(f"\nResult: Vehicles will NOT move!")
                print(f"{'='*70}\n")
            
            # Read and print ControlDesk variable values (commented out - focus on fellow feedback)
            # self._readAndPrintControlDeskValues()

    def _readAndPrintControlDeskValues(self):
        """Read ControlDesk variable values and print them out.
        
        NOTE: Commented out to focus on fellow vehicle feedback instead of ego feedback.
        This method reads VesiInterface control values which are for ego vehicle control.
        """
        # Commented out - focus on fellow feedback instead
        return
        
        # if not self._cd:
        #     return
        # 
        # try:
        #     # VesiInterface manual control variable paths
        #     KEY_THROTTLE = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_throttle_cmd/Value"
        #     KEY_BRAKE_FRONT = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_front/Value"
        #     KEY_BRAKE_REAR = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_rear/Value"
        #     KEY_STEERING = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_steering_cmd/Value"
        #     KEY_GEAR = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_gear_cmd/Value"
        #     KEY_CLUTCH = "Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel/ExternalUserData/Pos_ClutchPedal[%]/Value"
        #     
        #     print(f"\n[ControlDesk Values] Reading variable values:")
        #     
        #     # Read throttle (0-100 command range)
        #     try:
        #         throttle_val = self._cd.get_var(KEY_THROTTLE)
        #         throttle_scenic = throttle_val / 100.0  # Convert back to 0-1 range
        #         print(f"  Throttle: {throttle_val} ({throttle_scenic:.3f} normalized)")
        #     except Exception as e:
        #         print(f"  Throttle: [Error reading: {e}]")
        #     
        #     # Read brake front (0-100 command range)
        #     try:
        #         brake_front_val = self._cd.get_var(KEY_BRAKE_FRONT)
        #         brake_front_scenic = brake_front_val / 100.0  # Convert back to 0-1 range
        #         print(f"  Brake (Front): {brake_front_val} ({brake_front_scenic:.3f} normalized)")
        #     except Exception as e:
        #         print(f"  Brake (Front): [Error reading: {e}]")
        #     
        #     # Read brake rear (0-100 command range)
        #     try:
        #         brake_rear_val = self._cd.get_var(KEY_BRAKE_REAR)
        #         brake_rear_scenic = brake_rear_val / 100.0  # Convert back to 0-1 range
        #         print(f"  Brake (Rear): {brake_rear_val} ({brake_rear_scenic:.3f} normalized)")
        #     except Exception as e:
        #         print(f"  Brake (Rear): [Error reading: {e}]")
        #     
        #     # Read steering (-240 to +240 command range, -240=right to 240=left)
        #     try:
        #         steering_val = self._cd.get_var(KEY_STEERING)
        #         steering_scenic = -steering_val / 240.0  # Convert back to -1 to 1 range
        #         print(f"  Steering: {steering_val} ({steering_scenic:.3f} normalized)")
        #     except Exception as e:
        #         print(f"  Steering: [Error reading: {e}]")
        #     
        #     # Read gear (0-6 integer, 0 = neutral)
        #     try:
        #         gear_val = self._cd.get_var(KEY_GEAR)
        #         print(f"  Gear: {gear_val}")
        #     except Exception as e:
        #         print(f"  Gear: [Error reading: {e}]")
        #     
        #     # Read clutch (0-100 command range, managed automatically with gear)
        #     try:
        #         clutch_val = self._cd.get_var(KEY_CLUTCH)
        #         clutch_scenic = clutch_val / 100.0  # Convert back to 0-1 range
        #         print(f"  Clutch: {clutch_val} ({clutch_scenic:.3f} normalized)")
        #     except Exception as e:
        #         print(f"  Clutch: [Error reading: {e}]")
        #         
        # except Exception as e:
        #     print(f"[ControlDesk Values] Error reading variables: {e}")
        #     import traceback
        #     traceback.print_exc()

    def _initializeDSpaceActor(self, obj):
        """Initialize dSPACE actor representation for a vehicle object.
        
        Creates or updates the dspaceActor attribute to store the vehicle's state
        (position, velocity, etc.) that will be updated each timestep by reading
        from ControlDesk.
        
        Args:
            obj: The Scenic object (vehicle) to initialize
        """
        ensure_actor(obj)
    
    def _readVehicleStateFromControlDesk(self, obj):
        """Read vehicle state from ControlDesk and update dspaceActor.
        
        Reads position, velocity, heading from ControlDesk variables and
        updates the object's dspaceActor state.
        
        Args:
            obj: The Scenic object (vehicle) to read state for
            
        Returns:
            True if state was successfully read, False otherwise
        """
        if not self._var_access:
            return False
        
        try:
            # Ensure dspaceActor exists
            self._initializeDSpaceActor(obj)
            
            # Determine vehicle index based on whether it's ego or fellow
            is_ego = (obj is self.scene.egoObject)
            
            if is_ego:
                # Read ego vehicle state
                return self._readEgoStateFromControlDesk(obj)
            else:
                # Read fellow vehicle state
                return self._readFellowStateFromControlDesk(obj)
                
        except Exception as e:
            print(f"[_readVehicleStateFromControlDesk] Error: {e}")
            return False
    
    def _readEgoStateFromControlDesk(self, obj):
        """Read ego vehicle state from ControlDesk.
        
        Args:
            obj: The ego vehicle object
            
        Returns:
            True if successful, False otherwise
        """
        return read_ego_state(self, obj)
    
    def _verifyControlDeskReadback(self):
        """Verify that we can read back values from ControlDesk for all vehicles.
        
        Attempts to read position/state for ego and all fellow vehicles to ensure
        ControlDesk connection is working properly.
        
        Returns:
            True if all vehicles readback successfully, False otherwise
        """
        if not self._var_access:
            print("[Setup] [WARN] No variable access (ControlDesk/MAPort), skipping readback verification")
            return False
        
        all_success = True
        
        # Verify ego readback
        ego = self.scene.egoObject
        if ego:
            print("[Setup] Verifying ego vehicle readback...")
            try:
                ego_success = self._readVehicleStateFromControlDesk(ego)
                if ego_success:
                    if hasattr(ego, 'dspaceActor') and ego.dspaceActor:
                        pos = ego.dspaceActor.position
                        print(f"[Setup] [OK] Ego readback successful: position=({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
                    else:
                        print(f"[Setup] [OK] Ego readback successful (no position available)")
                else:
                    print(f"[Setup] [ERROR] Ego readback failed")
                    all_success = False
            except Exception as e:
                print(f"[Setup] [ERROR] Ego readback error: {e}")
                all_success = False
        else:
            print("[Setup] [WARN] No ego vehicle found")
        
        # Verify fellow readbacks
        fellows = [obj for obj in self.scene.objects if obj is not ego]
        if fellows:
            print(f"[Setup] Verifying {len(fellows)} fellow vehicle(s) readback...")
            for i, fellow in enumerate(fellows):
                try:
                    fellow_success = self._readVehicleStateFromControlDesk(fellow)
                    if fellow_success:
                        if hasattr(fellow, 'dspaceActor') and fellow.dspaceActor:
                            pos = fellow.dspaceActor.position
                            print(f"[Setup] [OK] Fellow {i} readback successful: position=({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
                        else:
                            print(f"[Setup] [OK] Fellow {i} readback successful (no position available)")
                    else:
                        print(f"[Setup] [ERROR] Fellow {i} readback failed")
                        all_success = False
                except Exception as e:
                    print(f"[Setup] [ERROR] Fellow {i} readback error: {e}")
                    all_success = False
        else:
            print("[Setup] No fellow vehicles to verify")
        
        return all_success
    
    def _ensureFellowArraysInitialized(self):
        """Deprecated wrapper for controldesk.arrays.ensure_fellow_arrays_initialized."""
        ensure_fellow_arrays_initialized(self)

    def _probe_external_index_base(self):
        """Probe ControlDesk External_Signals vector paths and index base.
        Determines whether the Value arrays exist and whether indexing is 0-based or 1-based.
        Caches the result on the simulation instance to avoid repeated COM calls.
        """
        return probe_external_index_base(self)
    
    def _readFellowStateFromControlDesk(self, obj):
        """Read fellow vehicle state from ControlDesk.
        
        Args:
            obj: The fellow vehicle object
            
        Returns:
            True if successful, False otherwise
            
        Note:
            ControlDesk arrays may not be initialized until the simulation has started.
            This method gracefully handles array bounds errors by returning False,
            allowing the simulation to continue with the last known state.
        """
        return read_fellow_state(self, obj, dutils)
    
    def _getFellowIndex(self, obj):
        """Get the fellow index for a vehicle object.
        
        Args:
            obj: The fellow vehicle object
            
        Returns:
            Fellow index (0-based integer) or None if not found
            
        Note:
            ControlDesk arrays typically have limited size (10-20 fellow vehicles).
            We prioritize the stored index from fellow_vehicles dict, which is the
            actual ModelDesk-assigned index based on creation order.
            raceNumber is NOT used as it's randomly assigned and doesn't correspond
            to ControlDesk array indices.
        """
        return indexing_get_fellow_index(self, obj)

    def step(self):
        """Execute one simulation step: RTA step, then block until simulated time has advanced by the step amount; print simulated time after the block."""
        _t0 = time.perf_counter()

        t_before = None
        if self._var_access:
            try:
                t_before = float(self._var_access.get_var(SIMULATED_TIME_PATH))
            except Exception:
                pass

        if self._cd:
            try:
                self._cd.advance_simulation_step()
            except Exception as e:
                time.sleep(self.timestep)
                self._timing_last['step_time'] = time.perf_counter() - _t0
                print(f"[Step] advance_failed: {e}")
                return

        step_logged = False
        if t_before is not None and self._var_access:
            deadline = t_before + self.timestep
            poll_timeout_wall = 10.0 * self.timestep
            max_retries = 10
            for attempt in range(max_retries):
                if attempt > 0 and self._cd:
                    self._cd.advance_simulation_step()
                poll_start = time.perf_counter()
                while time.perf_counter() - poll_start < poll_timeout_wall:
                    try:
                        t_now = float(self._var_access.get_var(SIMULATED_TIME_PATH))
                        if t_now >= deadline:
                            print(f"[Step] simulated_time={t_now:.6f}s")
                            step_logged = True
                            break
                    except Exception:
                        pass
                    time.sleep(0.001)
                if step_logged:
                    break
                print(f"[Step] retry attempt count (attempt {attempt + 1})")
            if not step_logged:
                print("[Step] timeout (simulated_time deadline not reached after retries)")
        else:
            time.sleep(self.timestep)
            if self._var_access:
                try:
                    t_after = float(self._var_access.get_var(SIMULATED_TIME_PATH))
                    print(f"[Step] simulated_time={t_after:.6f}s")
                    step_logged = True
                except Exception:
                    pass
            if not step_logged:
                print("[Step] no_var_access (slept, no time read)")

        self._timing_last['step_time'] = time.perf_counter() - _t0

    def getProperties(self, obj, properties):
        """Read the values of the given properties of the object from the simulator.
        
        Reads vehicle state from ControlDesk only every control_interval steps
        (same frequency as control); other steps use cached dspaceActor state.
        
        Args:
            obj: Scenic object in question
            properties: Set of names of properties to read from the simulator
            
        Returns:
            A dict mapping each of the given properties to its current value
        """
        _t_get_props = time.perf_counter()
        # Initialize dspaceActor if it doesn't exist
        self._initializeDSpaceActor(obj)

        # Read from ControlDesk only on control steps (same cadence as apply). Skip in light-step mode.
        if not getattr(self, "_light_step", False) and (self.currentTime % self._control_interval) == 0:
            _t0 = time.perf_counter()
            self._readVehicleStateFromControlDesk(obj)
            self._timing_last['com_reads'] = self._timing_last.get('com_reads', 0.0) + (time.perf_counter() - _t0)
        elif getattr(self, "_light_step", False):
            if not getattr(self, "_light_step_read_logged", False):
                self._light_step_read_logged = True
                print("[LightStep] COM reads disabled (get_var skipped) for this run.")
            self._timing_last['com_reads'] = self._timing_last.get('com_reads', 0.0) + 0.0

        # Get state from dspaceActor (fresh or cached from last read)
        actor = obj.dspaceActor
        pos = actor.position
        vel = actor.linvel
        ang = actor.angvel
        yaw = actor.heading
        
        # Build property values dictionary
        vals = {
            "position":        pos,
            "velocity":        vel,
            "speed":           vel.norm(),
            "angularVelocity": ang,
            "angularSpeed":    ang.norm(),
            "yaw":             float(yaw),
            "pitch":           0.0,
            "roll":            0.0,
            "elevation":       float(pos.z),
        }
        # Cache state for this step so MPC io_adapter can reuse it (avoid duplicate COM read)
        cache_key = (self.currentTime, id(obj))
        self._state_cache[cache_key] = {
            'x': float(pos.x),
            'y': float(pos.y),
            'yaw': float(yaw),
            'speed': float(vel.norm()),
            'yaw_rate': float(ang.z) if hasattr(ang, 'z') else 0.0,
        }
        # --- Tiny clock debug: capture latest ego snapshot for alignment checks ---
        # Only set after valid ego properties are read; never store None (avoids nan in ClockDebug print)
        scene_ego = getattr(self.scene, "egoObject", None)
        is_ego_obj = (
            (obj is scene_ego)
            or bool(getattr(obj, "isEgoObject", False))
            or bool(getattr(obj, "isEgo", False))
            or ("ego" in str(getattr(obj, "name", "")).lower())
        )
        if is_ego_obj and pos is not None and vel is not None:
            try:
                # Safe numeric extraction: do not store None (consumer uses .get(key, nan) for missing keys)
                _x = float(pos.x) if hasattr(pos, "x") and pos.x is not None else 0.0
                _y = float(pos.y) if hasattr(pos, "y") and pos.y is not None else 0.0
                _z = float(getattr(pos, "z", 0.0)) if pos is not None else 0.0
                _speed = float(vel.norm()) if hasattr(vel, "norm") else 0.0
                _yaw = float(yaw) if yaw is not None else 0.0
                if not math.isfinite(_x):
                    _x = 0.0
                if not math.isfinite(_y):
                    _y = 0.0
                if not math.isfinite(_z):
                    _z = 0.0
                if not math.isfinite(_speed):
                    _speed = 0.0
                if not math.isfinite(_yaw):
                    _yaw = 0.0
            except Exception:
                pass
        self._timing_last['get_properties'] = self._timing_last.get('get_properties', 0.0) + (time.perf_counter() - _t_get_props)
        # Mark end of this step's work for loop_other timing (gap until next executeActions)
        self._loop_end = time.perf_counter()
        # Best-effort cleanup of old entries (keep only current step)
        if hasattr(self, "_state_cache"):
            stale_keys = [k for k in self._state_cache.keys() if k[0] != self.currentTime]
            for k in stale_keys:
                self._state_cache.pop(k, None)
        return {k: vals[k] for k in properties if k in vals}

    def destroy(self):
        """Print final timing summary and COM per-path timing."""
        # Setup and total process time (from DSpaceSimulation __init__ to destroy)
        _now = time.perf_counter()
        _process_start = getattr(self, '_wall_process_start', None)
        if _process_start is not None:
            total_process = _now - _process_start
            if getattr(self, '_wall_start', None) is not None:
                setup_wall = self._wall_start - _process_start
                print(f"[Timing] setup (init to first executeActions)={setup_wall:.2f}s  total process={total_process:.2f}s")
        # Note: setup (before first step) and first steps (warm-up) are not included in per-step stats
        if self._timing_n > 0:
            n = self._timing_n
            s = self._timing_sums
            var_backend = "MAPort" if (self._maport is not None and self._var_access is self._maport) else "COM"
            print(
                f"[Timing] FINAL (steps={n}): mean(s): "
                f"apply_actions={s['apply_actions']/n:.4f} "
                f"var_writes({var_backend})={s['com_writes']/n:.4f} "
                f"step_time={s['step_time']/n:.4f} "
                f"var_reads({var_backend})={s['com_reads']/n:.4f} "
                f"loop_other={s['loop_other']/n:.4f} "
                f"get_properties={s['get_properties']/n:.4f}"
            )
            print(f"[Timing] Variable access backend: {var_backend}  (var_writes/var_reads = {var_backend} set_var/get_var time)")
            total = (s['apply_actions'] + s['com_writes'] + s['step_time'] + s['com_reads'] + s['loop_other'] + s['get_properties']) / n
            print(f"[Timing] mean total per step (all buckets)={total:.4f}s  (get_properties = getProperties wall time incl. behavior; loop_other = gap getProperties->executeActions)")
            sim_time_total = n * self.timestep
            step_wall_total = s['step_time']
            ratio = step_wall_total / sim_time_total if sim_time_total > 0 else 0
            print(f"[Timing] wall/sim ratio (step_time only): {ratio:.2f}  (>1 = slower than real time)")
            # Total wall from first step to now (excludes setup before first executeActions)
            if getattr(self, '_wall_start', None) is not None:
                total_wall = _now - self._wall_start
                ratio_total = total_wall / sim_time_total if sim_time_total > 0 else 0
                print(f"[Timing] total wall (first step to destroy)={total_wall:.2f}s for {sim_time_total:.2f}s sim -> wall/sim={ratio_total:.2f}  (excludes setup before first step)")
        if getattr(self, "_light_step", False) and getattr(self, "_light_step_times", None):
            lt = self._light_step_times
            if lt:
                n_lt = len(lt)
                mean_ms = sum(lt) / n_lt * 1000
                sim_total = n_lt * self.timestep
                wall_total = sum(lt)
                ratio_lt = wall_total / sim_total if sim_total > 0 else 0
                print(f"[LightStep] FINAL step_time: n={n_lt} mean={mean_ms:.2f} ms min={min(lt)*1000:.2f} ms max={max(lt)*1000:.2f} ms (COM was disabled)")
                print(f"[LightStep] Total step wall={wall_total:.2f}s for {sim_total:.2f}s sim -> wall/sim={ratio_lt:.2f}  (setup time before first step is not included)")
                # Steady-state: exclude first 20 steps (warm-up)
                warmup = 20
                if n_lt > warmup:
                    lt_ss = lt[warmup:]
                    mean_ss = sum(lt_ss) / len(lt_ss) * 1000
                    print(f"[LightStep] Steady-state (excluding first {warmup} steps): mean={mean_ss:.2f} ms min={min(lt_ss)*1000:.2f} ms max={max(lt_ss)*1000:.2f} ms")
        if self._var_access and hasattr(self._var_access, 'print_timing_summary'):
            if self._maport is not None and self._var_access is self._maport:
                print("[Timing] MAPort per-path timing (get_var/set_var) below:")
            self._var_access.print_timing_summary()
        # Optional: when MAPort was var backend, also print COM timing (step/session) if available
        if self._cd is not None and self._cd is not self._var_access and hasattr(self._cd, 'print_timing_summary'):
            self._cd.print_timing_summary()
        if self._maport is not None:
            print("[VarAccess] Teardown: disposing MAPort (variable backend was MAPort).")
            try:
                self._maport.dispose()
            except Exception:
                pass
            self._maport = None
        super().destroy()

    def getRacingControllers(self, agent, use_mpc=False, mpc_config_path=None):
        """Get racing controllers optimized for dSPACE racing scenarios.
        dSPACE-specific racing controllers tuned for ModelDesk's physics
        and control systems.
        
        Args:
            agent: The racing agent (RacingCar, etc.)
            use_mpc: If True, use MPC for both lateral and longitudinal control
            mpc_config_path: Path to MPC config YAML file (optional, uses default if None)
            
        Returns:
            A pair of controllers for throttle and steering respectively.
            If use_mpc=True: (MPCLongitudinalController, MPCLateralController)
            If use_mpc=False: (PIDLongitudinalController, PIDLateralController)
        """
        # Controller update period should match control cadence, not physics sim step
        control_dt = self.timestep * max(1, getattr(self, "_control_interval", 1))
        dt = control_dt
        
        # Controllers: MPC or PID
        if use_mpc:
            try:
                from scenic.domains.racing.mpc import (
                    MPCLongitudinalController,
                    MPCLateralController,
                    load_mpc_config
                )
                
                # Load MPC configuration
                config = load_mpc_config(mpc_config_path)
                
                # Adapt config to simulation timestep
                config.adapt_to_timestep(dt)
                
                # Create both MPC controllers
                lon_controller = MPCLongitudinalController(config, timestep=dt)
                lat_controller = MPCLateralController(config, timestep=dt)
                
                # Steering contract: MPC path uses road wheel angle in radians (see RACING_CONTROL_CONTRACT.md)
                agent._racing_steer_units = 'rad'
                
                # Store config in simulation for io_adapter access
                self.mpc_config = config
                
                print(f"[DSpaceSimulation] Using MPC controllers (longitudinal + lateral)")
                print(f"[DSpaceSimulation] Horizon={config.mpc_prediction_horizon}, dt={config.mpc_prediction_dt} (control_dt={dt})")
                return lon_controller, lat_controller
            except Exception as e:
                print(f"[DSpaceSimulation] WARNING: Failed to create MPC controllers: {e}")
                print(f"[DSpaceSimulation] Falling back to PID controllers")
                agent._racing_steer_units = 'normalized'
                # Fall back to PID
                from scenic.domains.driving.controllers import (
                    PIDLongitudinalController,
                    PIDLateralController
                )
                lon_controller = PIDLongitudinalController(K_P=0.8, K_D=0.15, K_I=0.9, dt=dt)
                lat_controller = PIDLateralController(K_P=0.3, K_D=0.15, K_I=0.0, dt=dt)
                return lon_controller, lat_controller
        else:
            # Standard PID controllers (steering in normalized [-1, 1])
            agent._racing_steer_units = 'normalized'
            from scenic.domains.driving.controllers import (
                PIDLongitudinalController,
                PIDLateralController
            )
            lon_controller = PIDLongitudinalController(K_P=0.8, K_D=0.15, K_I=0.9, dt=dt)
            lat_controller = PIDLateralController(K_P=0.3, K_D=0.15, K_I=0.0, dt=dt)
            return lon_controller, lat_controller
    
    def getRacingLineControllers(self, agent):
        """Get controllers optimized for following the racing line in dSPACE.
        Args:
            agent: The racing agent
            
        Returns:
            A pair of controllers for throttle and steering respectively.
        """
        dt = self.timestep
        
        # dSPACE racing line controllers - more aggressive for optimal lap times
        from scenic.domains.driving.controllers import PIDLongitudinalController, PIDLateralController
        lon_controller = PIDLongitudinalController(K_P=0.9, K_D=0.2, K_I=1.0, dt=dt)
        lat_controller = PIDLateralController(K_P=0.4, K_D=0.2, K_I=0.0, dt=dt)
        
        return lon_controller, lat_controller
    
    def getPitLaneControllers(self, agent):
        """Get controllers optimized for pit lane driving in dSPACE.
        Args:
            agent: The racing agent
            
        Returns:
            A pair of controllers for throttle and steering respectively.
        """
        dt = self.timestep
        
        # dSPACE pit lane controllers - precision over speed
        from scenic.domains.driving.controllers import PIDLongitudinalController, PIDLateralController
        lon_controller = PIDLongitudinalController(K_P=0.4, K_D=0.08, K_I=0.6, dt=dt)
        lat_controller = PIDLateralController(K_P=0.15, K_D=0.08, K_I=0.0, dt=dt)
        
        return lon_controller, lat_controller
    
    def detectTrackSegment(self, position):
        """Detect which track segment a position belongs to in dSPACE.
        
        Uses projection onto the road index and compares the projected road ID
        against IDs provided by the racing domain params (pitLaneRoadIds and
        mainRacingRoadIds).
        """
        try:
            params = getattr(self.scene, "params", {}) or {}
            seg = detect_track_segment(position, self._road_index, params, dutils)
            return seg
        except Exception as e:
            print(f"    [TrackSegment] Detection error: {e}")
            return None
    
    def assignRoute(self, agent, track_segment):
        """Assign appropriate dSPACE route based on track segment.
        
        Args:
            agent: The racing agent
            track_segment: Track segment identifier ('mainRacing' or 'pitLane')
            
        Returns:
            String indicating the route preference for dSPACE
        """
        return assign_route_for_segment(track_segment)
    
    def _detect_route_from_road_segment(self, obj):
        """Auto-detect route preference using racing domain abstract methods.
        
        Uses the abstract detectTrackSegment() and assignRoute() methods
        to determine the appropriate route for the object.
        
        Args:
            obj: The Scenic object
            
        Returns:
            String route preference ('Pit' or 'Lap') or None
        """
        try:
            # Get object position
            obj_pos = obj.position
            position = (float(obj_pos.x), float(obj_pos.y))
            
            # Use abstract method to detect track segment
            track_segment = self.detectTrackSegment(position)
            if track_segment is None:
                return None
            
            # Use abstract method to assign route
            route_preference = self.assignRoute(obj, track_segment)
            return route_preference
                
        except Exception as e:
            print(f"    [Route] Auto-detection error: {e}")
            return None
    
    def _set_fellow_route_via_sequence(self, sequence, obj):
        """Set the ModelDesk route for a fellow vehicle via FellowSequence.Route.
        
        Automatically detects appropriate route based on road segment placement.
        Falls back to explicit dspace_route attribute if auto-detection fails.
        
        Args:
            sequence: The fellow's sequence object (FellowSequence)
            obj: The Scenic object (vehicle)
        """
        # Delegate to routes module
        routes_set_route(sequence, obj, self.detectTrackSegment, self.assignRoute)