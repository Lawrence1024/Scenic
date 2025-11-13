# -*- coding: utf-8 -*-
# Scenic → dSPACE (ModelDesk) absolute placement:
# - SaveAs/Activate first (desired)
# - Build XODR reference index from Scenic param `map`
# - For each Scenic object: (x,y) → (s,t), then seg0 uses absolute Position/Deviation

import time
import pythoncom
from win32com.client import Dispatch

from scenic.domains.racing.simulators import RacingSimulator, RacingSimulation
from scenic.core.simulators import SimulationCreationError

from .utils import legacy as dutils
from .controldesk.per_tick_control import ExternalControlManager
from .vehicle import VehiclePhysicsState, VehicleController
from .ttl.loader import get_ttl_config, load_ttl_region, attach_to_ego
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




class DSpaceSimulator(RacingSimulator):
    def __init__(self, *, scenario_src="LagunaSeca_ExternalControl",
                 scenario_name=None, timestep=1, save_as=True):
        super().__init__()
        self.scenario_src = scenario_src
        self.scenario_name = scenario_name
        self.timestep = float(timestep)
        self.save_as = bool(save_as)

    def createSimulation(self, scene, **kwargs):
        return DSpaceSimulation(scene, self, **kwargs)

class DSpaceSimulation(RacingSimulation):
    def __init__(self, scene, sim: DSpaceSimulator, **kwargs):
        self.sim = sim
        self.exp = None
        self.ts  = None
        self._road_index = None   # parsed from XODR or RD
        self._coordinate_transform = None  # XODR→RD transformation if needed
        self._ego_created = False  # Track if ego vehicle was created
        
        # dSPACE two-phase architecture
        self._modeldesk_app = None  # ModelDesk COM application
        self._fellow_vehicles = {}  # Track fellow vehicles for runtime control
        self._cd = None  # ControlDesk app
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
        print(f"[DSpaceSimulation] timestep: {ts}")
        super().__init__(scene, timestep=ts, **kwargs)

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
        print("[ModelDesk] Saving and downloading scenario...")
        try:
            self.ts.Save()
            self.ts.Download()  # Download all to simulator
            print("[ModelDesk] Download complete")
        except Exception as e:
            print(f"[ModelDesk] Save/Download failed: {e}")

        # 8) Reset maneuver (but do NOT start yet)
        print("[ModelDesk] Resetting maneuver...")
        try:
            mc = self.exp.ManeuverControl
            try: 
                mc.Stop()
            except Exception: 
                pass
            time.sleep(0.2)
            mc.Reset()  # Reset to initial state
            time.sleep(0.2)
            print("[ModelDesk] Reset complete - maneuver ready (not started)")
        except Exception as e:
            print(f"[ModelDesk] Reset failed: {e}")

        # 9) Connect ControlDesk and initialize VesiInterface BEFORE starting maneuver
        print("[ControlDesk] Connecting and initializing VesiInterface...")
        self._cd = cd_session.connect_and_prepare(self)
        if self._cd:
            # Initialize VehicleController for applying controls
            self._vehicle_controller = VehicleController(self)
            print("[VehicleController] Initialized")
        else:
            self._vehicle_controller = None

        # 10) NOW start the maneuver via ControlDesk (AFTER VesiInterface is fully initialized)
        if self._cd and cd_session.start_maneuver(self._cd):
            print("[Maneuver] ✅ Started via ControlDesk - VesiInterface controls active")
        else:
            print("[Maneuver] ⚠️  Skipping start - ControlDesk not available")
        
        # Pause simulation initially for step-by-step control
        if self._cd and cd_session.pause(self._cd):
            # Immediately try to warm-up fellow arrays so first read/write won't warn
            self._ensureFellowArraysInitialized()

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
            print(f"Creating EGO vehicle in dSPACE at position: {obj.position}, heading: {obj.heading}")
            return self.createEgoInSimulator(obj)
        else:
            print(f"Creating FELLOW vehicle in dSPACE at position: {obj.position}, heading: {obj.heading}")
            return self.createFellowInSimulator(obj)
    
    def createEgoInSimulator(self, obj):
        """Create/configure the ego vehicle using the Maneuver API.
        
        Unlike Fellows which are added to the Fellows collection, the ego vehicle
        is accessed through TrafficScenario.Maneuver.Item(0) and configured.
        """
        print(f"  Configuring ego vehicle (Maneuver)")
        result = place_ego(self, obj)
        # Assign TTL to ego if available (delegated)
        attach_to_ego(self, obj)
        return result
    
    def createFellowInSimulator(self, obj):
        """Create a Fellow vehicle (non-ego) using the Fellows API."""
        return place_fellow(self, obj)
    
    def _needsDynamicControl(self):
        """Check if any Scenic objects need dynamic control (have behaviors with dSPACE actions)."""
        try:
            print(f"[dSPACE] Checking {len(self.scene.objects)} objects for dynamic control needs")
            # Check if any objects have behaviors that use dSPACE actions
            for obj in self.scene.objects:
                if hasattr(obj, 'behavior'):
                    behavior = obj.behavior
                    if behavior:
                        behavior_name = behavior.__class__.__name__
                        print(f"[dSPACE] Found behavior: {behavior_name}")
                        # Check if it's a racing behavior that uses dSPACE actions
                        if 'Racing' in behavior_name or 'Pit' in behavior_name:
                            print(f"[dSPACE] Found racing behavior: {behavior_name}")
                            return True
            print("[dSPACE] No racing behaviors found")
            return False
        except Exception as e:
            print(f"[dSPACE] Error checking dynamic control needs: {e}")
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

        # Write directly via ControlDesk using VesiInterface if available
        if self._cd:
            print(f"[setVehicleControl] Writing to ControlDesk (VesiInterface)...")
            # VesiInterface manual control keys (Platform_2)
            KEY_THROTTLE = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_throttle_cmd/Value"
            KEY_BRAKE_FRONT = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_front/Value"
            KEY_BRAKE_REAR = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_rear/Value"
            KEY_STEERING = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_steering_cmd/Value"
            
            try:
                # Throttle: Scenic uses 0-1, ControlDesk expects 0-100 command range
                if throttle is not None:
                    throttle_val = float(max(0.0, min(1.0, throttle)) * 100.0)
                    print(f"  [ControlDesk] Setting throttle: {throttle} -> {throttle_val}")
                    self._cd.set_var(KEY_THROTTLE, throttle_val)
                    print(f"  [ControlDesk] OK - Throttle written successfully")
                
                # Brake: Scenic uses 0-1, ControlDesk expects 0-100 command range
                # Apply same value to both front and rear
                if brake is not None:
                    brake_val = float(max(0.0, min(1.0, brake)) * 100.0)
                    print(f"  [ControlDesk] Setting brake: {brake} -> front={brake_val}, rear={brake_val}")
                    self._cd.set_var(KEY_BRAKE_FRONT, brake_val)
                    self._cd.set_var(KEY_BRAKE_REAR, brake_val)
                    print(f"  [ControlDesk] OK - Brake (front/rear) written successfully")
                
                # Steering: Scenic uses -1 to 1, ControlDesk expects -70 to +70 (right to left)
                # Map -1..1 to -70..+70 command range
                if steering is not None:
                    steer_val = -float(max(-1.0, min(1.0, steering)) * 70.0)
                    print(f"  [ControlDesk] Setting steering: {steering} -> {steer_val}")
                    self._cd.set_var(KEY_STEERING, steer_val)
                    print(f"  [ControlDesk] OK - Steering written successfully")
                
                # Note: Velocity control not available in VesiInterface manual control
                if velocity is not None:
                    print(f"  [ControlDesk] WARNING - Velocity control not supported in VesiInterface")
                    
            except Exception as e:
                print(f"[ControlDesk] ERROR - Write failed for {vehicle_name}: {e}")
                import traceback
                traceback.print_exc()
                return False
        
        return True
    
    def setVehicleGear(self, vehicle_name, gear):
        """Set gear for a vehicle using VesiInterface manual control (one-shot action).
        
        Gear range: 0 (neutral) to 6.
        """
        print(f"\n[setVehicleGear] Called for {vehicle_name}: gear={gear}")
        
        if self._cd:
            # Use VesiInterface gear control
            KEY_GEAR = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_gear_cmd/Value"
            try:
                gear_int = int(gear)
                # Clamp gear to valid range: 0 (neutral) to 6
                gear_int = max(0, min(6, gear_int))
                print(f"  [ControlDesk] Setting gear: {gear_int}")
                self._cd.set_var(KEY_GEAR, gear_int)
                print(f"  [ControlDesk] OK - Gear written successfully")
            except Exception as e:
                print(f"[ControlDesk] ERROR - Gear write failed for {vehicle_name}: {e}")
                import traceback
                traceback.print_exc()
    
    def setVehicleClutch(self, vehicle_name, clutch):
        """Set clutch pedal position for a vehicle using ControlDesk (one-shot action).
        
        Note: VesiInterface does not support clutch control, so we use ExternalUserData
        for clutch operations. This is acceptable as clutch is only used for starting
        from neutral, which is a separate operation from the main control loop.
        """
        print(f"\n[setVehicleClutch] Called for {vehicle_name}: clutch={clutch}")
        
        if self._cd:
            # VesiInterface doesn't have clutch, use ExternalUserData
            KEY_CLUTCH = "Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel/ExternalUserData/Pos_ClutchPedal[%]/Value"
            try:
                # Convert 0-1 to 0-100%
                clutch_pct = float(clutch * 100.0)
                print(f"  [ControlDesk] Setting clutch: {clutch} -> {clutch_pct}% (using ExternalUserData)")
                self._cd.set_var(KEY_CLUTCH, clutch_pct)
                print(f"  [ControlDesk] OK - Clutch written successfully")
            except Exception as e:
                print(f"[ControlDesk] ERROR - Clutch write failed for {vehicle_name}: {e}")
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
        if not self._fellow_arrays_initialized:
            ensure_fellow_arrays_initialized(self)
            if not self._fellow_arrays_initialized:
                # Skip this tick to give ControlDesk time to produce fellow arrays
                # Behaviors will run again next tick once arrays are ready
                # (avoids losing one-shot actions)
                return
        
        # First, let actions apply themselves (this calls setThrottle, setSteering, etc.)
        super().executeActions(allActions)
        
        # Now apply accumulated control state to ControlDesk using VehicleController
        if self._vehicle_controller:
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
        #     # Read steering (-70 to +70 command range, -70=right to 70=left)
        #     try:
        #         steering_val = self._cd.get_var(KEY_STEERING)
        #         steering_scenic = -steering_val / 70.0  # Convert back to -1 to 1 range
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
        if not self._cd:
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
        """Execute one simulation step (advance physics simulation).
        
        This advances the dSPACE simulation by one timestep using ControlDesk.
        Control variables should already be written by executeActions() before this is called.
        """
        cd_session.step(self._cd, self.timestep)

    def getProperties(self, obj, properties):
        """Read the values of the given properties of the object from the simulator.
        
        This method reads vehicle state from ControlDesk and updates the
        dspaceActor representation, then returns the requested properties.
        
        Args:
            obj: Scenic object in question
            properties: Set of names of properties to read from the simulator
            
        Returns:
            A dict mapping each of the given properties to its current value
        """
        # Initialize dspaceActor if it doesn't exist
        self._initializeDSpaceActor(obj)
        
        # Try to read fresh state from ControlDesk
        self._readVehicleStateFromControlDesk(obj)
        
        # Get state from dspaceActor
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
        
        # Return only requested properties
        return {k: vals[k] for k in properties if k in vals}

    def getRacingControllers(self, agent):
        """Get racing controllers optimized for dSPACE racing scenarios.
        dSPACE-specific racing controllers tuned for ModelDesk's physics
        and control systems.
        
        Args:
            agent: The racing agent (RacingCar, etc.)
            
        Returns:
            A pair of controllers for throttle and steering respectively.
        """
        dt = self.timestep
        
        # dSPACE-specific racing controller tuning
        # More aggressive than standard driving controllers
        from scenic.domains.driving.controllers import PIDLongitudinalController, PIDLateralController
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