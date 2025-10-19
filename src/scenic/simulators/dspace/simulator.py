# -*- coding: utf-8 -*-
# Scenic → dSPACE (ModelDesk) absolute placement:
# - SaveAs/Activate first (desired)
# - Build XODR reference index from Scenic param `map`
# - For each Scenic object: (x,y) → (s,t), then seg0 uses absolute Position/Deviation

import time
import pythoncom
from win32com.client import Dispatch

from scenic.core.vectors import Vector
from scenic.domains.racing.simulators import RacingSimulator, RacingSimulation
from scenic.core.simulators import SimulationCreationError

from . import utils as dutils
from .per_tick_control import PerTickController, ExternalControlManager


class DSpaceSimulator(RacingSimulator):
    def __init__(self, *, scenario_src="LagunaSeca_ExternalControl",
                 scenario_name=None, timestep=0.1, save_as=True):
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
        self._per_tick_controller = PerTickController()  # Per-tick control controller
        self._fellow_vehicles = {}  # Track fellow vehicles for runtime control
        
        ts = kwargs.pop("timestep", None) or sim.timestep
        super().__init__(scene, timestep=ts, **kwargs)
    

    def _get_scx_map_path(self):
        # Scenic param set in your script:
        #   param map = localPath('../../assets/maps/dSPACE/LS_converted.xodr')
        try:
            prm = getattr(self.scene, "params", {}) or {}
            for key in ("map", "opendrive", "xodr"):
                if key in prm and prm[key]:
                    return str(prm[key])
        except Exception:
            pass
        return None

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
            import os
            rd_path = map_path.replace('.xodr', '.rd').replace('LagunaSeca', 'Laguna_Seca')
            
            if os.path.exists(rd_path):
                # BEST CASE: We have both XODR and RD files
                # Build automatic transformation from XODR coords → RD coords
                try:
                    from . import coordinate_transform
                    
                    # Check if cached transform exists
                    cache_path = rd_path.replace('.rd', '_transform.json')
                    if os.path.exists(cache_path):
                        print(f"[Transform] Loading cached coordinate transformation")
                        self._coordinate_transform = coordinate_transform.load_transform(cache_path)
                    else:
                        print(f"[Transform] Building automatic XODR→RD coordinate transformation...")
                        self._coordinate_transform = coordinate_transform.build_coordinate_transform(
                            map_path, rd_path, num_samples=100
                        )
                        # Cache for future use
                        coordinate_transform.save_transform(self._coordinate_transform, cache_path)
                    
                    # Use RD geometry for projection (after transformation)
                    from . import rd_geometry
                    self._road_index = rd_geometry.build_rd_road_index(rd_path, step=0.5)
                    print(f"[Geometry] Using RD geometry for accurate (s,t) projection")
                    print(f"[Status] ✅ Full coordinate transformation pipeline active")
                    
                except Exception as e:
                    print(f"[Transform] Failed to build transformation: {e}")
                    print(f"[Transform] Falling back to XODR-only mode (may have positioning errors)")
                    try:
                        self._road_index = dutils.build_xodr_sec_points(map_path)
                        self._coordinate_transform = None
                        print(f"[Geometry] Using XODR geometry")
                    except Exception as e2:
                        print(f"[Error] Failed to parse {map_path}: {e2}")
                        self._road_index = None
                        self._coordinate_transform = None
            else:
                # FALLBACK: Only XODR available
                try:
                    self._road_index = dutils.build_xodr_sec_points(map_path)
                    self._coordinate_transform = None
                    print(f"[Geometry] Using XODR geometry")
                    print(f"[Warning] ⚠️  No RD file found - coordinate mismatches possible (up to 34m)")
                    print(f"[Hint] Place '{os.path.basename(rd_path)}' next to XODR for accurate positioning")
                except Exception as e:
                    print(f"[Error] Failed to parse {map_path}: {e}")
                    self._road_index = None
                    self._coordinate_transform = None
        else:
            print("[Map] No Scenic `map` param found; will fall back to (0,0).")

        # 5) Let Scenic create objects (calls createObjectInSimulator)
        super().setup()

        # 6) Phase 1: Author scenario in ModelDesk (if dynamic control is needed)
        if self._needsDynamicControl():
            print("[dSPACE] Dynamic control detected - authoring scenario in ModelDesk")
            self._authorScenarioInModelDesk()

        # 7) Persist and (optionally) run
        try:
            self.ts.Save()
            self.ts.Download()

            mc = self.exp.ManeuverControl
            try: mc.Stop()
            except Exception: pass
            time.sleep(0.2)
            mc.Reset()
            time.sleep(0.2)
            mc.Start(False)
        except Exception:
            pass

    def createObjectInSimulator(self, obj):
        """Place car (ego or fellow) by absolute (s,t) computed from (x,y) and XODR.
        
        This function automatically transforms Scenic world coordinates (x,y,z) 
        to the corresponding (s,t) coordinates that will map correctly to Aurelion's
        coordinate system through the dSPACE simulator.
        
        The ego vehicle is handled through the Maneuver API, while other vehicles
        are created as Fellows.
        """
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
        
        # 1) Project Scenic (x,y) → (s,t)
        if getattr(obj, "position", None) is not None:
            scenic_x, scenic_y = obj.position.x, obj.position.y
            
            # Apply coordinate transformation if available
            if self._coordinate_transform is not None:
                from . import coordinate_transform
                transformed_x, transformed_y = coordinate_transform.apply_coordinate_transform(
                    self._coordinate_transform, (scenic_x, scenic_y)
                )
                print(f"  Scenic coords ({scenic_x:.3f}, {scenic_y:.3f}) -> "
                      f"RD coords ({transformed_x:.3f}, {transformed_y:.3f})")
                work_x, work_y = transformed_x, transformed_y
            else:
                work_x, work_y = scenic_x, scenic_y
            
            # Use road index for proper geometric projection
            if self._road_index:
                s_val, t_val = dutils.project_world_to_st(self._road_index, (work_x, work_y))
                print(f"  World coordinates ({work_x:.3f}, {work_y:.3f}) -> Road coordinates (s={s_val:.1f}, t={t_val:.3f})")
            else:
                # Use direct projection to road network
                s_val, t_val = dutils.project_world_to_st(self._road_index, (work_x, work_y))
                print(f"  World coordinates ({work_x:.3f}, {work_y:.3f}) -> Fallback coordinates (s={s_val:.1f}, t={t_val:.3f})")
        else:
            s_val, t_val = 0.0, 0.0
            print("  Warning: No position available, using default coordinates (s=0, t=0)")
        
        # 2) Get velocity - always set to 0 for static scenarios
        base_v = 0.0  # Force velocity to 0 for all vehicles
        
        # 3) Access the ego maneuver (Maneuver is a collection, use Item(0))
        try:
            maneuver_collection = self.ts.Maneuver
            if maneuver_collection.Count == 0:
                print("  Warning: No ego maneuver found in scenario - cannot configure ego")
                return None
            
            ego_maneuver = maneuver_collection.Item(0)
            print(f"  Accessed ego maneuver: {ego_maneuver.Name if hasattr(ego_maneuver, 'Name') else 'Ego'}")
            
            # Access sequences
            sequences = ego_maneuver.Sequences
            if sequences.Count == 0:
                print("  Warning: No sequences in ego maneuver - cannot configure")
                return None
            
            seq = sequences.Item(0)
            
            # 4) Configure ego vehicle position and properties
            print(f"  Setting ego position: s={s_val:.1f}, t={t_val:.3f}, velocity={base_v:.1f}")
            
            # Set starting position (s-coordinate)
            seq.StartPosition = float(s_val)
            
            # Set initial velocity (0 for static positioning)
            seq.InitialLongitudinalVelocity = float(base_v)
            
            # Set orientation
            # VehicleOrientation in ModelDesk appears to be relative to road direction:
            #   0.0 = aligned with road
            #   positive = counter-clockwise rotation from road direction
            #   negative = clockwise rotation from road direction
            # For now, default to 0.0 to align with road
            # TODO: Compute relative angle if obj has specific heading requirements
            seq.VehicleOrientation = 0.0
            print(f"  Set orientation: 0.0 degrees (aligned with road)")
            
            # Optionally set lateral position through segments if t != 0
            if abs(t_val) > 0.1:
                try:
                    segments = seq.Segments
                    if segments.Count > 0:
                        seg0 = segments.Item(0)
                        
                        # Configure lateral position (similar to Fellows)
                        lat0 = seg0.Activity.LateralType
                        dutils.activate_type(lat0, "Deviation")
                        
                        # Set dependency to Absolute
                        dep = getattr(lat0.ActiveElement, "DependencyType", None)
                        if dep is not None:
                            dutils.activate_type(dep, "Absolute")
                        
                        dutils.set_activity_constant(lat0, t_val)
                        print(f"  Set lateral deviation: {t_val:.3f}m")
                except Exception as e:
                    print(f"  Warning: Could not set lateral position: {e}")
            
            self._ego_created = True
            return ego_maneuver
            
        except Exception as e:
            print(f"  Error configuring ego vehicle: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def createFellowInSimulator(self, obj):
        """Create a Fellow vehicle (non-ego) using the Fellows API.
        
        This is the original logic for creating Fellows.
        """

        # 1) Project Scenic (x,y) → (s,t). If no map, use zeros.
        if getattr(obj, "position", None) is not None:
            scenic_x, scenic_y = obj.position.x, obj.position.y
            
            # Apply coordinate transformation if available (XODR→RD correction)
            if self._coordinate_transform is not None:
                from . import coordinate_transform
                transformed_x, transformed_y = coordinate_transform.apply_coordinate_transform(
                    self._coordinate_transform, (scenic_x, scenic_y)
                )
                print(f"Scenic coords ({scenic_x:.3f}, {scenic_y:.3f}) -> "
                      f"RD coords ({transformed_x:.3f}, {transformed_y:.3f})")
                work_x, work_y = transformed_x, transformed_y
            else:
                work_x, work_y = scenic_x, scenic_y
            
            # Use road index for proper geometric projection
            if self._road_index:
                s_val, t_val = dutils.project_world_to_st(self._road_index, (work_x, work_y))
                
                # Auto-detect route and adjust s-coordinate if needed
                route_pref = self._detect_route_from_road_segment(obj)
                if route_pref == "Pit":
                    # For pit lane, ensure s-coordinate is reasonable for pit lane length (~883m)
                    if s_val > 1000:  # If s-value seems too high for pit lane
                        s_val = s_val % 883.4  # Wrap to pit lane length
                
                print(f"World coordinates ({work_x:.3f}, {work_y:.3f}) -> Road coordinates (s={s_val:.1f}, t={t_val:.3f})")
            else:
                # Use direct projection to road network
                s_val, t_val = dutils.project_world_to_st(self._road_index, (work_x, work_y))
                print(f"World coordinates ({work_x:.3f}, {work_y:.3f}) -> Fallback coordinates (s={s_val:.1f}, t={t_val:.3f})")
        else:
            s_val, t_val = 0.0, 0.0
            print("Warning: No position available, using default coordinates (s=0, t=0)")

        # 3) Create Fellow with one Sequence and two Segments
        F = self.ts.Fellows.Add()
        
        # Set a unique name
        try:
            if getattr(obj, "name", None):
                F.Name = str(obj.name)
            else:
                F.Name = f"Fellow_{self.ts.Fellows.Count}"
            print(f"    Created Fellow with name: {F.Name}")
        except Exception as e:
            F.Name = f"Fellow_{self.ts.Fellows.Count}"
            print(f"    Created Fellow with fallback name: {F.Name} (error: {e})")

        seqs = F.Sequences
        dutils.clear_collection(seqs)
        S1 = seqs.Add() if hasattr(seqs, "Add") else seqs.Item(0)
        segs = dutils.ensure_two_segments(S1)

        # 4) seg0 = ABSOLUTE pose: Position = s, Deviation(Absolute) = t
        dutils.configure_seg0_absolute_pose(segs, s=float(s_val), t=float(t_val))

        # 5) seg1 = Velocity + Endless; keep lateral "Continue"
        # Set velocity to 0 for static scenarios
        base_v = 0.0  # Force velocity to 0 for all vehicles
        dutils.configure_seg1_motion(segs, v=float(base_v), t=float(t_val))
        dutils.make_endless_transition(segs)

        # 6) Set Route via FellowSequence.Route (per updated fellow_starting.md)
        self._set_fellow_route_via_sequence(S1, obj)

        # Store fellow vehicle reference for dynamic control
        self._fellow_vehicles[F.Name] = {
            'fellow_object': F,
            'sequence': S1,
            'segments': segs,
            'scenic_object': obj
        }

        return F
    
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
            
            # Configure fellows based on Scenic objects
            for scenic_obj in self.scene.objects:
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
            
            # Determine route based on Scenic placement
            track_segment = self.detectTrackSegment((scenic_obj.position.x, scenic_obj.position.y))
            if track_segment == 'pitLane':
                route_name = "PitLane"  # Adjust based on actual route names
            else:
                route_name = "IMS_Main"  # Adjust based on actual route names
            
            # Activate route
            try:
                route_sel.Activate(route_name)
                print(f"[ModelDesk] Set route '{route_name}' for {fellow_name}")
            except Exception as e:
                print(f"[ModelDesk] Failed to set route '{route_name}' for {fellow_name}: {e}")
                # Try to list available routes
                try:
                    available_routes = list(route_sel.AvailableElements)
                    print(f"[ModelDesk] Available routes: {available_routes}")
                except:
                    pass
            
            # Set direction (forward)
            route_sel.Direction = 1
            route_sel.UseExternal = True  # Enable external control for fellow vehicles
            
            # Store fellow reference for runtime control
            self._fellow_vehicles[fellow_name] = {
                'fellow_object': fellow,
                'scenic_object': scenic_obj,
                'sequence': seq
            }
            
        except Exception as e:
            print(f"[ModelDesk] Error configuring fellow: {e}")
    
    def setVehicleControl(self, vehicle_name, throttle=None, brake=None, steering=None, velocity=None):
        """Set dynamic control inputs for a fellow vehicle using ControlDesk."""
        return self._per_tick_controller.setVehicleControl(vehicle_name, throttle, brake, steering, velocity)
    
    def startPerTickControl(self, vehicle_name, control_function=None, dt=0.01):
        """Start per-tick control loop for a vehicle."""
        return self._per_tick_controller.startPerTickControl(vehicle_name, control_function, dt)
    
    def stopPerTickControl(self, vehicle_name):
        """Stop per-tick control loop for a vehicle."""
        return self._per_tick_controller.stopPerTickControl(vehicle_name)
    
    
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
    
    
    def detectTrackSegment(self, position):
        """Detect which track segment a position belongs to using actual road projection.
        
        Uses the road index to determine which road the position actually projects onto,
        rather than distance-based heuristics.
        
        Args:
            position: (x, y) world coordinates
            
        Returns:
            String indicating the track segment: 'mainRacing', 'pitLane', or None
        """
        try:
            # Get the racing track regions from params (set by racing/model.scenic)
            params = getattr(self.scene, "params", {}) or {}
            pit_lane_ids = params.get('pitLaneRoadIds', [])
            main_racing_ids = params.get('mainRacingRoadIds', [])
            
            if not pit_lane_ids and not main_racing_ids:
                # Not a racing scenario
                return None
            
            obj_x, obj_y = float(position[0]), float(position[1])
            
            # Use actual road projection to determine which road the position is on
            if self._road_index:
                # Project onto road network to find the actual road
                s_val, t_val = dutils.project_world_to_st(self._road_index, (obj_x, obj_y))
                
                # Find which road this position projects onto
                projected_road_id = dutils.find_road_id_for_position(self._road_index, obj_x, obj_y)
                
                if projected_road_id:
                    # Check if this road ID belongs to pit lane or main racing
                    if str(projected_road_id) in pit_lane_ids:
                        return 'pitLane'
                    elif str(projected_road_id) in main_racing_ids:
                        return 'mainRacing'
            
            # If projection fails, return None (no fallback)
            return None
                
        except Exception as e:
            print(f"    [TrackSegment] Detection error: {e}")
            return None


    def assignRoute(self, agent, track_segment):
        """Assign appropriate route based on track segment (dSPACE-specific).
        
        Maps track segments to dSPACE ModelDesk route names.
        
        Args:
            agent: The racing agent
            track_segment: Track segment identifier ('mainRacing' or 'pitLane')
            
        Returns:
            String indicating the route preference for dSPACE
        """
        if track_segment == 'pitLane':
            return 'Pit'
        elif track_segment == 'mainRacing':
            return 'Lap'
        else:
            return None
    
    def _set_fellow_route_via_sequence(self, sequence, obj):
        """Set the ModelDesk route for a fellow vehicle via FellowSequence.Route.
        
        Automatically detects appropriate route based on road segment placement.
        Falls back to explicit dspace_route attribute if auto-detection fails.
        
        Args:
            sequence: The fellow's sequence object (FellowSequence)
            obj: The Scenic object (vehicle)
        """
        try:
            # Access Route property on the sequence
            if not hasattr(sequence, 'Route'):
                print(f"    [INFO] Sequence has no Route property, skipping route assignment")
                return
            
            route_sel = sequence.Route
            print(f"    [Route] Accessed FellowSequence.Route successfully")
            
            # Determine route preference
            route_preference = None
            
            # Priority 1: Auto-detect from road segment (clean design)
            route_preference = self._detect_route_from_road_segment(obj)
            if route_preference:
                print(f"    [Route] Auto-detected from placement: {route_preference}")
            # Priority 2: Explicit attribute (backward compatibility)
            elif hasattr(obj, 'dspace_route'):
                route_preference = obj.dspace_route
                print(f"    [Route] Using explicit attribute: {route_preference}")
            else:
                print(f"    [Route] No route preference determined")
            
            # Check available routes - try multiple methods
            available_routes = []
            
            # Method 1: Try AvailableElements directly
            if hasattr(route_sel, 'AvailableElements'):
                try:
                    available = route_sel.AvailableElements
                    if available is not None:
                        # Try to iterate even if Count doesn't work
                        if hasattr(available, 'Count'):
                            try:
                                count = available.Count
                                print(f"    [Route] Available routes via AvailableElements: {count}")
                                
                                for i in range(count):
                                    try:
                                        route = available.Item(i)
                                        if route:
                                            name = route.Name if hasattr(route, 'Name') else f"Route{i}"
                                            available_routes.append((i, name, route))
                                            print(f"      Route {i}: {name}")
                                    except Exception as e:
                                        print(f"      Route {i}: (error: {e})")
                            except Exception as e:
                                print(f"    [WARN] Could not get Count from AvailableElements: {e}")
                        
                        # Try iteration without Count
                        if not available_routes:
                            try:
                                for i, route in enumerate(available):
                                    if route:
                                        name = route.Name if hasattr(route, 'Name') else f"Route{i}"
                                        available_routes.append((i, name, route))
                                        print(f"      Route {i}: {name} (via iteration)")
                            except Exception as e:
                                print(f"    [INFO] Could not iterate AvailableElements: {e}")
                    else:
                        print(f"    [INFO] AvailableElements is None")
                except Exception as e:
                    print(f"    [WARN] Error accessing AvailableElements: {e}")
            
            # Method 2: Try getting current ActiveElement to see if routes exist
            if not available_routes and hasattr(route_sel, 'ActiveElement'):
                try:
                    active = route_sel.ActiveElement
                    if active:
                        name = active.Name if hasattr(active, 'Name') else "Unknown"
                        print(f"    [INFO] Found ActiveElement: {name}")
                        print(f"    [INFO] Routes exist but AvailableElements not accessible")
                        print(f"    [INFO] Will try to activate by name directly")
                except Exception as e:
                    print(f"    [INFO] Could not access ActiveElement: {e}")
            
            # If still no routes found, give helpful message but continue
            if not available_routes:
                print(f"    [INFO] Could not enumerate routes via AvailableElements")
                print(f"    [INFO] Will attempt direct activation by route name")
                # Continue anyway - try to activate by name even if we can't list them
            
            # Try to activate the route
            if not hasattr(route_sel, 'Activate'):
                print(f"    [INFO] RouteSelection does not support Activate method")
                return
            
            # Try to activate the route
            activated = False
            
            # If we have enumerated routes, use smart matching
            if available_routes and route_preference:
                # Strategy 1: Exact match
                matching_route = None
                for idx, name, route in available_routes:
                    if name.lower() == route_preference.lower():
                        matching_route = (idx, name, route)
                        print(f"    [Match] Found exact match: {name}")
                        break
                
                # Strategy 2: Pattern matching
                if not matching_route:
                    pref_lower = route_preference.lower()
                    for idx, name, route in available_routes:
                        name_lower = name.lower()
                        if pref_lower in name_lower or name_lower in pref_lower:
                            matching_route = (idx, name, route)
                            print(f"    [Match] Found pattern match: {name}")
                            break
                
                # Strategy 3: Heuristic matching by index
                # Common ModelDesk convention: Route0 = Pit, Route1 = Main/Lap
                if not matching_route and len(available_routes) >= 2:
                    pref_lower = route_preference.lower()
                    if pref_lower in ['pit', 'pitlane']:
                        # Prefer Route0 for pit
                        for idx, name, route in available_routes:
                            if 'route0' in name.lower() or idx == 0:
                                matching_route = (idx, name, route)
                                print(f"    [Match] Heuristic match for pit: {name} (index {idx})")
                                break
                    elif pref_lower in ['lap', 'main', 'race']:
                        # Prefer Route1 for main/lap
                        for idx, name, route in available_routes:
                            if 'route1' in name.lower() or idx == 1:
                                matching_route = (idx, name, route)
                                print(f"    [Match] Heuristic match for lap: {name} (index {idx})")
                                break
                
                # Activate matched route
                if matching_route:
                    idx, name, route = matching_route
                    
                    # Try multiple activation methods
                    activation_methods = [
                        ('by index', lambda: route_sel.Activate(idx)),
                        ('by name', lambda: route_sel.Activate(name)),
                        ('by route object', lambda: route_sel.Activate(route)),
                        ('set ActiveElement', lambda: setattr(route_sel, 'ActiveElement', route)),
                    ]
                    
                    for method_name, activate_fn in activation_methods:
                        try:
                            activate_fn()
                            print(f"    [OK] Activated route '{name}' (index {idx}) {method_name}")
                            activated = True
                            break
                        except Exception as e:
                            print(f"    [DEBUG] Activation {method_name} failed: {e}")
                            continue
            
            # If no enumerated routes, try direct activation with common names
            if not activated and route_preference:
                # Try common route name variations
                names_to_try = [
                    route_preference,  # Try preference as-is
                ]
                
                # Add variations
                pref_lower = route_preference.lower()
                if pref_lower in ['pit', 'pitlane']:
                    names_to_try.extend(['Pit', 'PitLane', 'Pit Lane', 'pit', 'pitlane', 'Route_1'])
                elif pref_lower in ['lap', 'main', 'race']:
                    names_to_try.extend(['Lap', 'Main', 'MainRoute', 'Main Route', 'lap', 'main', 'Route_2'])
                
                print(f"    [INFO] Trying direct activation with name variants...")
                for name in names_to_try:
                    try:
                        route_sel.Activate(name)
                        print(f"    [OK] Successfully activated route: '{name}'")
                        activated = True
                        break
                    except Exception as e:
                        print(f"    [DEBUG] Could not activate '{name}': {e}")
                        continue
            
            # Verify final activation status
            if activated:
                try:
                    if hasattr(route_sel, 'ActiveElement') and route_sel.ActiveElement:
                        active_name = route_sel.ActiveElement.Name if hasattr(route_sel.ActiveElement, 'Name') else "Unknown"
                        print(f"    [OK] Active route confirmed: '{active_name}'")
                except:
                    pass
            else:
                print(f"    [WARN] Could not activate any route for preference: {route_preference}")
                
        except Exception as e:
            print(f"    [WARN] Could not set route via sequence: {e}")
    


    def step(self):
        time.sleep(self.timestep)

    def getProperties(self, obj, properties):
        b = getattr(obj, "_backend", None)
        pos = getattr(b, "position", Vector(0, 0, 0))
        vel = getattr(b, "linvel",   Vector(0, 0, 0))
        ang = getattr(b, "angvel",   Vector(0, 0, 0))
        yaw = getattr(b, "heading",  0.0)
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
        return {k: vals[k] for k in properties}



        seqs = F.Sequences

        dutils.clear_collection(seqs)

        S1 = seqs.Add() if hasattr(seqs, "Add") else seqs.Item(0)

        segs = dutils.ensure_two_segments(S1)



        # 4) seg0 = ABSOLUTE pose: Position = s, Deviation(Absolute) = t

        dutils.configure_seg0_absolute_pose(segs, s=float(s_val), t=float(t_val))



        # 5) seg1 = Velocity + Endless; keep lateral "Continue"

        # Set velocity to 0 for static scenarios

        base_v = 0.0  # Force velocity to 0 for all vehicles

        dutils.configure_seg1_motion(segs, v=float(base_v), t=float(t_val))

        dutils.make_endless_transition(segs)



        # 6) Set Route via FellowSequence.Route (per updated fellow_starting.md)

        self._set_fellow_route_via_sequence(S1, obj)



        return F

    

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

    

    

    def detectTrackSegment(self, position):

        """Detect which track segment a position belongs to using actual road projection.
        

        Uses the road index to determine which road the position actually projects onto,
        rather than distance-based heuristics.
        

        Args:

            position: (x, y) world coordinates

            

        Returns:

            String indicating the track segment: 'mainRacing', 'pitLane', or None

        """

        try:

            # Get the racing track regions from params (set by racing/model.scenic)

            params = getattr(self.scene, "params", {}) or {}

            pit_lane_ids = params.get('pitLaneRoadIds', [])

            main_racing_ids = params.get('mainRacingRoadIds', [])

            

            if not pit_lane_ids and not main_racing_ids:

                # Not a racing scenario

                return None

            

            obj_x, obj_y = float(position[0]), float(position[1])

            

            # Use actual road projection to determine which road the position is on
            if self._road_index:
                # Project onto road network to find the actual road
                s_val, t_val = dutils.project_world_to_st(self._road_index, (obj_x, obj_y))
                
                # Find which road this position projects onto
                projected_road_id = dutils.find_road_id_for_position(self._road_index, obj_x, obj_y)
                
                if projected_road_id:
                    # Check if this road ID belongs to pit lane or main racing
                    if str(projected_road_id) in pit_lane_ids:
                        return 'pitLane'
                    elif str(projected_road_id) in main_racing_ids:
                        return 'mainRacing'

            
            # If projection fails, return None (no fallback)
            return None
                

        except Exception as e:

            print(f"    [TrackSegment] Detection error: {e}")

            return None




    def assignRoute(self, agent, track_segment):

        """Assign appropriate route based on track segment (dSPACE-specific).

        

        Maps track segments to dSPACE ModelDesk route names.

        

        Args:

            agent: The racing agent

            track_segment: Track segment identifier ('mainRacing' or 'pitLane')

            

        Returns:

            String indicating the route preference for dSPACE

        """

        if track_segment == 'pitLane':

            return 'Pit'

        elif track_segment == 'mainRacing':

            return 'Lap'

        else:

            return None

    

    def _set_fellow_route_via_sequence(self, sequence, obj):

        """Set the ModelDesk route for a fellow vehicle via FellowSequence.Route.

        

        Automatically detects appropriate route based on road segment placement.

        Falls back to explicit dspace_route attribute if auto-detection fails.

        

        Args:

            sequence: The fellow's sequence object (FellowSequence)

            obj: The Scenic object (vehicle)

        """

        try:

            # Access Route property on the sequence

            if not hasattr(sequence, 'Route'):

                print(f"    [INFO] Sequence has no Route property, skipping route assignment")

                return

            

            route_sel = sequence.Route

            print(f"    [Route] Accessed FellowSequence.Route successfully")

            

            # Determine route preference

            route_preference = None

            

            # Priority 1: Auto-detect from road segment (clean design)

            route_preference = self._detect_route_from_road_segment(obj)

            if route_preference:

                print(f"    [Route] Auto-detected from placement: {route_preference}")

            # Priority 2: Explicit attribute (backward compatibility)

            elif hasattr(obj, 'dspace_route'):

                route_preference = obj.dspace_route

                print(f"    [Route] Using explicit attribute: {route_preference}")

            else:

                print(f"    [Route] No route preference determined")

            

            # Check available routes - try multiple methods

            available_routes = []

            

            # Method 1: Try AvailableElements directly

            if hasattr(route_sel, 'AvailableElements'):

                try:

                    available = route_sel.AvailableElements

                    if available is not None:

                        # Try to iterate even if Count doesn't work

                        if hasattr(available, 'Count'):

                            try:

                                count = available.Count

                                print(f"    [Route] Available routes via AvailableElements: {count}")

                                

                                for i in range(count):

                                    try:

                                        route = available.Item(i)

                                        if route:

                                            name = route.Name if hasattr(route, 'Name') else f"Route{i}"

                                            available_routes.append((i, name, route))

                                            print(f"      Route {i}: {name}")

                                    except Exception as e:

                                        print(f"      Route {i}: (error: {e})")

                            except Exception as e:

                                print(f"    [WARN] Could not get Count from AvailableElements: {e}")

                        

                        # Try iteration without Count

                        if not available_routes:

                            try:

                                for i, route in enumerate(available):

                                    if route:

                                        name = route.Name if hasattr(route, 'Name') else f"Route{i}"

                                        available_routes.append((i, name, route))

                                        print(f"      Route {i}: {name} (via iteration)")

                            except Exception as e:

                                print(f"    [INFO] Could not iterate AvailableElements: {e}")

                    else:

                        print(f"    [INFO] AvailableElements is None")

                except Exception as e:

                    print(f"    [WARN] Error accessing AvailableElements: {e}")

            

            # Method 2: Try getting current ActiveElement to see if routes exist

            if not available_routes and hasattr(route_sel, 'ActiveElement'):

                try:

                    active = route_sel.ActiveElement

                    if active:

                        name = active.Name if hasattr(active, 'Name') else "Unknown"

                        print(f"    [INFO] Found ActiveElement: {name}")

                        print(f"    [INFO] Routes exist but AvailableElements not accessible")

                        print(f"    [INFO] Will try to activate by name directly")

                except Exception as e:

                    print(f"    [INFO] Could not access ActiveElement: {e}")

            

            # If still no routes found, give helpful message but continue

            if not available_routes:

                print(f"    [INFO] Could not enumerate routes via AvailableElements")

                print(f"    [INFO] Will attempt direct activation by route name")

                # Continue anyway - try to activate by name even if we can't list them

            

            # Try to activate the route

            if not hasattr(route_sel, 'Activate'):

                print(f"    [INFO] RouteSelection does not support Activate method")

                return

            

            # Try to activate the route

            activated = False

            

            # If we have enumerated routes, use smart matching

            if available_routes and route_preference:

                # Strategy 1: Exact match

                matching_route = None

                for idx, name, route in available_routes:

                    if name.lower() == route_preference.lower():

                        matching_route = (idx, name, route)

                        print(f"    [Match] Found exact match: {name}")

                        break

                

                # Strategy 2: Pattern matching

                if not matching_route:

                    pref_lower = route_preference.lower()

                    for idx, name, route in available_routes:

                        name_lower = name.lower()

                        if pref_lower in name_lower or name_lower in pref_lower:

                            matching_route = (idx, name, route)

                            print(f"    [Match] Found pattern match: {name}")

                            break

                

                # Strategy 3: Heuristic matching by index

                # Common ModelDesk convention: Route0 = Pit, Route1 = Main/Lap

                if not matching_route and len(available_routes) >= 2:

                    pref_lower = route_preference.lower()

                    if pref_lower in ['pit', 'pitlane']:

                        # Prefer Route0 for pit

                        for idx, name, route in available_routes:

                            if 'route0' in name.lower() or idx == 0:

                                matching_route = (idx, name, route)

                                print(f"    [Match] Heuristic match for pit: {name} (index {idx})")

                                break

                    elif pref_lower in ['lap', 'main', 'race']:

                        # Prefer Route1 for main/lap

                        for idx, name, route in available_routes:

                            if 'route1' in name.lower() or idx == 1:

                                matching_route = (idx, name, route)

                                print(f"    [Match] Heuristic match for lap: {name} (index {idx})")

                                break

                

                # Activate matched route

                if matching_route:

                    idx, name, route = matching_route

                    

                    # Try multiple activation methods

                    activation_methods = [

                        ('by index', lambda: route_sel.Activate(idx)),

                        ('by name', lambda: route_sel.Activate(name)),

                        ('by route object', lambda: route_sel.Activate(route)),

                        ('set ActiveElement', lambda: setattr(route_sel, 'ActiveElement', route)),

                    ]

                    

                    for method_name, activate_fn in activation_methods:

                        try:

                            activate_fn()

                            print(f"    [OK] Activated route '{name}' (index {idx}) {method_name}")

                            activated = True

                            break

                        except Exception as e:

                            print(f"    [DEBUG] Activation {method_name} failed: {e}")

                            continue

            

            # If no enumerated routes, try direct activation with common names

            if not activated and route_preference:

                # Try common route name variations

                names_to_try = [

                    route_preference,  # Try preference as-is

                ]

                

                # Add variations

                pref_lower = route_preference.lower()

                if pref_lower in ['pit', 'pitlane']:

                    names_to_try.extend(['Pit', 'PitLane', 'Pit Lane', 'pit', 'pitlane', 'Route_1'])

                elif pref_lower in ['lap', 'main', 'race']:

                    names_to_try.extend(['Lap', 'Main', 'MainRoute', 'Main Route', 'lap', 'main', 'Route_2'])

                

                print(f"    [INFO] Trying direct activation with name variants...")

                for name in names_to_try:

                    try:

                        route_sel.Activate(name)

                        print(f"    [OK] Successfully activated route: '{name}'")

                        activated = True

                        break

                    except Exception as e:

                        print(f"    [DEBUG] Could not activate '{name}': {e}")

                        continue

            

            # Verify final activation status

            if activated:

                try:

                    if hasattr(route_sel, 'ActiveElement') and route_sel.ActiveElement:

                        active_name = route_sel.ActiveElement.Name if hasattr(route_sel.ActiveElement, 'Name') else "Unknown"

                        print(f"    [OK] Active route confirmed: '{active_name}'")

                except:

                    pass

            else:

                print(f"    [WARN] Could not activate any route for preference: {route_preference}")

                

        except Exception as e:

            print(f"    [WARN] Could not set route via sequence: {e}")

    




    def step(self):

        time.sleep(self.timestep)



    def getProperties(self, obj, properties):

        b = getattr(obj, "_backend", None)

        pos = getattr(b, "position", Vector(0, 0, 0))

        vel = getattr(b, "linvel",   Vector(0, 0, 0))

        ang = getattr(b, "angvel",   Vector(0, 0, 0))

        yaw = getattr(b, "heading",  0.0)

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

        return {k: vals[k] for k in properties}


