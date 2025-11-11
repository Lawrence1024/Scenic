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
from scenic.core.regions import PolylineRegion

from . import utils as dutils
from .controldesk.per_tick_control import ExternalControlManager
from .controldesk.connection import ControlDeskApp
from .vehicle import VehiclePhysicsState, VehicleController


class DSpaceVehicleActor:
    """Internal representation of a vehicle in the dSPACE simulator.
    
    This class stores the current state of a vehicle as read from ControlDesk,
    providing a bridge between the dSPACE simulation and Scenic's object model.
    
    Attributes:
        scenic_obj: Reference to the parent Scenic object
        position: Current position as Vector(x, y, z) in meters
        linvel: Linear velocity as Vector(vx, vy, vz) in m/s
        angvel: Angular velocity as Vector(wx, wy, wz) in rad/s
        heading: Heading angle (yaw) in radians
    """
    
    def __init__(self, scenic_obj):
        """Initialize dSPACE actor for a vehicle.
        
        Args:
            scenic_obj: The parent Scenic vehicle object
        """
        self.scenic_obj = scenic_obj
        self.position = Vector(0, 0, 0)
        self.linvel = Vector(0, 0, 0)
        self.angvel = Vector(0, 0, 0)
        self.heading = 0.0
        
        # Initialize physics model for kinematic control (used for fellow vehicles)
        self.physics = VehiclePhysicsState(
            initial_velocity=0.0,
            initial_deviation=0.0
        )
        
        # Decision tree state variables
        self.speed_limit = None  # Current speed limit (m/s)
        self.speed_type = None  # "stop", "pit_crawl", "pit_lane", "yellow", "green", etc.
        self.ttl_selection = None  # "left", "right", "race", "optimal", "pit"
        self.target_gap = None  # Target following gap (meters)
        self.gap_type = None  # "no_gap", "attacker_preparing", "attacker_attacking", etc.
        self.strategy_type = "cruise_control"  # "cruise_control" or "follow_mode"
        self.scale_factor = 1.0  # Speed scale factor (0.0-1.0)
        self.powertrain_mode = "nominal"  # "pit_lane", "quiet", "nominal", "race", "overboost"
        self.push2pass_active = False  # Push2Pass activation state
    
    def set_control(self, control_dict):
        """Set control parameters (used by setMaxSpeed, setTTL, etc.).
        
        This method is called by the vehicle's control methods to pass
        control information to the simulator.
        
        Args:
            control_dict: Dictionary of control parameters
        """
        # Store control parameters for potential future use
        if not hasattr(self, '_control_params'):
            self._control_params = {}
        self._control_params.update(control_dict)

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
        super().__init__(scene, timestep=ts, **kwargs)

    # --- TTL (Target Trajectory Line) loading utilities ---
    def _get_ttl_config(self):
        """Read TTL configuration from scene params or use defaults.
        
        Returns:
            (ttl_folder, ttl_index, dx, dy)
        """
        import os
        params = getattr(self.scene, "params", {}) or {}
        # Defaults
        repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
        default_folder = os.path.join(repo_root, "assets", "ttls", "LS_ENU_TTL_CSV", "usable")
        ttl_folder = params.get("ttlFolder", default_folder)
        ttl_index = int(params.get("ttlIndex", 17))  # 1-based; default to 17
        # Global coarse alignment for all TTLs (common offset)
        dx = float(params.get("ttlDX", -53.6))
        dy = float(params.get("ttlDY", -15.7))
        return ttl_folder, ttl_index, dx, dy

    def _load_ttl_region(self):
        """Load a TTL CSV as a PolylineRegion, applying global transform (dx,dy)."""
        import os, csv
        ttl_folder, ttl_index, dx, dy = self._get_ttl_config()
        try:
            # Support either numbered ttl_N.csv or explicit original names already renamed to ttl_N.csv
            ttl_path = os.path.join(ttl_folder, f"ttl_{ttl_index}.csv")
            if not os.path.exists(ttl_path):
                print(f"[TTL] File not found: {ttl_path}")
                return None
            pts = []
            with open(ttl_path, newline="") as f:
                r = csv.reader(f)
                # skip metadata
                try:
                    next(r)
                except StopIteration:
                    pass
                for row in r:
                    if not row or len(row) < 2:
                        continue
                    try:
                        x = float(row[0]) + dx
                        y = float(row[1]) + dy
                        pts.append((x, y))
                    except Exception:
                        continue
            if len(pts) < 2:
                print(f"[TTL] Not enough points in {ttl_path}")
                return None
            region = PolylineRegion(pts)
            print(f"[TTL] Loaded {len(pts)} points from {ttl_path} with offset ({dx}, {dy})")
            # Keep a copy of transformed points for optional waypoint usage
            self._ttl_points_loaded = pts
            return region
        except Exception as e:
            print(f"[TTL] Error loading TTL: {e}")
            return None

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
                    from .geometry import coordinate_transform
                    
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
                    from .geometry.rd_parser import build_rd_road_index
                    self._road_index = build_rd_road_index(rd_path, step=0.5)
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
        try:
            self._cd = ControlDeskApp().connect()
            self._cd.go_online()
            self._cd.start_measurement()
            print("[ControlDesk] Online and measuring")
            
            # Initialize VehicleController for applying controls
            self._vehicle_controller = VehicleController(self)
            print("[VehicleController] Initialized")
            
            
            # Initialize VesiInterface manual control BEFORE starting maneuver
            self._cd.initialize_vesi_interface()

            self._cd.set_simulation_step(0.01)
            

            print("[VesiInterface] ✅ Initialization complete - ready for manual control")
        except Exception as e:
            print(f"[ControlDesk] ⚠️  Not available: {e}")
            self._cd = None
            self._vehicle_controller = None

        # 10) NOW start the maneuver via ControlDesk (AFTER VesiInterface is fully initialized)
        if self._cd:
            print("[ControlDesk] Starting maneuver (VesiInterface initialized)...")
            try:
                self._cd.start_maneuver()
                print("[Maneuver] ✅ Started via ControlDesk - VesiInterface controls active")
            except Exception as e:
                print(f"[Maneuver] ❌ Failed to start: {e}")
        else:
            print("[Maneuver] ⚠️  Skipping start - ControlDesk not available")
        
        # Pause simulation initially for step-by-step control
        if self._cd:
            self._cd.pause_simulation()
            # Immediately try to warm-up fellow arrays so first read/write won't warn
            self._ensureFellowArraysInitialized()
            # Optional: small additional delay to give ModelDesk time to spawn fellows fully
            # time.sleep(self.timestep)

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
        
        # 1) Project Scenic (x,y) → (s,t)
        if getattr(obj, "position", None) is not None:
            scenic_x, scenic_y = obj.position.x, obj.position.y
            
            # Apply coordinate transformation if available
            if self._coordinate_transform is not None:
                from .geometry.coordinate_transform import apply_coordinate_transform
                transformed_x, transformed_y = apply_coordinate_transform(
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
            # Scenic uses yaw=0 for +Y direction, but OpenDRIVE/dSPACE uses yaw=0 for +X direction
            # Need to convert: dSPACE_orientation = scenic_heading - π/2
            if hasattr(obj, 'heading'):
                import math
                dspace_orientation = obj.heading - math.pi / 2
                seq.VehicleOrientation = dspace_orientation
                print(f"  Set orientation: {math.degrees(dspace_orientation):.1f} degrees (from Scenic heading {math.degrees(obj.heading):.1f})")
            else:
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

            # 5) Set route for ego using same logic as fellows (if Route is available)
            try:
                self._set_fellow_route_via_sequence(seq, obj)
            except Exception as e:
                print(f"  [Route] Could not set route for ego: {e}")
            return ego_maneuver
            
        except Exception as e:
            print(f"  Error configuring ego vehicle: {e}")
            import traceback
            traceback.print_exc()
            return None
        finally:
            # Assign TTL to ego if available
            try:
                ttl_region = self._load_ttl_region()
                if ttl_region is not None:
                    setattr(obj, "ttl", ttl_region)
                    print(f"[TTL] Assigned TTL PolylineRegion to ego vehicle")
                    # Optional: also attach waypoints sampled from TTL (for behaviors/tools)
                    if hasattr(self, "_ttl_points_loaded") and self._ttl_points_loaded:
                        setattr(obj, "waypoints", list(self._ttl_points_loaded))
                        print(f"[TTL] Attached {len(self._ttl_points_loaded)} TTL waypoints to ego")
            except Exception as _e:
                print(f"[TTL] Could not assign TTL to ego: {_e}")
    
    def createFellowInSimulator(self, obj):
        """Create a Fellow vehicle (non-ego) using the Fellows API.
        
        This is the original logic for creating Fellows.
        """

        # 1) Project Scenic (x,y) → (s,t). If no map, use zeros.
        if getattr(obj, "position", None) is not None:
            scenic_x, scenic_y = obj.position.x, obj.position.y
            
            # Apply coordinate transformation if available (XODR→RD correction)
            if self._coordinate_transform is not None:
                from .geometry.coordinate_transform import apply_coordinate_transform
                transformed_x, transformed_y = apply_coordinate_transform(
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

        # 5) seg1 = set Longitudinal to constant Velocity = 0, Lateral = Continue; then make segment endless
        try:
            # Force fellows to remain stationary unless externally controlled
            dutils.configure_seg1_motion(segs, v=0.0, t=0.0)
        except Exception as e:
            print(f"    Warning: could not set seg1 Velocity(0)/Continue: {e}")
        # Ensure the segment is endless so external velocity can drive motion
        try:
            dutils.make_endless_transition(segs)
        except Exception:
            pass

        # Set orientation (if sequence supports it)
        # Scenic uses yaw=0 for +Y direction, but OpenDRIVE/dSPACE uses yaw=0 for +X direction
        # Need to convert: dSPACE_orientation = scenic_heading - π/2
        if hasattr(obj, 'heading'):
            try:
                import math
                dspace_orientation = obj.heading - math.pi / 2
                if hasattr(S1, 'VehicleOrientation'):
                    S1.VehicleOrientation = dspace_orientation
                    print(f"    Set orientation: {math.degrees(dspace_orientation):.1f} degrees (from Scenic heading {math.degrees(obj.heading):.1f})")
            except Exception as e:
                print(f"    Note: Cannot set orientation for Fellow (not supported or error: {e})")

        # 6) Set Route via FellowSequence.Route (per updated fellow_starting.md)
        self._set_fellow_route_via_sequence(S1, obj)

        # Store fellow vehicle reference for dynamic control
        # Use the ModelDesk-assigned name (e.g., "Fellow_1", "F1", etc.)
        self._fellow_vehicles[F.Name] = {
            'fellow_object': F,
            'sequence': S1,
            'segments': segs,
            'scenic_object': obj,
            'index': self.ts.Fellows.Count - 1  # Store the actual 0-based index
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
            self._ensureFellowArraysInitialized()
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
        if not hasattr(obj, 'dspaceActor') or obj.dspaceActor is None:
            # Create dSPACE actor representation
            obj.dspaceActor = DSpaceVehicleActor(obj)
            
            # Initialize with object's initial position if available
            if hasattr(obj, 'position'):
                obj.dspaceActor.position = obj.position
            if hasattr(obj, 'heading'):
                obj.dspaceActor.heading = obj.heading
    
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
        try:
            # Ego vehicle paths (adapt from fellow paths)
            # These paths may need adjustment based on actual ControlDesk structure
            base_path = "Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel"
            
            # Try to read ego position and state
            try:
                x = self._cd.get_var(f"{base_path}/Ego_x/Value")
                y = self._cd.get_var(f"{base_path}/Ego_y/Value")
                z = self._cd.get_var(f"{base_path}/Ego_z/Value")
                yaw_deg = self._cd.get_var(f"{base_path}/Ego_yaw/Value")
                velocity = self._cd.get_var(f"{base_path}/Ego_velocity/Value")
                
                # Update dspaceActor state
                obj.dspaceActor.position = Vector(float(x), float(y), float(z))
                obj.dspaceActor.heading = float(yaw_deg) * (3.14159265 / 180.0)  # Convert to radians
                obj.dspaceActor.linvel = Vector(float(velocity), 0, 0)  # Simplified
                
                return True
            except Exception:
                # Fallback: Keep current state if can't read from ControlDesk
                return False
                
        except Exception as e:
            print(f"[_readEgoStateFromControlDesk] Error: {e}")
            return False
    
    def _ensureFellowArraysInitialized(self):
        """Advance the simulation a few steps to allow fellow arrays to initialize."""
        if self._fellow_arrays_initialized:
            return
        if self._initializing_fellow_arrays:
            return
        if not self._cd:
            return
        base_path = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/FELLOW_POS_VEL/FellowTrailer"
        array_path = f"{base_path}/x"
        self._initializing_fellow_arrays = True
        try:
            for attempt in range(200):
                x_arr = y_arr = yaw_arr = None
                try:
                    x_arr = self._cd.get_var(array_path)  # x array
                except Exception:
                    x_arr = None
                try:
                    y_arr = self._cd.get_var(f"{base_path}/y")
                except Exception:
                    y_arr = None
                try:
                    yaw_arr = self._cd.get_var(f"{base_path}/yaw_deg_out")
                except Exception:
                    yaw_arr = None

                ready = False
                if isinstance(x_arr, (list, tuple)) and len(x_arr) > 0:
                    # consider arrays initialized only if we see at least one non-zero signal
                    # (heuristic to wait for plant to publish real data)
                    def has_signal(arr):
                        try:
                            for i in range(min(5, len(arr))):
                                val = arr[i]
                                if isinstance(val, (int, float)) and abs(val) > 1e-6:
                                    return True
                        except Exception:
                            pass
                        return False
                    ready = has_signal(x_arr) or has_signal(y_arr or []) or has_signal(yaw_arr or [])

                if ready:
                    self._fellow_arrays_initialized = True
                    self._fellow_index_base = 0  # bulk arrays appear 0-based
                    if attempt > 0:
                        print(f"[dSPACE] Fellow arrays populated after {attempt} warm-up step(s)")
                    break

                # Arrays not ready yet; advance simulation by one step
                if attempt < 199:
                    try:
                        self._cd.advance_simulation_step()
                    except Exception as step_err:
                        print(f"[dSPACE] Unable to advance simulation during fellow array initialization: {step_err}")
                        break
                    time.sleep(self.timestep)
            if not self._fellow_arrays_initialized:
                print("[dSPACE] Fellow arrays still not initialized after warm-up steps")
        finally:
            self._initializing_fellow_arrays = False

    def _probe_external_index_base(self):
        """Probe ControlDesk External_Signals vector paths and index base.
        Determines whether the Value arrays exist and whether indexing is 0-based or 1-based.
        Caches the result on the simulation instance to avoid repeated COM calls.
        """
        if self._ext_probe_done:
            return True
        if not self._cd:
            return False
        base = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/External_Signals"
        v_candidates = (
            f"{base}/Const_v_Fellows_External[km|h]/Value",
            f"{base}/Const_v_Fellows_External[km/h]/Value",
        )
        d_candidates = (
            f"{base}/Const_d_Fellows_External[m]/Value",
        )
        for vpath in v_candidates:
            try:
                # Try bulk read (no index) to ensure the node exists and is an array
                arr = self._cd.get_var(vpath)
                if not isinstance(arr, (list, tuple)):
                    continue
                # Probe index base by trying to read element 0 then 1
                for idx_base in (0, 1):
                    try:
                        _ = self._cd.get_var(f"{vpath}[{idx_base}]")
                        self._ext_v_path = vpath
                        # pick matching d_path from candidates (assume same unit m)
                        self._ext_d_path = d_candidates[0]
                        self._ext_index_base = idx_base
                        self._ext_probe_done = True
                        print(f"[dSPACE] ExternalSignals ready at {vpath}[{idx_base}]")
                        return True
                    except Exception:
                        continue
                # If element addressing not supported, still accept bulk-only mode
                self._ext_v_path = vpath
                self._ext_d_path = d_candidates[0]
                self._ext_index_base = 0
                self._ext_probe_done = True
                print(f"[dSPACE] ExternalSignals available via bulk at {vpath} (no element addressing)")
                return True
            except Exception:
                continue
        # Fallback defaults; may still fail, but we log once
        self._ext_v_path = v_candidates[0]
        self._ext_d_path = d_candidates[0]
        self._ext_index_base = 0
        self._ext_probe_done = True
        print("[dSPACE] ExternalSignals probe failed; using default paths")
        return False
    
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
        if not self._cd:
            return False
        
        self._ensureFellowArraysInitialized()
            
        try:
            # Determine fellow index
            fellow_index = self._getFellowIndex(obj)
            if fellow_index is None:
                return False
            # Adjust for base (0-based vs 1-based arrays)
            eff_index = fellow_index + (self._fellow_index_base or 0)
            
            # Fellow vehicle state paths
            base_path = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/FELLOW_POS_VEL/FellowTrailer"
            
            try:
                # Prefer bulk array reads and index in Python (works in this environment)
                try:
                    x_arr = self._cd.get_var(f"{base_path}/x")
                    if isinstance(x_arr, (list, tuple)):
                        first = x_arr[0] if len(x_arr) > 0 else None
                        at_idx = x_arr[eff_index] if isinstance(eff_index, int) and eff_index < len(x_arr) else None
                        # print(f"[Fellow {fellow_index}] bulk x: len={len(x_arr)}, x[0]={first}, x[{eff_index}]={at_idx}")
                        x = at_idx if at_idx is not None else (first if first is not None else 0.0)
                    else:
                        x = 0.0
                except Exception as _e:
                    print(f"[Fellow {fellow_index}] bulk x read failed: {_e}")
                    x = 0.0
                try:
                    y_arr = self._cd.get_var(f"{base_path}/y")
                    if isinstance(y_arr, (list, tuple)):
                        first = y_arr[0] if len(y_arr) > 0 else None
                        at_idx = y_arr[eff_index] if isinstance(eff_index, int) and eff_index < len(y_arr) else None
                        # print(f"[Fellow {fellow_index}] bulk y: len={len(y_arr)}, y[0]={first}, y[{eff_index}]={at_idx}")
                        y = at_idx if at_idx is not None else (first if first is not None else 0.0)
                    else:
                        y = 0.0
                except Exception as _e:
                    print(f"[Fellow {fellow_index}] bulk y read failed: {_e}")
                    y = 0.0
                try:
                    z_arr = self._cd.get_var(f"{base_path}/z")
                    if isinstance(z_arr, (list, tuple)):
                        first = z_arr[0] if len(z_arr) > 0 else None
                        at_idx = z_arr[eff_index] if isinstance(eff_index, int) and eff_index < len(z_arr) else None
                        # keep z logging minimal
                        z = at_idx if at_idx is not None else (first if first is not None else 0.0)
                    else:
                        z = 0.0
                except Exception:
                    z = 0.0
                try:
                    yaw_arr = self._cd.get_var(f"{base_path}/yaw_deg_out")
                    if isinstance(yaw_arr, (list, tuple)):
                        first = yaw_arr[0] if len(yaw_arr) > 0 else None
                        at_idx = yaw_arr[eff_index] if isinstance(eff_index, int) and eff_index < len(yaw_arr) else None
                        # print(f"[Fellow {fellow_index}] bulk yaw_deg_out: len={len(yaw_arr)}, [0]={first}, [{eff_index}]={at_idx}")
                        yaw_deg = at_idx if at_idx is not None else (first if first is not None else 0.0)
                    else:
                        yaw_deg = 0.0
                except Exception as _e:
                    print(f"[Fellow {fellow_index}] bulk yaw_deg_out read failed: {_e}")
                    yaw_deg = 0.0

                # Optional: try bulk velocity/omega if available; otherwise leave defaults
                v = 0.0
                w = 0.0
                try:
                    v_arr = self._cd.get_var(f"{base_path}/v_Fellows")
                    if isinstance(v_arr, (list, tuple)) and isinstance(eff_index, int) and eff_index < len(v_arr):
                        v = v_arr[eff_index] if v_arr[eff_index] is not None else 0.0
                except Exception:
                    pass
                try:
                    w_arr = self._cd.get_var(f"{base_path}/w_Fellows")
                    if isinstance(w_arr, (list, tuple)) and isinstance(eff_index, int) and eff_index < len(w_arr):
                        w = w_arr[eff_index] if w_arr[eff_index] is not None else 0.0
                except Exception:
                    pass
                
                # Clear warning flag if we successfully read (arrays are now ready)
                if hasattr(obj, '_array_bounds_warning_shown'):
                    delattr(obj, '_array_bounds_warning_shown')
                
                # Update dspaceActor state
                import math
                obj.dspaceActor.position = Vector(float(x), float(y), float(z))
                obj.dspaceActor.heading = float(yaw_deg) * (math.pi / 180.0)  # Convert to radians
                
                # Convert velocity to vector (assuming v is speed, w is angular velocity)
                yaw_rad = obj.dspaceActor.heading
                obj.dspaceActor.linvel = Vector(
                    float(v) * math.cos(yaw_rad),
                    float(v) * math.sin(yaw_rad),
                    0
                )
                obj.dspaceActor.angvel = Vector(0, 0, float(w))
                
                return True
            except Exception as e:
                # Array bounds error - arrays may not be initialized yet
                error_msg = str(e)
                if "Index was outside the bounds" in error_msg or "bounds of the array" in error_msg:
                    # Silently fail - arrays may not be ready yet, use last known state
                    # Only print warning on first occurrence to avoid spam
                    if not hasattr(obj, '_array_bounds_warning_shown'):
                        print(f"[_readFellowStateFromControlDesk] Warning: Fellow {fellow_index} array not ready yet (arrays may not be initialized)")
                        obj._array_bounds_warning_shown = True
                    return False
                else:
                    # Other errors - print for debugging
                    print(f"[_readFellowStateFromControlDesk] Failed to read state for fellow {fellow_index}: {e}")
                    return False
                
        except Exception as e:
            print(f"[_readFellowStateFromControlDesk] Error: {e}")
            return False
    
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
        try:
            # Priority 1: Try to find in fellow_vehicles dict (most reliable)
            # This has the actual ModelDesk-assigned index stored during creation
            for name, vehicle_data in self._fellow_vehicles.items():
                if vehicle_data.get('scenic_object') is obj:
                    # Use stored index if available (set during createFellowInSimulator)
                    if 'index' in vehicle_data:
                        index = int(vehicle_data['index'])  # Ensure integer
                        if 0 <= index < 100:  # Validate bounds
                            return index
                    
                    # Fallback: Extract index from name (handles "F1", "F2", "Fellow_1", etc.)
                    # Try "F" prefix first (e.g., "F1" -> 0, "F2" -> 1)
                    if name.startswith('F') and len(name) > 1:
                        try:
                            # Check if it's "F" followed by digits (e.g., "F1", "F2")
                            if name[1:].isdigit():
                                index = int(name[1:]) - 1
                                if 0 <= index < 100:
                                    return index
                        except ValueError:
                            pass
                    
                    # Try "Fellow_" prefix (e.g., "Fellow_1" -> 0, "Fellow_2" -> 1)
                    if name.startswith('Fellow_'):
                        try:
                            index = int(name[7:]) - 1  # "Fellow_".length = 7
                            if 0 <= index < 100:
                                return index
                        except ValueError:
                            pass
            
            # Priority 2: Use position in objects list (excluding ego)
            # This gives a sequential index based on creation order
            fellow_objects = [o for o in self.scene.objects if o is not self.scene.egoObject]
            if obj in fellow_objects:
                index = int(fellow_objects.index(obj))  # Ensure integer
                # Validate bounds (ControlDesk arrays are typically limited)
                if 0 <= index < 100:
                    return index
            
            # Do NOT use raceNumber - it's randomly assigned (1-999) and doesn't
            # correspond to ControlDesk array indices
            
            print(f"[_getFellowIndex] Could not determine valid index for {obj}")
            return None
        except Exception as e:
            print(f"[_getFellowIndex] Error: {e}")
            import traceback
            traceback.print_exc()
            return None

    def step(self):
        """Execute one simulation step (advance physics simulation).
        
        This advances the dSPACE simulation by one timestep using ControlDesk.
        Control variables should already be written by executeActions() before this is called.
        """
        if self._cd:
            try:
                # Advance simulation by one step using ControlDesk COM interface
                self._cd.advance_simulation_step()
            except Exception as e:
                print(f"[step] Warning: Failed to advance simulation step: {e}")
                # Fallback to sleep
                time.sleep(self.timestep)
        else:
            # No ControlDesk connection, just sleep
            time.sleep(self.timestep)

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
            pit_lane_ids = params.get('pitLaneRoadIds', [])
            main_racing_ids = params.get('mainRacingRoadIds', [])
            if not pit_lane_ids and not main_racing_ids:
                return None
            if not self._road_index:
                return None
            obj_x, obj_y = float(position[0]), float(position[1])
            # Find projected road id using utilities
            projected_road_id = dutils.find_road_id_for_position(self._road_index, obj_x, obj_y)
            print(f"    [TrackSegment] pitLaneRoadIds={pit_lane_ids}, mainRacingRoadIds={main_racing_ids}, projected={projected_road_id}")
            # If the projection returns an internal RD id (e.g., 0/1/2), try mapping to XODR id
            try:
                if projected_road_id is not None and hasattr(dutils, 'map_rd_to_xodr_road_id'):
                    mapped = dutils.map_rd_to_xodr_road_id(self._road_index, projected_road_id)
                    if mapped is not None:
                        print(f"    [TrackSegment] Mapped RD id {projected_road_id} -> XODR id {mapped}")
                        projected_road_id = mapped
            except Exception as e:
                print(f"    [TrackSegment] RD->XODR id mapping not available: {e}")
            if projected_road_id is None:
                return None
            if str(projected_road_id) in pit_lane_ids:
                return 'pitLane'
            if str(projected_road_id) in main_racing_ids:
                return 'mainRacing'
            # Fallback: infer from road name if available
            try:
                if hasattr(dutils, 'get_road_name_for_id'):
                    rname = dutils.get_road_name_for_id(self._road_index, projected_road_id)
                else:
                    rname = None
                if rname:
                    lname = str(rname).lower()
                    if 'pit' in lname:
                        print(f"    [TrackSegment] Fallback by name '{rname}' => pitLane")
                        return 'pitLane'
                    else:
                        print(f"    [TrackSegment] Fallback by name '{rname}' => mainRacing")
                        return 'mainRacing'
            except Exception as e:
                print(f"    [TrackSegment] Name fallback failed: {e}")
            # Final heuristic: common RD ids 0/1/2 → assume 1 is pit, others main
            try:
                if isinstance(projected_road_id, int) and projected_road_id in (0, 1, 2):
                    seg = 'pitLane' if projected_road_id == 1 else 'mainRacing'
                    print(f"    [TrackSegment] Heuristic RD id {projected_road_id} => {seg}")
                    return seg
            except Exception:
                pass
            return None
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
        # dSPACE-specific route assignment
        if track_segment == 'pitLane':
            return 'Pit'  # dSPACE pit lane route
        elif track_segment == 'mainRacing':
            return 'Lap'  # dSPACE main racing route
        else:
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