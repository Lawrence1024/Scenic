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
        
        # Configuration for relative positioning (can be overridden via kwargs)
        self.config = {
            'lateral_calibration_factor': kwargs.pop('lateral_calibration_factor', 0.5),  # Calibration factor for t-coordinate interpretation
            'duplicate_position_threshold': kwargs.pop('duplicate_position_threshold', 0.1),  # Threshold for detecting identical positions (meters)
            'relative_distance_threshold': kwargs.pop('relative_distance_threshold', 50.0),  # Maximum world distance for relative positioning (meters)
            's_coordinate_threshold': kwargs.pop('s_coordinate_threshold', 5.0),  # Maximum s-coordinate difference for relative positioning (meters)
            'heading_difference_threshold': kwargs.pop('heading_difference_threshold', 30.0),  # Maximum heading difference for relative positioning (degrees)
            'lateral_pattern_s_threshold': kwargs.pop('lateral_pattern_s_threshold', 1.0),  # Maximum s-coordinate difference for lateral pattern detection (meters)
            'lateral_pattern_t_threshold': kwargs.pop('lateral_pattern_t_threshold', 1.0),  # Minimum t-coordinate difference for lateral pattern detection (meters)
            'lateral_pattern_heading_threshold': kwargs.pop('lateral_pattern_heading_threshold', 15.0),  # Maximum heading difference for lateral pattern detection (degrees)
            'lateral_pattern_world_threshold': kwargs.pop('lateral_pattern_world_threshold', 20.0),  # Maximum world distance for lateral pattern detection (meters)
        }
        
        ts = kwargs.pop("timestep", None) or sim.timestep
        super().__init__(scene, timestep=ts, **kwargs)
    
    def configure_relative_positioning(self, **config_updates):
        """Configure relative positioning parameters.
        
        Args:
            **config_updates: Configuration parameters to update
                - lateral_calibration_factor: Calibration factor for t-coordinate interpretation (default: 0.5)
                - duplicate_position_threshold: Threshold for detecting identical positions in meters (default: 0.1)
                - relative_distance_threshold: Maximum world distance for relative positioning in meters (default: 50.0)
                - s_coordinate_threshold: Maximum s-coordinate difference for relative positioning in meters (default: 5.0)
                - heading_difference_threshold: Maximum heading difference for relative positioning in degrees (default: 30.0)
                - lateral_pattern_s_threshold: Maximum s-coordinate difference for lateral pattern detection in meters (default: 1.0)
                - lateral_pattern_t_threshold: Minimum t-coordinate difference for lateral pattern detection in meters (default: 1.0)
                - lateral_pattern_heading_threshold: Maximum heading difference for lateral pattern detection in degrees (default: 15.0)
                - lateral_pattern_world_threshold: Maximum world distance for lateral pattern detection in meters (default: 20.0)
        """
        for key, value in config_updates.items():
            if key in self.config:
                self.config[key] = value
                print(f"Updated {key} to {value}")
            else:
                print(f"Warning: Unknown configuration parameter '{key}'")

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

        # 6) Apply relative positioning logic after all objects are created
        self._apply_relative_positioning()

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
                s_val, t_val = dutils.map_scenic_to_modeldesk(work_x, work_y)
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
                s_val, t_val = dutils.map_scenic_to_modeldesk(work_x, work_y)
                print(f"World coordinates ({work_x:.3f}, {work_y:.3f}) -> Fallback coordinates (s={s_val:.1f}, t={t_val:.3f})")
        else:
            s_val, t_val = 0.0, 0.0
            print("Warning: No position available, using default coordinates (s=0, t=0)")

        # 2) Store the object's position for relative positioning analysis
        if not hasattr(self, '_object_positions'):
            self._object_positions = []
        
        # Store which Scenic road/region this object was placed on
        scenic_road_name = None
        if hasattr(obj, 'position'):
            # Try to determine which road the object is on by checking parent region
            try:
                # Check if there's a region attribute or similar
                if hasattr(obj.position, 'region'):
                    scenic_road_name = str(obj.position.region)
            except:
                pass
        
        self._object_positions.append({
            'obj': obj,
            'position': obj.position,
            's_coord': s_val,
            't_coord': t_val,
            'heading': obj.heading,
            'scenic_road': scenic_road_name
        })

        # 3) Create Fellow with one Sequence and two Segments
        F = self.ts.Fellows.Add()
        
        # Set a unique name for relative positioning
        fellow_idx = len(self._object_positions) if hasattr(self, '_object_positions') else 0
        try:
            if getattr(obj, "name", None):
                F.Name = str(obj.name)
            else:
                F.Name = f"Fellow_{fellow_idx}"
            print(f"    Created Fellow with name: {F.Name}")
        except Exception as e:
            F.Name = f"Fellow_{fellow_idx}"
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
    
    def _distance_to_road_region(self, x, y, road_ids):
        """Calculate minimum distance from point (x,y) to any road in road_ids.
        
        Args:
            x, y: Point coordinates
            road_ids: List of road IDs to check against
            
        Returns:
            Minimum distance in meters (or float('inf') if no roads found)
        """
        if not road_ids or not self._road_index:
            return float('inf')
        
        # Get roads from the workspace network (not _road_index which loses IDs)
        workspace = getattr(self.scene, 'workspace', None)
        if not workspace or not hasattr(workspace, 'network'):
            return float('inf')
        
        network = workspace.network
        min_dist = float('inf')
        
        # Check each road in the network
        for road in network.roads:
            road_id = str(getattr(road, 'id', None))
            if road_id in road_ids:
                # Calculate distance to this road's centerline
                road_dist = self._distance_to_road_centerline(x, y, road)
                min_dist = min(min_dist, road_dist)
        
        return min_dist
    
    def _distance_to_road_centerline(self, x, y, road):
        """Calculate minimum distance from point to a road's centerline.
        
        Args:
            x, y: Point coordinates  
            road: Road object with lanes
            
        Returns:
            Minimum distance to road centerline in meters
        """
        if not road.lanes:
            return float('inf')
        
        min_dist = float('inf')
        
        # Check distance to each lane's centerline
        for lane in road.lanes:
            if hasattr(lane, 'centerline'):
                centerline = lane.centerline
                if hasattr(centerline, 'distanceTo'):
                    # Use Scenic's built-in distance method
                    try:
                        from scenic.core.geometry import Point
                        point = Point(x, y)
                        dist = centerline.distanceTo(point)
                        min_dist = min(min_dist, dist)
                    except:
                        # Fallback: approximate distance using line segments
                        dist = self._distance_to_polyline(x, y, centerline.points)
                        min_dist = min(min_dist, dist)
        
        return min_dist
    
    def _distance_to_polyline(self, x, y, points):
        """Calculate minimum distance from point to polyline.
        
        Args:
            x, y: Point coordinates
            points: List of (x, y) points defining the polyline
            
        Returns:
            Minimum distance to polyline in meters
        """
        if len(points) < 2:
            return float('inf')
        
        min_dist = float('inf')
        
        # Check distance to each line segment
        for i in range(len(points) - 1):
            x0, y0 = points[i][:2]  # Take first 2 coordinates
            x1, y1 = points[i+1][:2]
            
            # Distance from point to line segment
            dist = self._point_to_line_segment_distance(x, y, x0, y0, x1, y1)
            min_dist = min(min_dist, dist)
        
        return min_dist
    
    def _point_to_line_segment_distance(self, px, py, x0, y0, x1, y1):
        """Calculate distance from point to line segment.
        
        Args:
            px, py: Point coordinates
            x0, y0, x1, y1: Line segment endpoints
            
        Returns:
            Distance from point to line segment
        """
        # Vector from line start to end
        dx = x1 - x0
        dy = y1 - y0
        
        # Vector from line start to point
        wx = px - x0
        wy = py - y0
        
        # Project point onto line
        seg_len_sq = dx*dx + dy*dy
        if seg_len_sq < 1e-12:
            # Degenerate line segment
            return ((px - x0)**2 + (py - y0)**2)**0.5
        
        t = (wx*dx + wy*dy) / seg_len_sq
        t = max(0.0, min(1.0, t))  # Clamp to segment
        
        # Closest point on line segment
        closest_x = x0 + t*dx
        closest_y = y0 + t*dy
        
        # Distance to closest point
        return ((px - closest_x)**2 + (py - closest_y)**2)**0.5
    
    def detectTrackSegment(self, position):
        """Detect which track segment a position belongs to (racing-specific).
        
        Uses distance calculation to determine if position is closer to
        pit lane or main racing circuit.
        
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
            
            # Calculate distance to pit lane region
            pit_dist = self._distance_to_road_region(obj_x, obj_y, pit_lane_ids)
            
            # Calculate distance to main racing region  
            main_dist = self._distance_to_road_region(obj_x, obj_y, main_racing_ids)
            
            # Choose the closer region
            if pit_dist < main_dist:
                return 'pitLane'
            else:
                return 'mainRacing'
                
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
    
    def _set_fellow_route_via_segment(self, segs, obj):
        """Set the ModelDesk route for a fellow vehicle via segment Activity.RouteSelection.
        
        Args:
            segs: The fellow's segments collection
            obj: The Scenic object (vehicle)
        """
        try:
            # Get the start segment (seg0)
            if hasattr(segs, 'Count') and segs.Count > 0:
                seg0 = segs.Item(0)
            else:
                print(f"    [INFO] No segments available for route assignment")
                return
            
            # Try to access RouteSelection through Activity
            route_sel = None
            if hasattr(seg0, 'Activity') and hasattr(seg0.Activity, 'RouteSelection'):
                route_sel = seg0.Activity.RouteSelection
                print(f"    [Route] Found RouteSelection via seg0.Activity")
            elif hasattr(seg0, 'RouteSelection'):
                route_sel = seg0.RouteSelection  
                print(f"    [Route] Found RouteSelection on seg0 directly")
            else:
                print(f"    [INFO] No RouteSelection found on segment")
                return
            
            # Determine which route to use
            route_index = 1  # Default to Lap
            route_name_to_find = "Lap"
            
            # Check if object specifies a route
            if hasattr(obj, 'dspace_route'):
                if obj.dspace_route.lower() == "pit":
                    route_index = 0
                    route_name_to_find = "Pit"
                print(f"    [Route] Object specifies route: {obj.dspace_route}")
            
            # Try to set the route
            if route_sel and hasattr(route_sel, 'AvailableElements'):
                available = route_sel.AvailableElements
                if hasattr(available, 'Count'):
                    print(f"    [Route] Available routes: {available.Count}")
                    
                    # List all available routes for debugging
                    for i in range(available.Count):
                        try:
                            route = available.Item(i)
                            name = route.Name if hasattr(route, 'Name') else f"Route{i}"
                            print(f"      Route {i}: {name}")
                        except:
                            pass
                    
                    # Try to activate the desired route
                    if available.Count > route_index:
                        try:
                            route_item = available.Item(route_index)
                            actual_name = route_item.Name if hasattr(route_item, 'Name') else f"Route{route_index}"
                            
                            if hasattr(route_sel, 'Activate'):
                                route_sel.Activate(actual_name)
                                print(f"    [OK] Activated route: {actual_name} (index {route_index})")
                            elif hasattr(route_sel, 'ActiveElement'):
                                route_sel.ActiveElement = route_item
                                print(f"    [OK] Set ActiveElement to: {actual_name}")
                        except Exception as e:
                            print(f"    [WARN] Could not activate route {route_index}: {e}")
            
        except Exception as e:
            print(f"    [WARN] Could not set route via segment: {e}")
    
    def _set_fellow_route(self, sequence, obj):
        """Set the ModelDesk route for a fellow vehicle based on which Scenic road it was placed on.
        
        ModelDesk typically has predefined routes:
        - R1: Pit (route index 0) - for pit lane
        - R2: Lap (route index 1) - for main racing line
        
        Args:
            sequence: The fellow's sequence object
            obj: The Scenic object (vehicle)
        """
        try:
            # Check if the sequence has RouteSelection
            if not hasattr(sequence, 'RouteSelection'):
                print(f"    [INFO] Sequence has no RouteSelection property, skipping route assignment")
                return
            
            route_sel = sequence.RouteSelection
            
            # Determine which route to use based on the object's placement
            route_name = "Lap"  # Default to R2: Lap (main racing line)
            route_index = 1
            
            # Check if object has dspace_route attribute (can be set in scenario)
            if hasattr(obj, 'dspace_route'):
                route_name = obj.dspace_route
                if route_name.lower() == "pit":
                    route_index = 0
                print(f"    [Route] Using explicit route from object: {route_name}")
            else:
                # Try to infer from object's region
                region_check = []
                
                # Check various possible sources of region information
                if hasattr(obj, '_parentRegion'):
                    region_check.append(str(obj._parentRegion))
                if hasattr(obj, 'position') and hasattr(obj.position, '_region'):
                    region_check.append(str(obj.position._region))
                    
                # Look for pit lane indicators
                region_str = ' '.join(region_check).lower()
                if 'pit' in region_str:
                    route_name = "Pit"
                    route_index = 0
                    print(f"    [Route] Detected pit lane from region info, setting to R1: Pit")
                else:
                    print(f"    [Route] Defaulting to R2: Lap (main racing line)")
            
            # Set the route in ModelDesk
            if hasattr(route_sel, 'Activate') and hasattr(route_sel, 'AvailableElements'):
                available = route_sel.AvailableElements
                if hasattr(available, 'Count') and available.Count > 0:
                    print(f"    [Route] Available routes: {available.Count}")
                    
                    # Try to activate by index
                    if available.Count > route_index:
                        try:
                            route_item = available.Item(route_index)
                            actual_name = route_item.Name if hasattr(route_item, 'Name') else f"Route{route_index}"
                            route_sel.Activate(actual_name)
                            print(f"    [OK] Activated route: {actual_name} (index {route_index})")
                        except Exception as e:
                            print(f"    [WARN] Could not activate route by index {route_index}: {e}")
                    else:
                        print(f"    [WARN] Route index {route_index} out of range (only {available.Count} routes available)")
            else:
                print(f"    [INFO] RouteSelection does not support Activate or AvailableElements")
            
        except Exception as e:
            print(f"    [WARN] Could not set route: {e}")

    def _apply_relative_positioning(self):
        """Apply relative positioning logic based on Scenic's resolved coordinates.
        
        This method analyzes the actual coordinates that Scenic has computed for each
        car and detects when cars appear to be positioned relative to each other
        (lateral vs longitudinal). It then adjusts the dSPACE Fellow positions to
        maintain proper track distances.
        
        This works with any Scenic positioning syntax since it only uses the
        final resolved coordinates that Scenic provides.
        """
        if not hasattr(self, '_object_positions') or len(self._object_positions) < 2:
            return
        # exit()
            
        print(f"\n=== Applying Relative Positioning Logic ===")
        print(f"Found {len(self._object_positions)} objects to analyze")
        
        # Group cars that are likely meant to be relative to each other
        car_groups = self._group_relative_cars()
        
        for group_idx, group in enumerate(car_groups):
            if len(group) < 2:
                continue
                
            print(f"\nProcessing group {group_idx + 1} with {len(group)} cars:")
            
            # Sort by s-coordinate to establish order
            group.sort(key=lambda x: x['s_coord'])
            
            # Calculate track distances and adjust s-coordinates
            self._adjust_group_positions(group)
    
    def _group_relative_cars(self):
        """Group cars that are likely meant to be relative to each other.
        
        Cars are grouped if they are:
        1. Close to each other in world coordinates (< 100m)
        2. Have similar headings (within 45 degrees)
        3. Are on the same road segment
        """
        groups = []
        processed = set()
        
        for i, obj1 in enumerate(self._object_positions):
            if i in processed:
                continue
                
            # Start a new group with this car
            group = [obj1]
            processed.add(i)
            
            # Find other cars that should be in the same group
            for j, obj2 in enumerate(self._object_positions[i+1:], i+1):
                if j in processed:
                    continue
                    
                if self._should_be_relative(obj1, obj2):
                    group.append(obj2)
                    processed.add(j)
            
            groups.append(group)
            
        return groups
    
    def _should_be_relative(self, obj1, obj2):
        """Determine if two cars should be considered relative to each other."""
        import math
        
        # Calculate world distance
        pos1 = obj1['position']
        pos2 = obj2['position']
        world_dist = ((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2)**0.5
        
        # Calculate s-coordinate distance
        s_dist = abs(obj1['s_coord'] - obj2['s_coord'])
        
        # Calculate t-coordinate distance (lateral)
        t_dist = abs(obj1['t_coord'] - obj2['t_coord'])
        
        # Calculate heading difference
        h1 = obj1['heading']
        h2 = obj2['heading']
        heading_diff = abs(h1 - h2)
        # Normalize to [0, π]
        if heading_diff > math.pi:
            heading_diff = 2*math.pi - heading_diff
        
        # Check if cars are too close (essentially identical positions)
        threshold = self.config['duplicate_position_threshold']
        if world_dist < threshold and s_dist < threshold and t_dist < threshold:
            print(f"  Cars are too close to be relative (identical positions): world_dist={world_dist:.3f}m")
            return False
        
        # Check if this looks like a lateral positioning pattern
        is_lateral_pattern = self._is_lateral_positioning_pattern(obj1, obj2)
        
        print(f"  Comparing cars: world_dist={world_dist:.1f}m, s_dist={s_dist:.1f}m, t_dist={t_dist:.1f}m, heading_diff={heading_diff:.2f}rad, lateral_pattern={is_lateral_pattern}")
        
        # Criteria for relative positioning:
        # 1. Close in world coordinates
        # 2. Close in s-coordinates OR lateral pattern detected
        # 3. Similar headings
        should_be_relative = (world_dist < self.config['relative_distance_threshold'] and 
                             (s_dist < self.config['s_coordinate_threshold'] or is_lateral_pattern) and 
                             heading_diff < math.radians(self.config['heading_difference_threshold']))
        
        print(f"  Should be relative: {should_be_relative}")
        return should_be_relative
    
    def _is_lateral_positioning_pattern(self, obj1, obj2):
        """Detect if two cars are positioned laterally based on their generated coordinates.
        
        This analyzes the actual coordinates that Scenic has computed and detects when
        cars appear to be positioned side-by-side (lateral positioning) rather than
        one behind the other (longitudinal positioning).
        
        The detection is purely based on the resolved coordinates, not on parsing
        Scenic syntax. This allows it to work with any Scenic positioning method.
        
        The key indicators are:
        1. Very similar s-coordinates (cars are at the same track position)
        2. Significant t-coordinate difference (cars are side by side)
        3. Similar headings (cars are aligned)
        4. Close world distance (cars are nearby)
        """
        import math
        
        # Calculate distances
        pos1 = obj1['position']
        pos2 = obj2['position']
        world_dist = ((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2)**0.5
        s_dist = abs(obj1['s_coord'] - obj2['s_coord'])
        
        # Calculate heading difference
        h1 = obj1['heading']
        h2 = obj2['heading']
        heading_diff = abs(h1 - h2)
        if heading_diff > math.pi:
            heading_diff = 2*math.pi - heading_diff
        
        # Get reference car's heading to calculate lateral component
        ref_heading = obj1['heading']
        
        # Calculate the vector between cars
        dx = pos2.x - pos1.x
        dy = pos2.y - pos1.y
        
        # Calculate lateral component (perpendicular to heading)
        left_normal_x = -math.sin(ref_heading)
        left_normal_y = math.cos(ref_heading)
        lateral_component = dx * left_normal_x + dy * left_normal_y
        
        # Calculate longitudinal component (along heading)
        longitudinal_component = dx * math.cos(ref_heading) + dy * math.sin(ref_heading)
        
        # Lateral positioning pattern indicators:
        # 1. Very similar s-coordinates (< 1m) - cars are at the same track position
        # 2. Significant t-coordinate difference (> 1m) - cars are side by side
        # 3. Similar headings (< 15 degrees) - cars are aligned
        # 4. Reasonable world distance (< 20m) - cars are close together
        
        t_dist = abs(obj1['t_coord'] - obj2['t_coord'])
        
        is_lateral = (
            s_dist < self.config['lateral_pattern_s_threshold'] and                       # Very similar s-coordinates
            t_dist > self.config['lateral_pattern_t_threshold'] and                       # Significant t-coordinate difference
            heading_diff < math.radians(self.config['lateral_pattern_heading_threshold']) and          # Similar headings
            world_dist < self.config['lateral_pattern_world_threshold']                      # Reasonable distance
        )
        
        if is_lateral:
            # Apply calibration factor for t-coordinate interpretation
            calibration_factor = self.config['lateral_calibration_factor']
            corrected_t_dist = t_dist * calibration_factor
            direction = "left" if corrected_t_dist > 0 else "right"
            print(f"    Detected lateral pattern: {direction} by {abs(corrected_t_dist):.1f}m (t-coord diff), s_diff={s_dist:.1f}m")
        
        return is_lateral
    
    def _adjust_group_positions(self, group):
        """Adjust positions using ModelDesk relative positioning when possible."""
        if len(group) < 2:
            return
            
        print(f"  Adjusting positions for group of {len(group)} cars using ModelDesk relative positioning")
        
        # Sort the group by s-coordinate to establish a clear order
        group.sort(key=lambda x: x['s_coord'])
        
        # Use the first car (lowest s-coordinate) as the reference point
        reference = group[0]
        ref_s = reference['s_coord']
        ref_t = reference['t_coord']
        
        print(f"  Reference car at s={ref_s:.3f}, t={ref_t:.3f}")
        
        # Set up reference car with absolute positioning (already done in createObjectInSimulator)
        
        # For each subsequent car, set up relative positioning
        for i, car in enumerate(group[1:], 1):
            # Check if this is a lateral positioning pattern
            is_lateral = self._is_lateral_positioning_pattern(reference, car)
            
            if is_lateral:
                # Calculate lateral distance and use absolute positioning for reliability
                lateral_dist = self._calculate_intended_lateral_distance(reference, car)
                print(f"  Car {i+1}: Lateral positioning detected, using absolute positioning (deviation: {lateral_dist:.1f}m)")
                
                # Use absolute positioning for lateral offset
                new_s = reference['s_coord']
                new_t = reference['t_coord'] + lateral_dist
                print(f"  Car {i+1}: Setting absolute position: s={new_s:.3f}, t={new_t:.3f}")
                self._update_fellow_position(car, new_s, new_t)
            else:
                # Calculate longitudinal distance and use absolute positioning for reliability
                longitudinal_dist = self._calculate_intended_longitudinal_distance(reference, car)
                print(f"  Car {i+1}: Longitudinal positioning detected, using absolute positioning (distance: {longitudinal_dist:.1f}m)")
                
                # Use absolute positioning for longitudinal offset
                new_s = reference['s_coord'] + longitudinal_dist
                new_t = car['t_coord']  # Keep original t-coordinate
                print(f"  Car {i+1}: Setting absolute position: s={new_s:.3f}, t={new_t:.3f}")
                self._update_fellow_position(car, new_s, new_t)
    
    def _calculate_intended_longitudinal_distance(self, ref_car, target_car):
        """Calculate the intended longitudinal (s-coordinate) distance between two cars."""
        import math
        
        # Calculate vector from reference to target
        ref_pos = ref_car['position']
        target_pos = target_car['position']
        
        dx = target_pos.x - ref_pos.x
        dy = target_pos.y - ref_pos.y
        
        # Calculate the world distance
        world_dist = (dx*dx + dy*dy)**0.5
        
        # Get the reference car's heading direction
        ref_heading = ref_car['heading']
        
        # Project the vector onto the reference car's heading direction
        # This gives us the longitudinal component
        longitudinal_component = dx * math.cos(ref_heading) + dy * math.sin(ref_heading)
        
        # If the projection is very small (cars are perpendicular), use 0 for longitudinal
        if abs(longitudinal_component) < 0.1:
            intended_dist = 0.0
            print(f"    Using 0 for longitudinal (perpendicular): {intended_dist:.1f}m")
        else:
            intended_dist = abs(longitudinal_component)
            print(f"    Using projected longitudinal distance: {intended_dist:.1f}m")
        
        # Determine the sign based on the component
        if longitudinal_component < 0:
            intended_dist = -intended_dist
            
        print(f"    Longitudinal calculation: dx={dx:.1f}, dy={dy:.1f}, component={longitudinal_component:.1f}, intended_dist={intended_dist:.1f}")
        
        return intended_dist
    
    def _calculate_intended_lateral_distance(self, ref_car, target_car):
        """Calculate the intended lateral (t-coordinate) distance between two cars."""
        import math
        
        # For lateral positioning, use the t-coordinate difference directly
        # This is more reliable than trying to project world coordinates
        t_diff = target_car['t_coord'] - ref_car['t_coord']
        
        print(f"    Lateral calculation: t_diff={t_diff:.3f}")
        
        # If this is a lateral positioning pattern, use the t-coordinate difference
        if self._is_lateral_positioning_pattern(ref_car, target_car):
            # Apply calibration factor for t-coordinate interpretation
            # This accounts for differences between OpenDRIVE coordinate system and expected positioning
            
            # Use configurable calibration factor for t-coordinate interpretation
            calibration_factor = self.config['lateral_calibration_factor']
            corrected_t_diff = t_diff * calibration_factor
            
            print(f"    Using t-coordinate difference for lateral positioning: {t_diff:.1f}m (raw)")
            print(f"    Corrected for track orientation: {corrected_t_diff:.1f}m (calibration factor: {calibration_factor})")
            return corrected_t_diff
        else:
            # For non-lateral patterns, calculate from world coordinates
            ref_pos = ref_car['position']
            target_pos = target_car['position']
            
            dx = target_pos.x - ref_pos.x
            dy = target_pos.y - ref_pos.y
            
            # Get the reference car's heading direction
            ref_heading = ref_car['heading']
            
            # Calculate the left normal vector (perpendicular to heading, pointing left)
            left_normal_x = -math.sin(ref_heading)
            left_normal_y = math.cos(ref_heading)
            
            # Project the vector onto the left normal direction
            lateral_component = dx * left_normal_x + dy * left_normal_y
            
            print(f"    Using projected lateral component: {lateral_component:.1f}m")
            return lateral_component
    
    def _update_fellow_position(self, car_info, new_s, new_t=None):
        """Update the dSPACE Fellow object's position with the new s-coordinate and optionally t-coordinate."""
        try:
            # Find the Fellow object that corresponds to this car
            # We need to match by the order they were created
            fellow_idx = self._object_positions.index(car_info)
            fellow = self.ts.Fellows.Item(fellow_idx)
            
            # Update the first segment's position
            seqs = fellow.Sequences
            if seqs.Count > 0:
                seq = seqs.Item(0)
                segs = seq.Segments
                if segs.Count > 0:
                    seg0 = segs.Item(0)
                    
                    # Update longitudinal position (s-coordinate)
                    lt0 = seg0.Activity.LongitudinalType
                    if hasattr(lt0, 'ActiveElement') and hasattr(lt0.ActiveElement, 'SourceType'):
                        src_type = lt0.ActiveElement.SourceType
                        if hasattr(src_type, 'ActiveElement') and hasattr(src_type.ActiveElement, 'Constant'):
                            src_type.ActiveElement.Constant = float(new_s)
                            print(f"    Updated Fellow longitudinal position to s={new_s:.3f}")
                    
                    # Update lateral position (t-coordinate) if provided
                    if new_t is not None:
                        lat0 = seg0.Activity.LateralType
                        if hasattr(lat0, 'ActiveElement') and hasattr(lat0.ActiveElement, 'SourceType'):
                            src_type = lat0.ActiveElement.SourceType
                            if hasattr(src_type, 'ActiveElement') and hasattr(src_type.ActiveElement, 'Constant'):
                                old_t = src_type.ActiveElement.Constant
                                src_type.ActiveElement.Constant = float(new_t)
                                print(f"    Updated Fellow lateral position from t={old_t:.3f} to t={new_t:.3f}")
                                print(f"    DEBUG: Aurelion t-coordinate change: {new_t - old_t:.3f}m")
                            
        except Exception as e:
            print(f"    Warning: Could not update Fellow position: {e}")

    def _setup_reference_car(self, car_info):
        """Set up the reference car with absolute positioning (already done in createObjectInSimulator)."""
        # The reference car is already set up with absolute positioning in createObjectInSimulator
        print(f"    Reference car already configured with absolute positioning")
        pass

    def _setup_lateral_relative_car(self, car_info, reference_car, deviation_m):
        """Set up a car with lateral relative positioning using ModelDesk's LaneSelection with Relative dependency."""
        try:
            # Find the Fellow object that corresponds to this car
            fellow_idx = self._object_positions.index(car_info)
            if fellow_idx >= self.ts.Fellows.Count:
                print(f"    Error: Fellow index {fellow_idx} is out of range (max: {self.ts.Fellows.Count - 1})")
                return
            fellow = self.ts.Fellows.Item(fellow_idx)
            
            # Find the reference Fellow object
            ref_idx = self._object_positions.index(reference_car)
            if ref_idx >= self.ts.Fellows.Count:
                print(f"    Error: Reference Fellow index {ref_idx} is out of range (max: {self.ts.Fellows.Count - 1})")
                return
            ref_fellow = self.ts.Fellows.Item(ref_idx)
            
            print(f"    Setting up lateral relative positioning: deviation={deviation_m:.1f}m")
            
            # Get the sequence and segments
            seqs = fellow.Sequences
            if seqs.Count == 0:
                print(f"    Error: Fellow has no sequences")
                return
                
            seq = seqs.Item(0)
            segs = seq.Segments
            
            # Ensure we have at least 2 segments
            while segs.Count < 2:
                try:
                    segs.Add()
                except Exception as e:
                    print(f"    Error: Could not add segment: {e}")
                    return
            
            # Set up segment 1 for lateral relative positioning
            if segs.Count < 2:
                print(f"    Error: Not enough segments (have {segs.Count}, need 2)")
                return
            seg1 = segs.Item(1)
            
            # Configure lateral type to LaneSelection
            lat1 = seg1.Activity.LateralType
            if not hasattr(lat1, 'Activate'):
                print(f"    Error: LateralType has no Activate method")
                return
                
            try:
                lat1.Activate("LaneSelection")
                print(f"    Activated LaneSelection")
            except Exception as e:
                print(f"    Error: Could not activate LaneSelection: {e}")
                return
            
            # Set up relative dependency
            try:
                active_elem = lat1.ActiveElement
                if not hasattr(active_elem, 'DependencyType'):
                    print(f"    Error: ActiveElement has no DependencyType")
                    return
                    
                dep_type = active_elem.DependencyType
                if not hasattr(dep_type, 'Activate'):
                    print(f"    Error: DependencyType has no Activate method")
                    return
                    
                dep_type.Activate("Relative")
                print(f"    Activated Relative dependency")
                
                # Set relative vehicle to reference car
                if not hasattr(dep_type, 'ActiveElement'):
                    print(f"    Error: DependencyType has no ActiveElement")
                    return
                    
                if not hasattr(dep_type.ActiveElement, 'RelativeVehicle'):
                    print(f"    Error: DependencyType.ActiveElement has no RelativeVehicle")
                    return
                    
                rel_vehicle = dep_type.ActiveElement.RelativeVehicle
                if not hasattr(rel_vehicle, 'Activate'):
                    print(f"    Error: RelativeVehicle has no Activate method")
                    return
                
                # Use the name we set during Fellow creation instead of trying to read it back
                try:
                    # Use the name we know we set during creation
                    ref_name = f'Fellow_{ref_idx}'
                    print(f"    Using reference car name: {ref_name}")
                    rel_vehicle.Activate(ref_name)
                    print(f"    Successfully set relative vehicle to: {ref_name}")
                except Exception as e:
                    print(f"    Warning: Could not set relative vehicle reference: {e}")
                    print(f"    ModelDesk relative positioning not supported, falling back to absolute positioning")
                    return
                        
            except Exception as e:
                print(f"    Warning: Could not set up relative dependency: {e}")
                return
            
            # Set the deviation constant
            try:
                if not hasattr(active_elem, 'SourceType'):
                    print(f"    Error: ActiveElement has no SourceType")
                    return
                    
                src_type = active_elem.SourceType
                if not hasattr(src_type, 'Activate'):
                    print(f"    Error: SourceType has no Activate method")
                    return
                    
                src_type.Activate("Constant")
                print(f"    Activated Constant source type")
                
                if not hasattr(src_type, 'ActiveElement'):
                    print(f"    Error: SourceType has no ActiveElement")
                    return
                    
                if not hasattr(src_type.ActiveElement, 'Constant'):
                    print(f"    Error: SourceType.ActiveElement has no Constant property")
                    return
                    
                src_type.ActiveElement.Constant = float(deviation_m)
                print(f"    Set lateral deviation constant to {deviation_m:.1f}m")
                
            except Exception as e:
                print(f"    Warning: Could not set lateral deviation constant: {e}")
                return
            
            # Set longitudinal to continue (or velocity)
            try:
                lt1 = seg1.Activity.LongitudinalType
                if hasattr(lt1, 'Activate'):
                    lt1.Activate("Continue")
                    print(f"    Set longitudinal to Continue")
            except Exception as e:
                print(f"    Warning: Could not set longitudinal to Continue: {e}")
            
            # Set up endless transition
            self._make_endless_transition(seg1)
            
            print(f"    Successfully configured lateral relative positioning")
                
        except Exception as e:
            print(f"    Warning: Could not set up lateral relative positioning: {e}")

    def _setup_longitudinal_relative_car(self, car_info, reference_car, distance_m):
        """Set up a car with longitudinal relative positioning using ModelDesk's DistanceMeter."""
        try:
            # Find the Fellow object that corresponds to this car
            fellow_idx = self._object_positions.index(car_info)
            fellow = self.ts.Fellows.Item(fellow_idx)
            
            # Find the reference Fellow object
            ref_idx = self._object_positions.index(reference_car)
            ref_fellow = self.ts.Fellows.Item(ref_idx)
            
            print(f"    Setting up longitudinal relative positioning: distance={distance_m:.1f}m")
            
            # Get the sequence and segments
            seqs = fellow.Sequences
            if seqs.Count > 0:
                seq = seqs.Item(0)
                segs = seq.Segments
                
                # Ensure we have at least 2 segments
                while segs.Count < 2:
                    segs.Add()
                
                # Set up segment 1 for longitudinal relative positioning
                seg1 = segs.Item(1)
                
                # Configure longitudinal type to DistanceMeter
                lt1 = seg1.Activity.LongitudinalType
                if hasattr(lt1, 'Activate'):
                    lt1.Activate("DistanceMeter")
                
                # Set the distance constant
                try:
                    active_elem = lt1.ActiveElement
                    if hasattr(active_elem, 'SourceType'):
                        src_type = active_elem.SourceType
                        if hasattr(src_type, 'Activate'):
                            src_type.Activate("Constant")
                        
                        if hasattr(src_type, 'ActiveElement') and hasattr(src_type.ActiveElement, 'Constant'):
                            src_type.ActiveElement.Constant = float(distance_m)
                            print(f"    Set longitudinal distance constant to {distance_m:.1f}m")
                except Exception as e:
                    print(f"    Warning: Could not set longitudinal distance constant: {e}")
                
                # Set lateral to continue
                lat1 = seg1.Activity.LateralType
                if hasattr(lat1, 'Activate'):
                    lat1.Activate("Continue")
                
                # Set up endless transition
                self._make_endless_transition(seg1)
                
                print(f"    Successfully configured longitudinal relative positioning")
                
        except Exception as e:
            print(f"    Warning: Could not set up longitudinal relative positioning: {e}")

    def _make_endless_transition(self, segment):
        """Set up endless transition for a segment."""
        try:
            tr = segment.Transition
            if hasattr(tr, 'Conditions'):
                conds = tr.Conditions
                # Clear existing conditions
                while conds.Count > 0:
                    try:
                        conds.Remove(0)
                    except:
                        break
                # Add Endless condition
                if hasattr(conds, 'Add'):
                    conds.Add("Endless")
        except Exception as e:
            print(f"    Warning: Could not set up endless transition: {e}")

    def _verify_relative_positioning_setup(self, car_info):
        """Verify that relative positioning was set up correctly. Returns True if successful, False otherwise."""
        try:
            # Find the Fellow object that corresponds to this car
            fellow_idx = self._object_positions.index(car_info)
            if fellow_idx >= self.ts.Fellows.Count:
                return False
            fellow = self.ts.Fellows.Item(fellow_idx)
            
            # Check if the Fellow has the expected configuration
            seqs = fellow.Sequences
            if seqs.Count == 0:
                return False
                
            seq = seqs.Item(0)
            segs = seq.Segments
            if segs.Count < 2:
                return False
                
            seg1 = segs.Item(1)
            
            # Check if lateral type is set to LaneSelection
            lat1 = seg1.Activity.LateralType
            try:
                # Try to access the ActiveElement to see if it's configured
                active_elem = lat1.ActiveElement
                if hasattr(active_elem, 'DependencyType'):
                    dep_type = active_elem.DependencyType
                    if hasattr(dep_type, 'ActiveElement'):
                        return True  # Basic structure is there
            except:
                pass
                
            return False
        except Exception as e:
            print(f"    Warning: Could not verify relative positioning setup: {e}")
            return False

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
