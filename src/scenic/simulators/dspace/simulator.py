# -*- coding: utf-8 -*-
# Scenic → dSPACE (ModelDesk) absolute placement:
# - SaveAs/Activate first (desired)
# - Build XODR reference index from Scenic param `map`
# - For each Scenic object: (x,y) → (s,t), then seg0 uses absolute Position/Deviation

import time
import pythoncom
from win32com.client import Dispatch

from scenic.core.vectors import Vector
from scenic.domains.driving.simulators import DrivingSimulator, DrivingSimulation
from scenic.core.simulators import SimulationCreationError

from . import utils as dutils
import pandas as pd
import math
import os
import datetime


class DSpaceSimulator(DrivingSimulator):
    def __init__(self, *, scenario_src="LagunaSeca_ExternalControl",
                 scenario_name=None, timestep=0.1, save_as=True):
        super().__init__()
        self.scenario_src = scenario_src
        self.scenario_name = scenario_name
        self.timestep = float(timestep)
        self.save_as = bool(save_as)
        

    def createSimulation(self, scene, **kwargs):
        return DSpaceSimulation(scene, self, **kwargs)


class DSpaceSimulation(DrivingSimulation):
    def __init__(self, scene, sim: DSpaceSimulator, **kwargs):
        self.sim = sim
        self.exp = None
        self.ts  = None
        self._road_index = None   # parsed from XODR or RD
        self._coordinate_transform = None  # XODR→RD transformation if needed
        
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
        """Place every car by absolute (s,t) computed from (x,y) and XODR.
        
        This function automatically transforms Scenic world coordinates (x,y,z) 
        to the corresponding (s,t) coordinates that will map correctly to Aurelion's
        coordinate system through the dSPACE simulator.
        
        Additionally, it analyzes the resolved coordinates from Scenic to detect
        relative positioning patterns (lateral vs longitudinal) and adjusts
        positions to maintain proper track distances in the dSPACE simulator.
        
        Note: This works with any Scenic positioning syntax since it only uses
        the final resolved coordinates that Scenic provides.
        """
        print(f"Creating object in dSPACE with position and heading: {obj.position}, heading: {obj.heading}")

        # 1) Project Scenic (x,y) → (s,t). If no map, use zeros.
        if getattr(obj, "position", None) is not None:
            scenic_x, scenic_y = obj.position.x, obj.position.y
            
            # Apply coordinate transformation if available (XODR→RD correction)
            if self._coordinate_transform is not None:
                from . import coordinate_transform
                transformed_x, transformed_y = coordinate_transform.apply_coordinate_transform(
                    self._coordinate_transform, (scenic_x, scenic_y)
                )
                print(f"Scenic coords ({scenic_x:.3f}, {scenic_y:.3f}) → "
                      f"RD coords ({transformed_x:.3f}, {transformed_y:.3f})")
                work_x, work_y = transformed_x, transformed_y
            else:
                work_x, work_y = scenic_x, scenic_y
            
            # Use road index for proper geometric projection
            if self._road_index:
                s_val, t_val = dutils.project_world_to_st(self._road_index, (work_x, work_y))
                print(f"World coordinates ({work_x:.3f}, {work_y:.3f}) → Road coordinates (s={s_val:.1f}, t={t_val:.3f})")
            else:
                s_val, t_val = dutils.map_scenic_to_modeldesk(work_x, work_y)
                print(f"World coordinates ({work_x:.3f}, {work_y:.3f}) → Fallback coordinates (s={s_val:.1f}, t={t_val:.3f})")
        else:
            s_val, t_val = 0.0, 0.0
            print("Warning: No position available, using default coordinates (s=0, t=0)")

        # 2) Store the object's position for relative positioning analysis
        if not hasattr(self, '_object_positions'):
            self._object_positions = []
        self._object_positions.append({
            'obj': obj,
            'position': obj.position,
            's_coord': s_val,
            't_coord': t_val,
            'heading': obj.heading
        })

        # 3) Create Fellow with one Sequence and two Segments
        F = self.ts.Fellows.Add()

        # Set the scenario name for csv file
        csv_filename = "scenic_runs_fellow2_left_by_2.csv"

        # Set a unique name for relative positioning
        fellow_idx = len(self._object_positions) if hasattr(self, '_object_positions') else 0
        try:
            if getattr(obj, "name", None):
                F.Name = str(obj.name)
            else:
                F.Name = f"Fellow_{fellow_idx}"
            print(f"    Created Fellow with name: {F.Name}")
            fellow1_df = pd.DataFrame()
            fellow2_df = pd.DataFrame()
            analysis_df = pd.DataFrame()
            if fellow_idx == 1:
                # Store fellow1_df IN MEMORY, not in a file
                self._fellow1_df = self.fellow_coords_df(
                    obj, fellow_idx, scenic_x, scenic_y, transformed_x, transformed_y, s_val, t_val
                )

                # Set CSV filename once
                if not hasattr(self, '_csv_filename'):
                    scenario_name = "fellow2_left_by_2"
                    self._csv_filename = f"scenic_runs_{scenario_name}.csv"

            elif fellow_idx == 2:
                fellow2_df = self.fellow_coords_df(
                    obj, fellow_idx, scenic_x, scenic_y, transformed_x, transformed_y, s_val, t_val
                )

                fellow1_df = self._fellow1_df  # Fellow1 info stored in memory

                # Combine to compute analysis
                combined_df = pd.concat([fellow1_df, fellow2_df], axis=0, ignore_index=True)
                analysis_df = self.create_analysis(
                    combined_df['scenic_vector_x'],
                    combined_df['scenic_vector_y'],
                    combined_df['true_left_x'],
                    combined_df['true_left_y'],
                    combined_df['rd_world_x'],
                    combined_df['rd_world_y'],
                    combined_df['road_s'],
                    combined_df['road_t']
                )

                # Flatten all data into a single-row dict (ordered)
                row = {
                    # Fellow1 (columns 1-11)
                    'scene_name_f1':         fellow1_df.iloc[0]['scene_name'],
                    'car_name_f1':           fellow1_df.iloc[0]['car_name'],
                    'scenic_heading_f1':     fellow1_df.iloc[0]['scenic_heading'],
                    'true_left_x_f1':        fellow1_df.iloc[0]['true_left_x'],
                    'true_left_y_f1':        fellow1_df.iloc[0]['true_left_y'],
                    'scenic_vector_x_f1':    fellow1_df.iloc[0]['scenic_vector_x'],
                    'scenic_vector_y_f1':    fellow1_df.iloc[0]['scenic_vector_y'],
                    'rd_world_x_f1':         fellow1_df.iloc[0]['rd_world_x'],
                    'rd_world_y_f1':         fellow1_df.iloc[0]['rd_world_y'],
                    'road_s_f1':             fellow1_df.iloc[0]['road_s'],
                    'road_t_f1':             fellow1_df.iloc[0]['road_t'],

                    # Fellow2 (columns 12-22)
                    'scene_name_f2':       fellow2_df.iloc[0]['scene_name'],
                    'car_name_f2':         fellow2_df.iloc[0]['car_name'],
                    'scenic_vector_x_f2':  fellow2_df.iloc[0]['scenic_vector_x'],
                    'scenic_vector_y_f2':  fellow2_df.iloc[0]['scenic_vector_y'],
                    'rd_world_x_f2':       fellow2_df.iloc[0]['rd_world_x'],
                    'rd_world_y_f2':       fellow2_df.iloc[0]['rd_world_y'],
                    'road_s_2_f2':           fellow2_df.iloc[0]['road_s'],
                    'road_t_2_f2':           fellow2_df.iloc[0]['road_t'],

                    # Analysis (columns 23-30)
                    'scenic_x_diff':      analysis_df.iloc[0]['scenic_x_diff'],
                    'scenic_y_diff':      analysis_df.iloc[0]['scenic_y_diff'],
                    'scenic_diff_dot_true_left': analysis_df.iloc[0]['scenic_diff_dot_true_left'],
                    'scenic_magnitude': analysis_df.iloc[0]['scenic_magnitude'],
                    'scenic_diff_to_true_left_angle (radians)': analysis_df.iloc[0]['scenic_diff_to_true_left_angle (radians)'],
                    'rd_x_diff':          analysis_df.iloc[0]['rd_x_diff'],
                    'rd_y_diff':          analysis_df.iloc[0]['rd_y_diff'],
                    'rd_diff_dot_true_left':     analysis_df.iloc[0]['rd_diff_dot_true_left'],
                    's_diff':             analysis_df.iloc[0]['s_diff'],
                    't_diff':             analysis_df.iloc[0]['t_diff']
                }

                # Convert to DataFrame
                row_df = pd.DataFrame([row])
                write_header = not os.path.exists(self._csv_filename)
                row_df.to_csv(self._csv_filename, mode='a', header=write_header, index=False)

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
        base_v = getattr(obj, "md_v", None)
        if base_v is None:
            base_v = getattr(obj, "speed", 0.0) or 0.0
        dutils.configure_seg1_motion(segs, v=float(base_v), t=float(t_val))
        dutils.make_endless_transition(segs)

        return F

    def fellow_coords_df(self, obj, fellow_idx, scenic_x, scenic_y, transformed_x, transformed_y, s_val, t_val):
        """Create a CSV file with car positions and orientations.
        
        Columns:
            scen_name: __dir__ of __dist__
            scenic_vector_x/y/z: Position vector in Scenic coordinates
            scenic_heading: Heading in radians
            left_vector_x/y: Unit vector in left direction
            scenic_x/y: Original Scenic coordinates
            rd_world_x/y: Transformed RD/World coordinates
            road_s/t: Road coordinates (s,t)
        """
        # Calculate left unit vector (90 degrees counter-clockwise from heading)
        heading = obj.heading if hasattr(obj, 'heading') else 0
        true_left_x = math.cos(heading + math.pi/2) 
        true_left_y = math.sin(heading + math.pi/2)   # sin(heading + π/2)

        coords = {
            'scene_name': ['Left of 2'],
            'car_name': [f'fellow{fellow_idx}'],
            'scenic_vector_x': [scenic_x],
            'scenic_vector_y': [scenic_y],
            'scenic_heading': [heading],
            'true_left_x': [true_left_x],
            'true_left_y': [true_left_y],
            'rd_world_x': [transformed_x],
            'rd_world_y': [transformed_y],
            'road_s': [s_val],
            'road_t': [t_val]
        }

        data_frame = pd.DataFrame(coords)
        return data_frame
    
    def create_analysis(self, scenic_x, scenic_y, true_left_x, true_left_y, rd_world_x, rd_world_y, road_s, road_t):
        # scenic coordinates difference
        scenic_x_diff = scenic_x[1] - scenic_x[0]
        scenic_y_diff = scenic_y[1] - scenic_y[0]

        scenic_diff_dot_true = scenic_x_diff * true_left_x[0] + scenic_y_diff * true_left_y[0]
        # should approximately be the distance defined in fellow_placing_road.scenic since ||true_left|| = 1, cos(theta) = 1
        # if dot > 0, then the fellow2 is to the left of fellow1
        # if dot = 0, then the fellow2 is directly ahead/behind fellow1
        # if dot < 0, then the fellow2 is to the right of fellow1

        scenic_diff_magnitude = (scenic_x_diff**2 + scenic_y_diff**2)**0.5
        # should approximately be the distance defined in fellow_placing_road.scenic without considering the direction
        
        scenic_diff_to_true_angle = math.acos(scenic_diff_dot_true / scenic_diff_magnitude) if scenic_diff_magnitude != 0 else 0
        # should be close to 0 if they are in parallel

        # road coordinates difference
        rd_x_diff = rd_world_x[1] - rd_world_x[0]
        rd_y_diff = rd_world_y[1] - rd_world_y[0]

        rd_diff_dot_true = rd_x_diff * true_left_x[0] + rd_y_diff * true_left_y[0]    
        # should approximately be the distance defined in fellow_placing_road.scenic

        # s, t difference
        s_diff = road_s[1] - road_s[0]  # should be 0
        t_diff = road_t[1] - road_t[0]  # should approximately be the distance defined in fellow_placing_road.scenic

        analysis_data = {
            'scenic_x_diff': [scenic_x_diff],
            'scenic_y_diff': [scenic_y_diff],
            'scenic_diff_dot_true_left': [scenic_diff_dot_true],
            'scenic_diff_magnitude': [scenic_diff_magnitude],
            'scenic_diff_to_true_left_angle (radians)': [scenic_diff_to_true_angle],
            'rd_x_diff': [rd_x_diff],
            'rd_y_diff': [rd_y_diff],
            'rd_diff_dot_true_left': [rd_diff_dot_true],
            's_diff': [s_diff],
            't_diff': [t_diff],
        }

        analysis_df = pd.DataFrame(analysis_data)
        return analysis_df


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
