import math
import pyxodr
import json
import numpy as np
from typing import Dict, List, Optional, Any
from pyxodr.road_objects.road import Road
from lxml import etree

# obtain OpenDrive File
#network = RoadNetwork("../../assets/maps/dSPACE/LagunaSeca.xodr")

def load_road_xodr(xodr_path: str):
    doc = etree.parse(xodr_path)
    root = doc.getroot()
    roads_xml = list(root.findall('road'))
    roads = [Road(road_xml) for road_xml in roads_xml]
    return roads

def extract_road_geometries(road_obj):
    road_geometries = []
    for geom in road_obj.planView.geometries:
        print(geom)
    # for geom_xml in road_obj.road_xml.findall('planView/geometry'):
    #     s_start = float(geom_xml.attrib.get('s'))
    #     length = float(geom_xml.attrib.get('length'))
    #     s_end = s_start + length
    
    #     ref_line = road_obj.reference_line # (x, y)
    #     deltas = np.diff(ref_line, axis=0)
    #     dists = np.linalg.norm(deltas, axis=1)
    #     cum_dists = np.insert(np.cumsum(dists), 0, 0.0)
        
    #     idx_start = int(np.argmin(np.abs(cum_dists - s_start)))
    #     idx_end = int(np.argmin(np.abs(cum_dists - s_end)))
    #     idx_start = max(0, min(idx_start, len(ref_line)-1))
    #     idx_end = max(0, min(idx_end, len(ref_line)-1))

    #     x0, y0 = ref_line[idx_start]
    #     x1, y1 = ref_line[idx_end]
    #     straight_dist = math.hypot(x1 - x0, y1 - y0)
    #     delta_s = length
    #     # simple lateral deviation approx: how much the path 'bends' compared to straight line
    #     delta_t = abs(delta_s - straight_dist)

    #     # curvature: try to read arc/spiral/poly3 attributes
    #     curvature = None
    #     if geom_xml.find('arc') is not None:
    #         curvature = float(geom_xml.find('arc').attrib.get('curvature', 0.0))
    #         geom_type = 'arc'
    #     elif geom_xml.find('line') is not None:
    #         geom_type = 'line'
    #         curvature = 0.0
    #     elif geom_xml.find('spiral') is not None:
    #         geom_type = 'spiral'
    #         curstart = float(geom_xml.find('spiral').attrib.get('curvStart', 0.0))
    #         curend = float(geom_xml.find('spiral').attrib.get('curvEnd', 0.0))
    #         curvature = max(abs(curstart), abs(curend))
    #     elif geom_xml.find('poly3') is not None or geom_xml.find('paramPoly3') is not None:
    #         geom_type = 'poly3'
    #         curvature = 0.0
    #     else:
    #         geom_type = 'unknown'

    #     road_geometries.append({
    #         "s_start": s_start,
    #         "s_end": s_end,
    #         "delta_s": delta_s,
    #         "delta_t": delta_t,
    #         "straight_dist": straight_dist,
    #         "curvature": curvature,
    #         "geom_type": geom_type,
    #         "road_id": road_obj.id,
    #         "road_obj": road_obj,
    #     })
    # road_geometries.sort(key=lambda g: g['s_start'])
    # return road_geometries


def classify_road(geom, t_diff_threshold=1.0, curvature_threshold=0.001):
    if geom.get('curvature') is not None and abs(geom['curvature']) > curvature_threshold:
        return 'Corner'
    elif geom.get('t_diff') is not None and geom['t_diff'] > t_diff_threshold:
        return 'Corner'
    else:
        return 'Straight'

def compute_distance_for_vehicle(speed, maxspeed, hardware_delay, friction_coeff=0.8, reaction_buffer=0.2):
    reaction_time = hardware_delay + reaction_buffer
    braking_distance = (speed ** 2) / (2 * friction_coeff * 9.81)
    safe_forward_distance = speed * reaction_time + braking_distance
    safe_backward_distance = braking_distance * 0.9 + reaction_time * speed * 0.5
    return {
        'safe_forward_distance': float(safe_forward_distance),
        'safe_backward_distance': float(safe_backward_distance),
        'reaction_time': float(reaction_time)
    }

def safe_t_bounds(road_obj, lane_margin):
    try:
        left_border = road_obj.lane_borders['left']
        right_border = road_obj.lane_borders['right']
    except Exception:
        left_border = []
        right_border = []
    left_width = 0.0
    right_width = 0.0
    try:
        for lanes in road_obj.lanes.lane_sections:
            for lane_side in ('left', 'right'):
                lane_width = 0.0
                attr = getattr(lanes, 'lanes_' + lane_side, None)
                if hasattr(lane, 'lanes'):
                    lanes_dict = lanes.lanes
                    for lid, lane in lanes_dict.items():
                        try:
                            if hasattr(lane, 'max_width'):
                                lane_width += getattr(lane, 'max_width', 0.0)
                            elif hasattr(lane, 'widths'):
                                first_w = getattr(lane, 'widths')[0]
                                wval = getattr(first_w, 'a', None)
                                lane_width += float(wval) if wval is not None else 0.0
                        except Exception:
                            pass
                if lane_side == 'left':
                    left_width = max(left_width, lane_width)
                else:
                    right_width = max(right_width, lane_width)
    except Exception:
        left_width = max(left_width, 3.5)
        right_width = max(right_width, 3.5)
    
    return {
        "safe_t_left": float(left_width - lane_margin) if left_width > lane_margin else float(max(left_width * 0.8, 1.0)),
        "safe_t_right": float(right_width - lane_margin) if right_width > lane_margin else float(max(right_width * 0.8, 1.0))
    }

def build_segment_records(xodr_path, vehicle_state):
    roads = load_road_xodr(xodr_path)
    segment_records = []
    for road in roads:
        geoms = extract_road_geometries(road)
        for i, g in enumerate(geoms):
            g['class'] = classify_road(g)
        for i, g in enumerate(geoms):
            next_type = geoms[i+1]['class'] if i+1 < len(geoms) else None
            dist_to_next = (geoms[i+1]['s_start'] - vehicle_state.get('s_pos')) if i+1 < len(geoms) else None
            # compute safe distances from vehicle state
            speed = float(vehicle_state.get('speed', vehicle_state.get('currentspeed', 0.0)))
            maxspeed = float(vehicle_state.get('maxspeed', speed))
            minspeed = float(vehicle_state.get('minspeed', 0.0))
            hardware_delay = float(vehicle_state.get('hardware_delay', 0.1))
            mu = float(vehicle_state.get('mu', 0.8))
            safe_d = compute_distance_for_vehicle(speed, maxspeed, hardware_delay, mu=mu)
            delta_t_bounds = safe_t_bounds(road)
            # positions of other cars (optional)
            forward_car_s = vehicle_state.get('forward_car_s_pos', None)
            back_car_s = vehicle_state.get('back_car_s_pos', None)
            forward_gap = None
            back_gap = None
            if forward_car_s is not None:
                forward_gap = max(0.0, forward_car_s - vehicle_state.get('s_pos'))
            if back_car_s is not None:
                back_gap = max(0.0, vehicle_state.get('s_pos') - back_car_s)
            rec = {
                "road_id": g.get('road_id'),
                "geom_index": g.get('geom_index'),
                "s_start": g.get('s_start'),
                "s_end": g.get('s_end'),
                "delta_s": g.get('delta_s'),
                "delta_t": g.get('delta_t'),
                "straight_dist": g.get('straight_dist'),
                "curvature": g.get('curvature'),
                "geom_type": g.get('geom_type'),
                "current_seg_type": g.get('class'),
                "next_seg_type": next_type,
                "dist_to_next_segment": dist_to_next,
                # vehicle and runtime parameters
                "current_lap_time": float(vehicle_state.get('current_lap_time', 0.0)),
                "hardware_delay_time": hardware_delay,
                "speed": speed,
                "maxspeed": maxspeed,
                "currentspeed": speed,
                "minspeed": minspeed,
                # safe distances
                "safe_distance_forward": safe_d['safe_forward'],
                "safe_distance_back": safe_d['safe_back'],
                "forward_gap": forward_gap,
                "back_gap": back_gap,
                # lateral safe bounds
                "safe_delta_t_left": delta_t_bounds['safe_delta_t_left'],
                "safe_delta_t_right": delta_t_bounds['safe_delta_t_right'],
            }
            segment_records.append(rec)
    return segment_records


if __name__ == "__main__":
    xodr_path = "assets/maps/dSPACE/LagunaSeca.xodr"
    roads = load_road_xodr(xodr_path)
    print(roads[0].lane_sections)
    # for road in roads:
    #     extract_road_geometries(road)
    #print(json.dumps([extract_road_geometries(r) for r in roads], indent=2))