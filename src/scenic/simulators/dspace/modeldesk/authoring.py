import time

from ..controldesk.per_tick_control import ExternalControlManager
from .traffic_object import apply_fellow_traffic_object


def author_scenario(sim):
    """Author scenario in ModelDesk using COM automation. Delegated from simulator."""
    try:
        _setup_scenario(sim)
        # Configure fellows based on Scenic objects (skip ego and already-created fellows)
        for scenic_obj in sim.scene.objects:
            if scenic_obj is sim.scene.egoObject:
                continue
            try:
                exists = any(v.get('scenic_object') is scenic_obj for v in sim._fellow_vehicles.values())
            except Exception:
                exists = False
            if exists:
                continue
            if hasattr(scenic_obj, 'raceNumber'):
                configure_fellow(sim, scenic_obj)
        # External control
        ExternalControlManager.enableExternalControlViaScript(sim.scene.objects)
        # Check consistency and download
        if sim.ts.CheckConsistency():
            print("[ModelDesk] Scenario is consistent")
            _pre_download_s = float(getattr(sim.sim, "pre_download_delay_s", 30.0))
            print(f"[ModelDesk] Pre-download pause {_pre_download_s:.1f}s ...")
            time.sleep(_pre_download_s)
            downloaded = sim.ts.Download()
            _post_download_s = float(getattr(sim.sim, "post_modeldesk_download_delay_s", 30.0))
            print(f"[ModelDesk] Post-download pause {_post_download_s:.1f}s ...")
            time.sleep(_post_download_s)
            if downloaded:
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


def _setup_scenario(sim):
    """Setup ModelDesk scenario using existing save_as logic (wrapper)."""
    print(f"[ModelDesk] Using existing scenario: {sim.ts.Name}")


def configure_fellow(sim, scenic_obj):
    """Configure a fellow in ModelDesk: name, route, indexing, store reference."""
    try:
        fellow_name = f"F{scenic_obj.raceNumber}"
        try:
            fellow = sim.ts.Fellows.Item(fellow_name)
            print(f"[ModelDesk] Using existing fellow: {fellow_name}")
        except:
            fellow = sim.ts.Fellows.Add()
            fellow.Name = fellow_name
            print(f"[ModelDesk] Created new fellow: {fellow_name}")
        sequences = fellow.Sequences
        if sequences.Count == 0:
            seq = sequences.Add()
        else:
            seq = sequences.Item(1)
        route_sel = seq.Route
        # Determine track segment robustly
        try:
            pos_xy = (float(scenic_obj.position.x), float(scenic_obj.position.y))
        except Exception:
            pos_xy = None
        track_segment = None
        try:
            if pos_xy is not None:
                track_segment = sim.detectTrackSegment(pos_xy)
        except Exception:
            track_segment = None
        if track_segment not in ('pitLane', 'mainRacing'):
            track_segment = 'mainRacing'
        desired_pref = (sim.assignRoute(scenic_obj, track_segment) or 'Lap')
        desired_is_pit = (desired_pref.lower().startswith('pit'))
        chosen_route = None
        available_names = []
        try:
            available = list(route_sel.AvailableElements)
            available_names = [str(x) for x in available]
            if desired_is_pit:
                pit_candidates = [n for n in available_names if 'pit' in n.lower()]
                if pit_candidates:
                    chosen_route = pit_candidates[0]
            else:
                non_pit = [n for n in available_names if 'pit' not in n.lower()]
                if non_pit:
                    chosen_route = non_pit[0]
            if chosen_route is None and available_names:
                chosen_route = available_names[0]
        except Exception as e:
            print(f"[ModelDesk] Could not enumerate AvailableElements: {e}")
        if not chosen_route:
            chosen_route = desired_pref
        try:
            route_sel.Activate(chosen_route)
            print(f"[ModelDesk] Set route '{chosen_route}' (from pref '{desired_pref}') for {fellow_name}")
        except Exception as e:
            print(f"[ModelDesk] Failed to set route '{chosen_route}' for {fellow_name}: {e}")
            if available_names:
                print(f"[ModelDesk] Available routes: {available_names}")
        try:
            route_sel.Direction = 1
            route_sel.UseExternal = True
        except Exception:
            pass
        apply_fellow_traffic_object(fellow)
        sim._fellow_vehicles[fellow_name] = {
            'fellow_object': fellow,
            'scenic_object': scenic_obj,
            'sequence': seq
        }
    except Exception as e:
        print(f"[ModelDesk] Error configuring fellow: {e}")


