def set_route(sequence, obj, detect_segment_fn, assign_route_fn):
    """Activate a ModelDesk route on a FellowSequence based on placement or preference.
    
    detect_segment_fn: fn(position_xy)->'pitLane'|'mainRacing'|None
    assign_route_fn: fn(agent, segment)->'Pit'|'Lap'|None
    """
    try:
        if not hasattr(sequence, 'Route'):
            return
        route_sel = sequence.Route

        # Determine preference
        route_preference = None
        try:
            pos = getattr(obj, 'position', None)
            if pos is not None:
                position_xy = (float(pos.x), float(pos.y))
                seg = detect_segment_fn(position_xy)
                if seg:
                    route_preference = assign_route_fn(obj, seg)
        except Exception:
            pass
        if not route_preference and hasattr(obj, 'dspace_route'):
            route_preference = obj.dspace_route
        if not route_preference:
            route_preference = 'Lap'

        # Try to activate directly by name; if that fails, try common variants
        candidates = [route_preference]
        low = route_preference.lower()
        if low.startswith('pit'):
            candidates += ['Pit', 'PitLane', 'Pit Lane']
        else:
            candidates += ['Lap', 'Main', 'MainRoute', 'Main Route']

        for name in candidates:
            try:
                route_sel.Activate(name)
                # Success if no exception
                return
            except Exception:
                continue

        # Try AvailableElements enumeration (best effort)
        try:
            available = getattr(route_sel, 'AvailableElements', None)
            if available is not None:
                for idx, r in enumerate(available):
                    try:
                        name = r.Name if hasattr(r, 'Name') else str(r)
                        if name:
                            route_sel.Activate(name)
                            return
                    except Exception:
                        continue
        except Exception:
            pass
    except Exception as e:
        print(f"[Routes] set_route error: {e}")


