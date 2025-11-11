def get_fellow_index(sim, obj):
    """Get the fellow index for a vehicle object (0-based), delegating to sim state."""
    try:
        for name, vehicle_data in sim._fellow_vehicles.items():
            if vehicle_data.get('scenic_object') is obj:
                if 'index' in vehicle_data:
                    index = int(vehicle_data['index'])
                    if 0 <= index < 100:
                        return index
                if name.startswith('F') and len(name) > 1 and name[1:].isdigit():
                    index = int(name[1:]) - 1
                    if 0 <= index < 100:
                        return index
                if name.startswith('Fellow_'):
                    try:
                        index = int(name[7:]) - 1
                        if 0 <= index < 100:
                            return index
                    except ValueError:
                        pass
        fellow_objects = [o for o in sim.scene.objects if o is not sim.scene.egoObject]
        if obj in fellow_objects:
            index = int(fellow_objects.index(obj))
            if 0 <= index < 100:
                return index
        print(f"[_getFellowIndex] Could not determine valid index for {obj}")
        return None
    except Exception as e:
        print(f"[_getFellowIndex] Error: {e}")
        import traceback
        traceback.print_exc()
        return None


