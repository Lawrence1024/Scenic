import time
import json
import urllib.request

class AurelionSensorHub:
    """Tiny REST client for AURELION to provide a time/frame barrier and RGB pulls.

    NOTE: Placeholder field names! Replace SIM_TIME_KEY / FRAME_KEY / image endpoints
    with your real schema once you confirm them.
    """

    SIM_TIME_KEY = "simulationTime"   # <-- e.g., seconds (float)
    FRAME_KEY    = "frame"            # <-- integer frame index (optional)

    def __init__(self, base_url="http://localhost:8585", timeout=1.0):
        self.base = base_url.rstrip("/")
        self.timeout = timeout
        # you can accumulate per-sensor callbacks here later if needed

    # --- simulation time / frame ---
    def _get_json(self, path):
        with urllib.request.urlopen(self.base + path, timeout=self.timeout) as r:
            return json.loads(r.read())

    def read_sim_time(self):
        try:
            sim = self._get_json("/api/Aurelion/v2/Simulation")
            # Try a couple of likely keys; fall back to None if unknown
            for key in (self.SIM_TIME_KEY, "time", "simTime", "simulation_time"):
                if key in sim:
                    return float(sim[key])
            return None
        except Exception:
            return None

    def wait_until_time(self, target_time_s):
        """Spin/poll until AURELION reports simulation time >= target_time_s."""
        deadline = time.time() + 10  # hard cap; adjust if needed
        t = self.read_sim_time()
        while (t is None) or (t + 1e-9 < target_time_s):
            if time.time() > deadline:
                break
            time.sleep(0.001)
            t = self.read_sim_time()
        return t

    def barrier_flush(self):
        """Hook for per-sensor frame sync if you expose multiple streams."""
        return

# Example RGB “sensor” pull (wire into Scenic CallbackSensor later if desired)
class AurelionRGBClient:
    """Fetch the latest RGB frame as numpy array (placeholder: route & format)."""
    def __init__(self, hub: AurelionSensorHub, camera_id="FrontCam"):
        self.hub = hub
        self.camera_id = camera_id

    def latest_image(self):
        # Example placeholder route; replace with your actual endpoint/bytes
        path = f"/api/Aurelion/v2/Assets/{self.camera_id}/latest"
        try:
            with urllib.request.urlopen(self.hub.base + path, timeout=self.hub.timeout) as r:
                raw = r.read()  # bytes -> decode to numpy elsewhere when format is known
            return raw
        except Exception:
            return None
