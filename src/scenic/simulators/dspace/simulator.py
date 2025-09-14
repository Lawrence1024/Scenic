# scenic/simulators/dspace/simulator.py
import time
import pythoncom
from win32com.client import Dispatch
from scenic.domains.driving.simulators import DrivingSimulator, DrivingSimulation
from scenic.syntax.veneer import verbosePrint

# ---------------- AURELION time/frame barrier (Option C) ----------------
import json, urllib.request

class AurelionHub:
    # Replace base URL / keys once confirmed
    def __init__(self, base="http://localhost:8585", timeout=1.0):
        self.base = base.rstrip("/")
        self.timeout = timeout
    def _get(self, path):
        with urllib.request.urlopen(self.base + path, timeout=self.timeout) as r:
            return json.loads(r.read())
    def read_sim_time(self):
        try:
            js = self._get("/api/Aurelion/v2/Simulation")  # placeholder route
            for key in ("simulationTime", "simTime", "time"):
                if key in js:
                    return float(js[key])
        except Exception:
            pass
        return None
    def wait_until(self, target_s):
        t = self.read_sim_time()
        deadline = time.time() + 10
        while t is None or t + 1e-9 < target_s:
            if time.time() > deadline:
                break
            time.sleep(0.002)
            t = self.read_sim_time()
        return t

# ---------------- Minimal agent wrapper providing the Steers protocol ---
# This makes Scenic driving actions available immediately; later we route to COM.
class DSpaceAgent:
    isCar = True  # good default; behaviors use this for PID choice

    def __init__(self, name="Ego"):
        self.name = name
        self._throttle = 0.0
        self._steer = 0.0
        self._brake = 0.0
        self._handbrake = False
        self._reverse = False
        self.position = None   # fill from COM/REST if you wish
        self.heading = 0

    # ---- Steers protocol methods (required by driving.actions) ----
    def setThrottle(self, v):   self._throttle = float(v)
    def setSteering(self, v):   self._steer = float(v)
    def setBraking(self, v):    self._brake = float(v)
    def setHandbrake(self, v):  self._handbrake = bool(v)
    def setReverse(self, v):    self._reverse = bool(v)

# ---------------- Simulator --------------------------------------------
class DSpaceSimulator(DrivingSimulator):
    """dSPACE (ModelDesk/VEOS + AURELION) simulator facade for Scenic driving."""

    def __init__(self,
                 scenario_src="LagunaSeca_ExternalControl",
                 project_path=None,           # if None, attach to the open project
                 timestep=0.05,
                 aurl_base="http://localhost:8585",
                 saveas_prefix="LagunaSeca_ExternalControl__scenic_",
                 clone_scenario=True,
                 render=False):
        super().__init__()
        self.timestep = timestep
        self.render = render
        self._aurl = AurelionHub(aurl_base)

        # Attach to ModelDesk and prep experiment
        pythoncom.CoInitialize()
        app = Dispatch("ModelDesk.Application")
        self.proj = app.ActiveProject if project_path is None else app.OpenProject(project_path)
        self.exp  = self.proj.ActiveExperiment

        verbosePrint(f"[dSPACE] Connected to ModelDesk; activating scenario: {scenario_src}")
        self.exp.ActivateTrafficScenario(scenario_src)

        if clone_scenario:
            clone = f"{saveas_prefix}{int(time.time())}"
            ts = self.exp.TrafficScenario
            ts.SaveAs(clone, True)                     # <-- correct SaveAs location/signature
            self.exp.ActivateTrafficScenario(clone)

        # Ribbon equivalents
        self.exp.ManeuverControl.Reset()
        self.exp.ManeuverControl.Start(False)          # <-- Start(wait=False) to avoid COM arg error

    # Scenic hook
    def createSimulation(self, scene, **kwargs):
        return DSpaceSimulation(scene, self, **kwargs)

# ---------------- Per-run Simulation -----------------------------------
# --- add this class attribute (declares what you can return) ---
# scenic/simulators/dspace/simulator.py
from scenic.core.vectors import Vector

class DSpaceSimulation(DrivingSimulation):
    def __init__(self, scene, sim, **kwargs):
        self.sim = sim
        ts = kwargs.pop('timestep', None) or sim.timestep
        super().__init__(scene, timestep=ts, **kwargs)   # ← no extra kwargs

    def createObjectInSimulator(self, obj):
        # create/bind a backend handle; seed defaults so getProperties can answer
        # (replace this with real MD/VEOS/AURELION handles later)
        b = type("DSpaceAgent", (), {})()
        b.position = Vector(0.0, 0.0, 0.0)
        b.heading  = 0.0           # radians
        b.linvel   = Vector(0.0, 0.0, 0.0)
        b.angvel   = Vector(0.0, 0.0, 0.0)
        obj._backend = b

    def getProperties(self, obj, properties):
        # read back from backend (replace placeholders with COM/REST reads)
        b = getattr(obj, "_backend", None)
        if b is None:
            # still must return all requested keys
            pos = Vector(0.0, 0.0, 0.0)
            vel = Vector(0.0, 0.0, 0.0)
            ang = Vector(0.0, 0.0, 0.0)
            yaw = 0.0
        else:
            pos = getattr(b, "position", Vector(0,0,0))
            vel = getattr(b, "linvel",   Vector(0,0,0))
            ang = getattr(b, "angvel",   Vector(0,0,0))
            yaw = getattr(b, "heading",  0.0)

        out = {
            "position":        pos,                         # Vector
            "velocity":        vel,                         # Vector
            "speed":           vel.norm(),                  # float
            "angularVelocity": ang,                         # Vector
            "angularSpeed":    ang.norm(),                  # float
            "yaw":             float(yaw),                  # float (radians)
            "pitch":           0.0,                         # float
            "roll":            0.0,                         # float
            "elevation":       float(pos.z),                # float
        }
        # VERY IMPORTANT: return exactly the requested keys
        return {k: out[k] for k in properties}


    def setup(self):
        super().setup()
        self._last_time = self.sim._aurl.read_sim_time()

    def executeActions(self, allActions):
        # Scenic will call actions; our agent wrapper already satisfies Steers.
        super().executeActions(allActions)
        # TODO: forward _backend control values to ModelDesk/VEOS via COM if desired.

    def step(self):
        # Option C: block until AURELION time reaches last + dt
        target = (self._last_time or 0.0) + self.timestep
        tnext = self.sim._aurl.wait_until(target)
        self._last_time = tnext

    # Controller gains for lane-follow/turning/lane-change come from DrivingSimulation;
    # override here only if dSPACE dynamics need different PID values. :contentReference[oaicite:5]{index=5}
