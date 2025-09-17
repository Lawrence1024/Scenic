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

from . import utils_md as utils
from . import utils as dutils


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
        self._road_index = None   # parsed from XODR
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

        # 4) Build XODR index from Scenic param map
        map_path = self._get_scx_map_path()
        if map_path:
            try:
                self._road_index = dutils.build_xodr_sec_points(map_path)
                print(f"[XODR] Reference polylines loaded: {map_path}")
            except Exception as e:
                print(f"[XODR] Failed to parse {map_path}: {e}")
                self._road_index = None
        else:
            print("[XODR] No Scenic `map` param found; will fall back to (0,0).")

        # 5) Let Scenic create objects (calls createObjectInSimulator)
        super().setup()

        # 6) Persist and (optionally) run
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
        """Place every car by absolute (s,t) computed from (x,y) and XODR."""
        print(f"Creating object in dSPACE with position and heading: {obj.position}, heading: {obj.heading}")

        # 1) Project Scenic (x,y) → (s,t). If no map, use zeros.
        if getattr(obj, "position", None) is not None and self._road_index is not None:
            s_val, t_val = dutils.project_world_to_st(self._road_index, (obj.position.x, obj.position.y))
        else:
            s_val, t_val = 0.0, 0.0

        # 2) Create Fellow with one Sequence and two Segments
        F = self.ts.Fellows.Add()
        try:
            if getattr(obj, "name", None):
                F.Name = str(obj.name)
        except Exception:
            pass

        seqs = F.Sequences
        dutils.clear_collection(seqs)
        S1 = seqs.Add() if hasattr(seqs, "Add") else seqs.Item(0)
        segs = dutils.ensure_two_segments(S1)

        # 3) seg0 = ABSOLUTE pose: Position = s, Deviation(Absolute) = t
        dutils.configure_seg0_absolute_pose(segs, s=float(s_val), t=float(t_val))

        # 4) seg1 = Velocity + Endless; keep lateral "Continue"
        base_v = getattr(obj, "md_v", None)
        if base_v is None:
            base_v = getattr(obj, "speed", 0.0) or 0.0
        dutils.configure_seg1_motion(segs, v=float(base_v), t=float(t_val))
        dutils.make_endless_transition(segs)

        return F

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
