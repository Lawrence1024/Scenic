# -*- coding: utf-8 -*-
# Scenic ↔ dSPACE (ModelDesk) adapter
# - SaveAs/Activate the MD TrafficScenario at setup-time (before object creation)
# - Add each non-ego Scenic Car as a Fellow (2 segments; seg1 Deviation; Endless)
# - No VEOS/AURELION run; return neutral properties to the core

import time
import pythoncom
from win32com.client import Dispatch

from scenic.core.vectors import Vector
from scenic.domains.driving.simulators import DrivingSimulator, DrivingSimulation
from scenic.core.simulators import SimulationCreationError  # core contract

# --- COM helpers (from your working example) ---
def _count_any(coll):
    try: return int(getattr(coll, "Count", len(coll)))
    except Exception: return 0

def _clear_collection(coll):
    print("Clearing collection of size", _count_any(coll))
    n = _count_any(coll)
    for i in reversed(range(n)):
        for m in ("Remove", "Delete", "RemoveAt"):
            if hasattr(coll, m):
                try: getattr(coll, m)(i); break
                except Exception: pass

def _ensure_two_segments(sequence):
    segs = sequence.Segments
    while _count_any(segs) < 2:
        if hasattr(segs, "Add"): segs.Add()
        else: raise SimulationCreationError("Segments.Add() missing; create 2 segs once in UI.")
    return segs

def _activate_type(typed_obj, element_name):
    try:
        typed_obj.Activate(element_name); return True
    except Exception:
        pass
    try:
        avail = getattr(typed_obj, "AvailableElements", None)
        if avail:
            for el in avail:
                if str(el).lower() == element_name.lower():
                    typed_obj.Activate(el); return True
    except Exception:
        pass
    return False

def _set_activity_constant(typed_obj, value):
    tgt = typed_obj.ActiveElement.SourceType.ActiveElement
    tgt.Constant = float(value)

def _make_endless_transition(segs):
    try:
        conds = segs[1].Transition.Conditions
        for i in reversed(range(_count_any(conds))):
            try: conds.Remove(i)
            except Exception: pass
        conds.Add("Endless")
    except Exception:
        pass


# ---------------- Public simulator (no COM side effects here) ----------------

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

# ---------------- Per-run simulation ----------------------------------------

class DSpaceSimulation(DrivingSimulation):
    def __init__(self, scene, sim: DSpaceSimulator, **kwargs):
        self.sim = sim
        self.exp = None      # set in setup()
        self.ts  = None
        ts = kwargs.pop("timestep", None) or sim.timestep
        super().__init__(scene, timestep=ts, **kwargs)

    def setup(self):
        """SaveAs/Activate first, then create Scenic objects, then persist & start."""
        pythoncom.CoInitialize()
        app = Dispatch("ModelDesk.Application")
        proj = app.ActiveProject
        if proj is None:
            raise SimulationCreationError("Open a ModelDesk project first.")
        exp = proj.ActiveExperiment
        if exp is None:
            raise SimulationCreationError("Activate an experiment in ModelDesk.")

        # 1) Switch to source, then SaveAs to new scenario
        try:
            exp.ActivateTrafficScenario(self.sim.scenario_src)
        except Exception:
            pass

        name = self.sim.scenario_name or time.strftime("Scenic_%Y%m%d_%H%M%S")
        if self.sim.save_as:
            ts0 = exp.TrafficScenario
            try:
                ts0.SaveAs(name, True)    # same API as your working script
            except Exception:
                editor = exp.EditTrafficScenario()
                try:
                    editor.SaveAs(name, True)
                finally:
                    try: editor.Close(False)
                    except Exception: pass
            try:
                exp.ActivateTrafficScenario(name)
            except Exception:
                pass

        # 2) Rebind fresh handles after SaveAs/Activate
        pythoncom.PumpWaitingMessages()
        time.sleep(0.2)
        proj = app.ActiveProject
        self.exp = proj.ActiveExperiment
        self.ts  = self.exp.TrafficScenario
        if self.ts is None:
            raise SimulationCreationError("Active experiment has no TrafficScenario.")

        # 3) Start from a clean Fellows list (operate on FRESH self.ts)
        try:
            _clear_collection(self.ts.Fellows)
        except Exception:
            pass

        # 4) NOW let Scenic create objects (calls our createObjectInSimulator for each)
        super().setup()   # <— this is where your Fellow is actually added

        # 5) Persist configuration and (optionally) reset/start
        try:
            self.ts.Save()
            self.ts.Download()

            mc = self.exp.ManeuverControl
            # Be defensive in case VEOS is mid-run
            try: mc.Stop()
            except Exception: pass
            time.sleep(0.2)
            mc.Reset()
            time.sleep(0.2)
            mc.Start(False)   # blocking-normal run; use True for non-blocking if desired
        except Exception:
            pass


        

    def createObjectInSimulator(self, obj):
        # Minimal backend so Scenic stays happy
        print(f"Creating object in DSpaceSimulation: {obj}")
        b = type("DSpaceAgent", (), {})()
        b.position = Vector(0, 0, 0)
        b.linvel   = Vector(0, 0, 0)
        b.angvel   = Vector(0, 0, 0)
        b.heading  = 0.0
        obj._backend = b

        # Only add non-ego cars to ModelDesk
        if self.ts is None or getattr(obj, "isEgo", False) or not getattr(obj, "isCar", True):
            return

        ts = self.ts


        # --------- read Scenic-specified placement/controls ----------
        # (s, t) longitudinal & lateral in road coordinates
        s = getattr(obj, "roadS", None)
        t = getattr(obj, "roadT", None)

        # Convenience: allow using Scenic position (x,y) as (s,t)
        if (s is None or t is None) and getattr(obj, "position", None) is not None:
            try:
                s = float(obj.position.x)
                t = float(obj.position.y)
            except Exception:
                pass

        # Defaults if not provided
        if s is None: s = 20.0
        if t is None: t = 0.0

        # Longitudinal/Lateral “kinds” and values (override-able from Scenic)
        seg0_long_kind = getattr(obj, "md_seg0_long_kind", "Position")   # e.g., "Position"
        seg0_long_val  = getattr(obj, "md_seg0_long_val",  s)            # value for seg0.longitudinal

        seg1_long_kind = getattr(obj, "md_seg1_long_kind", "Velocity")   # "Velocity" or "Speed"
        seg1_long_val  = getattr(obj, "md_seg1_long_val",  getattr(obj, "md_v", 0.0))  # velocity m/s

        seg1_lat_kind  = getattr(obj, "md_seg1_lat_kind",  "Deviation")  # "Deviation"
        seg1_lat_val   = getattr(obj, "md_seg1_lat_val",   t)            # lateral dev m

        # --------- COM: add the fellow with two segments ----------
        fellows = ts.Fellows
        if not hasattr(fellows, "Add"):
            raise SimulationCreationError("Fellows.Add() not available—create one in UI once to expose COM.")

        F = fellows.Add()
        seqs = F.Sequences
        _clear_collection(seqs)
        S1 = seqs.Add() if hasattr(seqs, "Add") else seqs[0]
        segs = _ensure_two_segments(S1)

        print(f"segs: {segs}, kinds: {seg0_long_kind}, {seg1_long_kind}, {seg1_lat_kind}, vals: {seg0_long_val}, {seg1_long_val}, {seg1_lat_val}")

        # Segment 0: longitudinal kind/value (default=Position(s)), lateral neutral
        lt0 = segs[0].Activity.LongitudinalType
        if not _activate_type(lt0, seg0_long_kind):
            raise SimulationCreationError(f"Could not activate seg0 LongitudinalType '{seg0_long_kind}'.")
        _set_activity_constant(lt0, seg0_long_val)
        try:
            lat0 = segs[0].Activity.LateralType
            _activate_type(lat0, "Continue")
        except Exception:
            pass

        # Segment 1: longitudinal kind/value (default=Velocity(v))
        lt1 = segs[1].Activity.LongitudinalType
        if not _activate_type(lt1, seg1_long_kind):
            _activate_type(lt1, "Speed")  # fallback if build exposes Speed
        _set_activity_constant(lt1, seg1_long_val)

        # Segment 1: lateral kind/value (default=Deviation(t))
        try:
            lat1 = segs[1].Activity.LateralType
            _activate_type(lat1, seg1_lat_kind)
            _set_activity_constant(lat1, seg1_lat_val)
        except Exception:
            pass

        _make_endless_transition(segs)


    # We are not “running” the sim; keep a tiny sleep so the core loop advances.
    def step(self):
        time.sleep(self.timestep)

    # Core asks us for dynamic properties; return neutral values of correct types
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
    

