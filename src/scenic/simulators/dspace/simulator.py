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

# ---------------- Small COM helpers (from your working script) ----------------

def _count_any(coll):
    try:
        c = getattr(coll, "Count", None)
        if c is None:
            c = len(coll)
        return int(c)
    except Exception:
        return 0

def _clear_collection(coll):
    n = _count_any(coll)
    for i in reversed(range(n)):
        for m in ("Remove", "Delete", "RemoveAt"):
            if hasattr(coll, m):
                try:
                    getattr(coll, m)(i)
                    break
                except Exception:
                    pass

def _ensure_two_segments(sequence):
    segs = sequence.Segments
    n = _count_any(segs)
    while n < 2:
        if hasattr(segs, "Add"):
            segs.Add()
            n += 1
        else:
            raise SimulationCreationError("Segments.Add() not available; create 2 segments once in UI.")
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
    tgt = typed_obj.ActiveElement
    tgt = tgt.SourceType
    tgt = tgt.ActiveElement
    tgt.Constant = float(value)

def _make_endless_transition(segs):
    try:
        tr = segs[1].Transition
        conds = tr.Conditions
        k = _count_any(conds)
        for i in reversed(range(k)):
            try:
                conds.Remove(i)
            except Exception:
                pass
        conds.Add("Endless")
    except Exception:
        pass

def _add_stationary_fellow(ts, s, t):
    """Create one Fellow: seg0 Longitudinal=Position(s); seg1 Velocity=0 & Lateral=Deviation(t); Endless."""
    fellows = ts.Fellows
    if not hasattr(fellows, "Add"):
        raise SimulationCreationError("Fellows.Add() not available — add one Fellow once via UI to expose COM path.")

    F = fellows.Add()
    seqs = F.Sequences
    S1 = seqs.Add() if hasattr(seqs, "Add") else seqs[0]
    segs = _ensure_two_segments(S1)

    lt0 = segs[0].Activity.LongitudinalType
    if not _activate_type(lt0, "Position"):
        raise SimulationCreationError("Could not activate LongitudinalType 'Position' on seg0.")
    _set_activity_constant(lt0, s)

    try:
        lat0 = segs[0].Activity.LateralType
        _activate_type(lat0, "Continue")
    except Exception:
        pass

    lt1 = segs[1].Activity.LongitudinalType
    if not _activate_type(lt1, "Velocity"):
        _activate_type(lt1, "Speed")
    _set_activity_constant(lt1, 0.0)

    try:
        lat1 = segs[1].Activity.LateralType
        _activate_type(lat1, "Deviation")
        _set_activity_constant(lat1, t)
    except Exception:
        pass

    _make_endless_transition(segs)
    return F

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
        """SaveAs/Activate the MD scenario first, then create Scenic objects.

        Core will later call updateObjects/step; we don't need to run VEOS.  
        (Core doc: setup() is where objects are created via _createObject → createObjectInSimulator.)"""
        # 1) SaveAs / Activate (before object creation)
        pythoncom.CoInitialize()
        app = Dispatch("ModelDesk.Application")
        proj = app.ActiveProject
        if proj is None:
            raise SimulationCreationError("Open a ModelDesk project first.")
        exp = proj.ActiveExperiment
        if exp is None:
            raise SimulationCreationError("Activate an experiment in ModelDesk.")

        # switch to source, then SaveAs to new scenario
        try:
            exp.ActivateTrafficScenario(self.sim.scenario_src)
        except Exception:
            pass

        name = self.sim.scenario_name or time.strftime("Scenic_%Y%m%d_%H%M%S")
        if self.sim.save_as:
            ts = exp.TrafficScenario
            try:
                ts.SaveAs(name, True)                        # same API used in your working script
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

        # rebind fresh handles after SaveAs/Activate
        pythoncom.PumpWaitingMessages()
        time.sleep(0.2)
        proj = app.ActiveProject
        self.exp = proj.ActiveExperiment
        self.ts  = self.exp.TrafficScenario
        if self.ts is None:
            raise SimulationCreationError("Active experiment has no TrafficScenario.")

        # Optional: start from a clean Fellows list each run
        try:
            _clear_collection(self.ts.Fellows)
        except Exception:
            pass

        # 2) NOW let the core create objects (calls our createObjectInSimulator for each)
        super().setup()  # ← object creation happens here in the core flow :contentReference[oaicite:2]{index=2}

        # persist the scenario with our newly added Fellows
        try:
            self.ts.Save()
            self.ts.Download()
            mc = exp.ManeuverControl
            time.sleep(0.2)
            mc.Reset()   # syncs the scenario state to the saved state
            time.sleep(0.2)
            mc.Start(True)
        except Exception:
            pass

    def createObjectInSimulator(self, obj):
        """Create Scenic object in ModelDesk: ego=no-op; non-ego Car → Fellow."""
        # Always keep a tiny backend object for Scenic bookkeeping
        b = type("DSpaceAgent", (), {})()
        b.position = Vector(0, 0, 0)
        b.linvel   = Vector(0, 0, 0)
        b.angvel   = Vector(0, 0, 0)
        b.heading  = 0.0
        obj._backend = b

        # If we don't have a TrafficScenario yet (shouldn't happen with the new order), bail
        if self.ts is None:
            return

        # Ego already exists in MD scenario → nothing to add
        if getattr(obj, "isEgo", False):
            return

        # Only map cars
        if not getattr(obj, "isCar", True):
            return

        # Resolve (s, t) from Scenic object.
        # Preferred: explicit Scenic properties (roadS/roadT) if you add them later.
        s = getattr(obj, "roadS", None)
        t = getattr(obj, "roadT", None)

        # Fallback: use Scenic position's (x, y) as (s, t) if present.
        if (s is None or t is None) and getattr(obj, "position", None) is not None:
            try:
                s = float(obj.position.x)
                t = float(obj.position.y)
            except Exception:
                pass

        # Final fallback: just place it near s=20, t=0 if nothing else is available.
        if s is None or t is None:
            s, t = 20.0, 0.0

        _add_stationary_fellow(self.ts, s, t)  # adds Fellow with the validated sequence

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
