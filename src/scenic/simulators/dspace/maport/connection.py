# -*- coding: utf-8 -*-
"""MAPort (XIL API) wrapper for variable read/write.

Provides get_var(path) and set_var(path, value) with the same semantics as
ControlDesk COM, so the simulator can use MAPort for variable access while
keeping session control (go online, start maneuver, step) in ControlDesk.

Requires: pythonnet (clr), dSPACE XIL API .NET assemblies in GAC.
"""

import time
from typing import Any

# Lazy clr / XIL imports so this module only loads when MAPort is used
_clr_loaded = False


def _ensure_clr():
    global _clr_loaded
    if _clr_loaded:
        return
    import clr
    clr.AddReference(
        "ASAM.XIL.Implementation.TestbenchFactory, Version=2.2.0.0, Culture=neutral, PublicKeyToken=bf471dff114ae984"
    )
    clr.AddReference(
        "ASAM.XIL.Interfaces, Version=2.2.0.0, Culture=neutral, PublicKeyToken=bf471dff114ae984"
    )
    _clr_loaded = True


def _to_python_value(val):
    """Convert .NET value to Python scalar or list."""
    if val is None:
        return None
    try:
        # .NET collections (e.g. List[Double]) are iterable in pythonnet
        if hasattr(val, "GetEnumerator") and hasattr(val, "Count"):
            return [float(x) for x in val]
    except Exception:
        pass
    try:
        return float(val)
    except (TypeError, ValueError):
        pass
    return val


class MAPortApp:
    """MAPort-based variable access: get_var(path) and set_var(path, value).

    Mirrors the variable-access API of ControlDeskApp (controldesk/connection.py).
    Session control (go_online, start_maneuver, advance_simulation_step) is not
    provided here; use ControlDesk for that.
    """

    def __init__(self, config_path: str):
        self._config_path = config_path
        self._maport = None
        self._vrf = None  # VariableRefFactory
        self._vf = None   # ValueFactory
        self._write_refs_cache = {}
        self._timing_log = []  # optional: (path, 'get'|'set', duration_sec)

    def connect(self, start_if_needed: bool = True):
        """Create and configure MAPort. Optionally start simulation if not running."""
        _ensure_clr()
        from ASAM.XIL.Implementation.TestbenchFactory.Testbench import TestbenchFactory
        from ASAM.XIL.Interfaces.Testbench.MAPort.Enum import MAPortState

        factory = TestbenchFactory()
        testbench = factory.CreateVendorSpecificTestbench("dSPACE GmbH", "XIL API", "2023-A")
        self._vrf = testbench.VariableRefFactory
        self._vf = testbench.ValueFactory
        self._maport = testbench.MAPortFactory.CreateMAPort("ScenicMAPort")
        config = self._maport.LoadConfiguration(self._config_path)
        self._maport.Configure(config, False)
        if start_if_needed and self._maport.State != MAPortState.eSIMULATION_RUNNING:
            self._maport.StartSimulation()
        return self

    def get_var(self, path: str) -> Any:
        """Read variable at path. Returns scalar (float/int) or Python list for arrays."""
        from ASAM.XIL.Interfaces.Testbench.Common.VariableRef.Enum import ValueRepresentation
        from .DemoHelpers import convertIBaseValue

        t0 = time.perf_counter()
        try:
            ref = self._vrf.CreateGenericVariableRef(path, ValueRepresentation.ePhysicalValue)
            raw = self._maport.Read2(ref)
            conv = convertIBaseValue(raw)
            val = conv.Value
            out = _to_python_value(val)
            return out
        finally:
            self._timing_log.append((path, "get", time.perf_counter() - t0))

    def _get_write_ref(self, path: str):
        """Cached variable reference for writes (same idea as ControlDesk)."""
        from ASAM.XIL.Interfaces.Testbench.Common.VariableRef.Enum import ValueRepresentation

        if path in self._write_refs_cache:
            return self._write_refs_cache[path]
        ref = self._vrf.CreateGenericVariableRef(path, ValueRepresentation.ePhysicalValue)
        self._write_refs_cache[path] = ref
        return ref

    def set_var(self, path: str, value: Any):
        """Write variable at path. value can be scalar (float/int) or list (array)."""
        t0 = time.perf_counter()
        try:
            ref = self._get_write_ref(path)
            if hasattr(value, "__iter__") and not isinstance(value, (str, bytes)):
                arr = list(value)
                import System
                from System import Array
                self._maport.Write2(ref, self._vf.CreateFloatVectorValue(Array[System.Double](arr)))
            else:
                self._maport.Write2(ref, self._vf.CreateFloatValue(float(value)))
        finally:
            self._timing_log.append((path, "set", time.perf_counter() - t0))

    def dispose(self):
        """Release MAPort and clear caches."""
        if self._maport is not None:
            try:
                self._maport.Dispose()
            except Exception:
                pass
            self._maport = None
        self._write_refs_cache.clear()
        self._vrf = None
        self._vf = None

    def print_timing_summary(self):
        """Print per-path timing summary (optional, like ControlDesk)."""
        if not self._timing_log:
            return
        agg = {}
        for path, op, duration in self._timing_log:
            key = (path, op)
            if key not in agg:
                agg[key] = [0.0, 0]
            agg[key][0] += duration
            agg[key][1] += 1
        rows = [(path, op, total, count) for (path, op), (total, count) in agg.items()]
        rows.sort(key=lambda x: -x[2])
        print("[MAPort Timing] Per-path summary (total_sec, count, mean_ms):")
        for path, op, total, count in rows:
            mean_ms = (total / count) * 1000.0 if count else 0
            parts = path.split("/")
            short = "/".join(parts[-2:]) if len(parts) >= 2 else path
            print("  [%3s] %.3fs  n=%6d  mean=%.2fms  %s" % (op, total, count, mean_ms, short))
        total_all = sum(d for _, _, d in self._timing_log)
        print("[MAPort Timing] TOTAL: %.3fs over %d calls" % (total_all, len(self._timing_log)))
