"""ModelDesk scenario management."""

import time
import pythoncom


class ScenarioManager:
    """Manages ModelDesk scenario save/load/activation."""
    
    def __init__(self, exp, app):
        self.exp = exp
        self.app = app
        self.ts = None
    
    def save_as_scenario(self, source_name: str, new_name: str = None):
        """SaveAs scenario from source template.
        
        Args:
            source_name: Source scenario name
            new_name: New scenario name (auto-generated if None)
        """
        # 1) Switch to source, then SaveAs a working copy
        try:
            self.exp.ActivateTrafficScenario(source_name)
        except Exception:
            pass
        
        name = new_name or time.strftime("Scenic_%Y%m%d_%H%M%S")
        try:
            self.exp.TrafficScenario.SaveAs(name, True)
        except Exception:
            editor = self.exp.EditTrafficScenario()
            try:
                editor.SaveAs(name, True)
            finally:
                try:
                    editor.Close(False)
                except Exception:
                    pass
        try:
            self.exp.ActivateTrafficScenario(name)
        except Exception:
            pass
        
        # 2) Rebind fresh handles
        pythoncom.PumpWaitingMessages()
        time.sleep(0.2)
        self.exp = self.app.ActiveProject.ActiveExperiment
        self.ts = self.exp.TrafficScenario
        if self.ts is None:
            raise RuntimeError("Active experiment has no TrafficScenario.")
        
        return self.ts
    
    def clear_fellows(self):
        """Clear existing Fellows on the scenario."""
        try:
            from ..geometry import clear_collection
            clear_collection(self.ts.Fellows)
        except Exception:
            pass
    
    def start_simulation(self):
        """Save, download, reset, and start simulation."""
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

