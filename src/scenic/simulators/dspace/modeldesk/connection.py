"""ModelDesk COM connection and application access."""

import pythoncom
from win32com.client import Dispatch


class ModelDeskConnection:
    """Connection to ModelDesk COM application."""
    
    def __init__(self):
        self.app = None
        self.proj = None
        self.exp = None
    
    def connect(self):
        """Connect to ModelDesk COM application."""
        pythoncom.CoInitialize()
        self.app = Dispatch("ModelDesk.Application")
        self.proj = self.app.ActiveProject
        if self.proj is None:
            raise RuntimeError("Open a ModelDesk project first.")
        self.exp = self.proj.ActiveExperiment
        if self.exp is None:
            raise RuntimeError("Activate an experiment in ModelDesk.")
        return self
    
    def get_traffic_scenario(self):
        """Get the active TrafficScenario."""
        return self.exp.TrafficScenario


def connect_modeldesk():
    """Connect to ModelDesk COM application for scenario authoring."""
    try:
        pythoncom.CoInitialize()
        app = Dispatch("ModelDesk.Application")
        print("[ModelDesk] Connected to ModelDesk application")
        return app
    except Exception as e:
        print(f"[ModelDesk] Failed to connect: {e}")
        return None

