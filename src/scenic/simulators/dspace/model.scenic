from scenic.domains.driving.model import *
from scenic.domains.driving.actions import *
from scenic.domains.driving.behaviors import *

# Try to import dSPACE simulator and action markers
try:
    import scenic.simulators.dspace as dspace
    from scenic.simulators.dspace.actions import _DSpaceVehicle
except Exception:
    # Fallbacks so scenarios can compile without full dSPACE backend available
    from scenic.core.simulators import SimulatorInterfaceWarning as _SIW
    import warnings as _warnings
    _warnings.warn('dSPACE backend not fully available; running without dynamic control', _SIW)

    class _DSpaceVehicle: pass


# dSPACE ModelDesk parameters
param scenario_src = "LagunaSeca_ExternalControl"
param scenario_name = None
param timestep = 0.1

# Configure the dSPACE simulator
simulator dspace.DSpaceSimulator(
    scenario_src=scenario_src,
    scenario_name=scenario_name,
    timestep=float(timestep),
)


class DSpaceActor(DrivingObject):
    """Base class for dSPACE-backed Scenic objects.

    Provides storage for desired control values which the dSPACE simulator
    can consume each tick (or via external control).
    """
    # Common properties used in dSPACE naming conventions
    name: None
    raceNumber: None   # optional racing number (F1/F2...)

    # Internal desired control state (used by actions/behaviors)
    _desiredThrottle: 0.0
    _desiredBrake: 0.0
    _desiredSteer: 0.0
    _desiredVelocity: None


class Vehicle(Vehicle, DSpaceActor, Steers, _DSpaceVehicle):
    """Steerable vehicle backed by dSPACE.

    Multiple inheritance composes:
    - Driving-domain Vehicle (semantics/geometry/eligibility for behaviors)
    - DSpaceActor (simulator binding and desired control storage)
    - Steers (declares steering/throttle/brake capabilities for actions)
    - _DSpaceVehicle (Python-side marker for action dispatch)
    """

    def setThrottle(self, throttle):
        self._desiredThrottle = float(throttle)

    def setSteering(self, steering):
        self._desiredSteer = float(steering)

    def setBraking(self, braking):
        self._desiredBrake = float(braking)

    def setReverse(self, reverse):
        # Placeholder for future gearbox integration via ControlDesk/ModelDesk
        pass

    @property
    def isCar(self):
        return True
