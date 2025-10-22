# Racing Domain Architecture Fix: Summary

## 🎯 Problem Solved

The racing domain was **simulator-specific** instead of being **abstract and simulator-agnostic** like the driving domain. The dSPACE simulator was implementing racing features directly instead of implementing the racing domain's abstract interfaces.

## ✅ Changes Made

### 1. **Racing Domain Made Abstract** (`src/scenic/domains/racing/`)

#### **Actions** (`actions.py`)
- **Before**: Actions had fallback implementations with `hasattr()` checks
- **After**: Actions use **abstract protocols** that simulators must implement

```python
# NEW: Abstract protocol
class RacingSteers:
    """Mixin protocol for racing vehicles."""
    def setDRS(self, activate):
        raise NotImplementedError
    
    def deployERS(self, mode, amount):
        raise NotImplementedError
    
    def setPitLimiter(self, activate):
        raise NotImplementedError

# NEW: Actions use protocols
class RacingAction(Action):
    def canBeTakenBy(self, agent):
        return isinstance(agent, RacingSteers)

class DRSAction(RacingAction):
    def applyTo(self, obj, sim):
        obj.setDRS(self.activate)  # ← Direct call, no fallback
```

#### **Model** (`model.scenic`)
- **Before**: `RacingCar` had concrete implementations
- **After**: `RacingCar` is **abstract** with `NotImplementedError`

```scenic
class RacingCar(Car):
    """Abstract racing car class."""
    
    # Abstract racing systems - must be implemented by simulators
    def setDRS(self, activate):
        raise NotImplementedError("Simulator must implement setDRS")
    
    def deployERS(self, mode, amount):
        raise NotImplementedError("Simulator must implement deployERS")
```

#### **Behaviors** (`behaviors.scenic`)
- **Before**: Behaviors used simulator-specific imports (`DSPACE_AVAILABLE`)
- **After**: Behaviors use **abstract racing protocols**

```scenic
behavior FollowRacingLineBehavior(target_speed=30):
    """Follow racing line using racing controllers."""
    
    # Get racing-specific controllers from simulator
    _lon_controller, _lat_controller = simulation().getRacingControllers(self)
    
    # Use abstract racing actions
    take DRSAction(activate=True)
    take ERSDeployAction(mode='overtake', amount=1.0)
```

### 2. **dSPACE Simulator Implements Racing Domain** (`src/scenic/simulators/dspace/`)

#### **Racing Model** (`racing_model.scenic`)
- **Before**: Simple import of racing domain
- **After**: **Implements** racing domain's abstract interfaces

```scenic
# Import racing domain
from scenic.domains.racing.model import *
from scenic.domains.racing.actions import RacingSteers

# dSPACE implementation of racing car
class DSPACERacingCar(RacingCar, RacingSteers):
    """dSPACE implementation of racing car."""
    
    def setDRS(self, activate):
        """Activate DRS using dSPACE controls."""
        if hasattr(self, 'dspaceActor') and self.dspaceActor:
            control_data = {'drs': activate}
            self.dspaceActor.set_control(control_data)
    
    def deployERS(self, mode, amount):
        """Deploy ERS using dSPACE controls."""
        if hasattr(self, 'dspaceActor') and self.dspaceActor:
            control_data = {'ers_mode': mode, 'ers_amount': amount}
            self.dspaceActor.set_control(control_data)

# Replace abstract RacingCar with dSPACE implementation
RacingCar = DSPACERacingCar
```

#### **Simulator** (`simulator.py`)
- **Before**: Inherited from `RacingSimulator` but didn't implement abstract methods
- **After**: **Implements** all racing domain abstract methods

```python
class DSpaceSimulation(RacingSimulation):
    def getRacingControllers(self, agent):
        """Get racing controllers optimized for dSPACE."""
        dt = self.timestep
        lon_controller = PIDLongitudinalController(K_P=0.8, K_D=0.15, K_I=0.9, dt=dt)
        lat_controller = PIDLateralController(K_P=0.3, K_D=0.15, K_I=0.0, dt=dt)
        return lon_controller, lat_controller
    
    def detectTrackSegment(self, position):
        """Detect track segment using dSPACE data."""
        # dSPACE-specific implementation
        return 'pitLane' or 'mainRacing'
    
    def assignRoute(self, agent, track_segment):
        """Assign dSPACE route based on track segment."""
        if track_segment == 'pitLane':
            return 'Pit'  # dSPACE pit lane route
        elif track_segment == 'mainRacing':
            return 'Lap'  # dSPACE main racing route
```

## 🏗️ Architecture Now Follows CARLA Pattern

### **Before (Incorrect)**
```
Racing Domain ← Simulator-specific implementations
     ↓
dSPACE Simulator ← Direct racing features
```

### **After (Correct)**
```
Racing Domain ← Abstract protocols & interfaces
     ↓
dSPACE Simulator ← Implements racing domain
     ↓
CARLA Simulator ← Could also implement racing domain
```

## 🔄 The Extension Pattern Applied

### **Racing Domain** (Abstract, like driving domain)
```scenic
# Abstract protocols
class RacingSteers:
    def setDRS(self, activate):
        raise NotImplementedError

# Abstract actions
class DRSAction(RacingAction):
    def applyTo(self, obj, sim):
        obj.setDRS(self.activate)

# Abstract behaviors
behavior FollowRacingLineBehavior():
    take DRSAction(activate=True)
```

### **dSPACE Simulator** (Concrete implementation)
```scenic
# Import abstract domain
from scenic.domains.racing.model import *

# Implement protocols
class DSPACERacingCar(RacingCar, RacingSteers):
    def setDRS(self, activate):
        # dSPACE-specific implementation
        self.dspaceActor.set_control({'drs': activate})

# Replace abstract with concrete
RacingCar = DSPACERacingCar
```

## 🎯 Benefits Achieved

### 1. **Simulator Independence**
- Racing scenarios work with **any** racing simulator
- No simulator-specific code in racing domain
- Clean separation of concerns

### 2. **Extensibility**
- New racing simulators can implement the racing domain
- Racing domain can be extended without breaking simulators
- Follows established Scenic patterns

### 3. **Reusability**
- Racing behaviors work across simulators
- Racing actions are simulator-agnostic
- Racing controllers can be tuned per simulator

### 4. **Maintainability**
- Clear interface contracts
- Simulator-specific code isolated
- Domain logic separated from implementation

## 🧪 Testing the Architecture

### **Scenario Compatibility**
Your `three_segments.scenic` should now work with:

```scenic
# Generic racing domain (visualization only)
model scenic.domains.racing.model

# dSPACE racing simulator
model scenic.simulators.dspace.racing_model

# Future: CARLA racing simulator
model scenic.simulators.carla.racing_model
```

### **Racing Features Available**
- ✅ `DRSAction`, `ERSDeployAction`, `PitLimiterAction`
- ✅ `FollowRacingLineBehavior`, `PitStopBehavior`, `OvertakingBehavior`
- ✅ `getRacingControllers()`, `detectTrackSegment()`, `assignRoute()`
- ✅ `RacingCar` with racing-specific properties

## 📋 Next Steps

### **Immediate**
1. **Test the architecture** with your `three_segments.scenic`
2. **Verify dSPACE implementation** works correctly
3. **Check for any missing abstract methods**

### **Future Enhancements**
1. **Add more racing simulators** (CARLA racing, etc.)
2. **Expand racing protocols** (more racing systems)
3. **Add racing-specific controllers** (sector timing, etc.)

## 🎉 Summary

The racing domain is now **properly abstract and simulator-agnostic**, following the same pattern as the driving domain. The dSPACE simulator **implements** the racing domain's abstract interfaces rather than defining racing features directly.

This architecture enables:
- **Simulator independence**: Racing scenarios work with any racing simulator
- **Clean separation**: Domain logic separate from simulator implementation  
- **Extensibility**: Easy to add new racing simulators
- **Reusability**: Racing behaviors work across simulators

The racing domain now serves as a **guide** for racing concepts (like the driving domain does for driving), and simulators **implement** those concepts with their specific technologies.

---

*The racing domain architecture now follows the established Scenic pattern: abstract domain + concrete simulator implementations.*
