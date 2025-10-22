# Domain Architecture Quick Reference

## The Layered Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    SCENIC SCENARIOS (.scenic)                │
│  Example: three_segments.scenic, overtaking.scenic          │
└───────────────────────────┬─────────────────────────────────┘
                            │ imports
┌───────────────────────────▼─────────────────────────────────┐
│              RACING DOMAIN (scenic.domains.racing)           │
│  ┌──────────────────────────────────────────────────────┐   │
│  │ model.scenic - Racing world model                    │   │
│  │  • Imports: from driving.model import *              │   │
│  │  • Adds: RacingCar, track, racingLine, pitLane      │   │
│  ├──────────────────────────────────────────────────────┤   │
│  │ tracks.py - Racing infrastructure                    │   │
│  │  • RacingTrack(network: Network)  ← wraps driving    │   │
│  │  • PitLane, Sector, RacingLine                       │   │
│  ├──────────────────────────────────────────────────────┤   │
│  │ actions.py - Racing actions                          │   │
│  │  • Imports: from driving.actions import *            │   │
│  │  • Adds: DRSAction, ERSDeployAction, PitLimiter     │   │
│  ├──────────────────────────────────────────────────────┤   │
│  │ behaviors.scenic - Racing behaviors                  │   │
│  │  • Imports: from driving.behaviors import *          │   │
│  │  • Adds: Racing-specific behaviors                   │   │
│  └──────────────────────────────────────────────────────┘   │
└───────────────────────────┬─────────────────────────────────┘
                            │ extends/uses
┌───────────────────────────▼─────────────────────────────────┐
│             DRIVING DOMAIN (scenic.domains.driving)          │
│  ┌──────────────────────────────────────────────────────┐   │
│  │ model.scenic - Driving world model                   │   │
│  │  • Network.fromFile(map) → road network              │   │
│  │  • Regions: road, sidewalk, intersection             │   │
│  │  • Objects: Vehicle, Car, Pedestrian                 │   │
│  ├──────────────────────────────────────────────────────┤   │
│  │ roads.py - Road network infrastructure               │   │
│  │  • Network, Road, Lane, Intersection                 │   │
│  │  • Load from OpenDRIVE, SUMO, etc.                   │   │
│  ├──────────────────────────────────────────────────────┤   │
│  │ actions.py - Driving actions                         │   │
│  │  • Steers, Walks protocols                           │   │
│  │  • SetThrottle, SetSteer, SetBrake                   │   │
│  ├──────────────────────────────────────────────────────┤   │
│  │ behaviors.scenic - Driving behaviors                 │   │
│  │  • FollowLaneBehavior, TurnBehavior                  │   │
│  │  • LaneChangeBehavior                                │   │
│  ├──────────────────────────────────────────────────────┤   │
│  │ controllers.py - PID controllers                     │   │
│  │  • Longitudinal, Lateral controllers                 │   │
│  └──────────────────────────────────────────────────────┘   │
└───────────────────────────┬─────────────────────────────────┘
                            │ uses
┌───────────────────────────▼─────────────────────────────────┐
│                  CORE SCENIC (scenic.core.*)                 │
│  • Object, Region, VectorField                               │
│  • Distributions, Specifiers                                 │
│  • Simulator interface                                       │
└──────────────────────────────────────────────────────────────┘
```

---

## The Extension Pattern

### How Racing Extends Driving

```
DRIVING DOMAIN                    RACING DOMAIN
──────────────                    ─────────────

Network                           RacingTrack
  ├─ roads                          ├─ network (wrapped Network)
  ├─ lanes                          ├─ direction
  ├─ intersections                  ├─ pitLane (new)
  └─ regions                        ├─ sectors (new)
                                    └─ startingGrid (new)

Regions                           Extended Regions
  ├─ road                           ├─ road (inherited)
  ├─ sidewalk                       ├─ racingLine = road - pitLane
  ├─ intersection                   ├─ mainRacingRoad (new)
  └─ roadOrShoulder                 └─ pitLaneRoad (new)

Objects                           Extended Objects
  ├─ DrivingObject                  ├─ DrivingObject (inherited)
  ├─ Vehicle                        ├─ Vehicle (inherited)
  └─ Car                            └─ RacingCar(Car) ← extends Car
                                         ├─ raceNumber
                                         ├─ fuelLevel
                                         └─ tireWear

Actions                           Extended Actions
  ├─ SetThrottle                    ├─ SetThrottle (inherited)
  ├─ SetSteer                       ├─ SetSteer (inherited)
  ├─ SetBrake                       ├─ SetBrake (inherited)
  └─ RegulatedControl               ├─ DRSAction (new)
                                    ├─ ERSDeployAction (new)
                                    └─ PitLimiterAction (new)

Behaviors                         Extended Behaviors
  ├─ FollowLaneBehavior             ├─ FollowLaneBehavior (inherited)
  ├─ TurnBehavior                   ├─ FollowRacingLineBehavior (new)
  └─ LaneChangeBehavior             ├─ PitStopBehavior (new)
                                    └─ OvertakingBehavior (new)
```

---

## File-by-File Responsibilities

### `__init__.py` - The Domain's "README"
- **What**: Documentation of domain purpose and usage
- **Contains**: Domain description, example usage, supported simulators
- **Pattern**: Docstring only, no code

### `model.scenic` - The Domain's World Model
- **What**: Main entry point for scenarios
- **Contains**: 
  - Parameter definitions
  - Network/infrastructure setup
  - Region definitions
  - Object class definitions
  - Utility functions
- **Pattern**: 
  - Import from parent domain (if extending)
  - Load/create infrastructure
  - Define semantic regions
  - Define object classes
  - Provide utility functions

### `roads.py` / `tracks.py` - Infrastructure Layer
- **What**: Road network and track representation
- **Contains**:
  - Network/Track classes
  - Road, Lane, Intersection classes (driving)
  - PitLane, Sector, RacingLine classes (racing)
  - Geometry and topology
- **Pattern**:
  - Load from files (OpenDRIVE, etc.)
  - Provide spatial queries (laneAt, roadAt)
  - Cache for performance

### `actions.py` - What Agents Can Do
- **What**: Atomic actions agents can perform
- **Contains**:
  - Protocol classes (mixins like `Steers`, `Walks`)
  - Action classes (SetThrottle, SetSteer, etc.)
- **Pattern**:
  - Define protocols for capabilities
  - Define actions that use protocols
  - Simulators implement protocol methods

### `behaviors.scenic` - How Agents Act Over Time
- **What**: Time-extended strategies using actions
- **Contains**:
  - Behavior definitions
  - Control loops
  - Strategy implementations
- **Pattern**:
  - Use actions from `actions.py`
  - Use `do`, `take`, `wait`, `interrupt`
  - Compose behaviors from simpler ones

### `controllers.py` - Control Algorithms
- **What**: PID and other controllers for behaviors
- **Contains**:
  - Lateral controllers (steering)
  - Longitudinal controllers (speed)
- **Pattern**:
  - Implement control algorithms
  - Used by behaviors
  - Tunable parameters

### `workspace.py` - Visualization
- **What**: How scenarios are displayed
- **Contains**:
  - Workspace class
  - 2D/3D rendering methods
- **Pattern**:
  - Extend core Workspace
  - Implement show2D/show3D
  - Set zoom/view parameters

### `simulators.py` - Simulator Interface
- **What**: Base class for domain simulators
- **Contains**:
  - Abstract simulator interface
  - Required method signatures
- **Pattern**:
  - Extend core Simulator
  - Define domain-specific methods
  - Actual simulators implement these

---

## Import Patterns

### In `.scenic` Files (model.scenic, behaviors.scenic)

```scenic
# ✅ Import everything from parent domain
from scenic.domains.driving.model import *

# ✅ Import domain-specific modules
from scenic.domains.racing.tracks import RacingTrack
from scenic.domains.racing.actions import *

# ✅ Import specific items from core
from scenic.core.distributions import RejectionException
from scenic.simulators.utils.colors import Color
```

### In `.py` Files (tracks.py, actions.py)

```python
# ✅ Import specific items from parent domain
from scenic.domains.driving.roads import Network, Road, Lane

# ❌ Don't import * in Python files
# from scenic.domains.driving.roads import *  # Too broad

# ✅ Import from core as needed
from scenic.core.regions import PolygonalRegion, PolylineRegion
from scenic.core.vectors import Vector
```

---

## The "Network Wrapping" Pattern

This is a **key architectural pattern** used by racing domain:

```python
# In RacingTrack class:
class RacingTrack:
    def __init__(self, network: Network, ...):
        self.network = network  # ← Wrap the driving domain's Network
        # Add racing-specific features
        self._identifyRacingFeatures()

# In model.scenic:
# 1. Create Network from driving domain
network: Network = Network.fromFile(globalParameters.map, ...)

# 2. Wrap it in RacingTrack
track: RacingTrack = RacingTrack(network, ...)

# 3. Replace network reference with wrapped version
network = track.network  # Still a Network object!

# Result: All driving domain features still work!
# network.roadAt(), network.laneAt(), etc. all still function
# But now we also have track.pitLane, track.sectors, etc.
```

**Why this works**:
- Racing domain doesn't replace the Network, it **wraps** it
- All driving domain code that uses `network` still works
- Racing-specific features are **additions**, not replacements

---

## The "Region Derivation" Pattern

Don't create separate regions; derive from existing ones:

```scenic
# ✅ GOOD: Derive from driving regions
racingLine: Region = road.difference(pitLane)
# ^ Uses 'road' from driving domain
#   Subtracts racing-specific 'pitLane'
#   Result: Compatible with all driving features

# ❌ BAD: Create completely separate region
racingLine: Region = track.createCustomRacingRegion()
# ^ Disconnected from 'road'
#   Driving domain features might not work
```

**Key insight**: Racing regions are **derived from** driving regions, maintaining compatibility.

---

## The "Class Extension" Pattern

Always extend, never replace:

```scenic
# In driving/model.scenic:
class Car(Vehicle):
    """A car"""
    width: 2
    length: 4.5
    color: Color.defaultCarColor()

# In racing/model.scenic:
# ✅ GOOD: Extend Car
class RacingCar(Car):
    """A racing car (extends Car)"""
    speed: 25  # Override default
    raceNumber: Range(1, 999)  # Add new property
    fuelLevel: Range(0.5, 1.0)  # Add new property

# ❌ BAD: Create from scratch
class RacingVehicle(DrivingObject):
    """A racing vehicle (duplicate of Car)"""
    width: 2
    length: 4.5
    # ... duplicated code
```

**Why this matters**:
- `RacingCar` **is a** `Car` (inheritance)
- All code expecting a `Car` will accept a `RacingCar`
- All `Car` properties and methods are inherited
- Only add what's **new** for racing

---

## The "Behavior Composition" Pattern

Build complex behaviors from simpler ones:

```scenic
# ✅ GOOD: Compose behaviors
behavior RaceWithPitStopBehavior(pitLap=3):
    """Race with planned pit stop"""
    
    for lap in range(1, pitLap):
        do FollowRacingLineBehavior()  # Reuse!
    
    do ExecutePitStopBehavior()  # Racing-specific
    
    do FollowRacingLineBehavior()  # Back to racing

# ❌ BAD: Duplicate code
behavior RaceWithPitStopBehavior(pitLap=3):
    """Race with pit stop (duplicated code)"""
    
    # [200 lines of duplicated lane-following code]
    # [50 lines of duplicated pit stop code]
    # [200 more lines of duplicated code]
```

---

## Quick Decision Tree

### "Where does this belong?"

```
Is it about road/track geometry and topology?
  YES → roads.py / tracks.py
  NO  → Continue...

Is it a single action an agent can take?
  YES → actions.py
  NO  → Continue...

Is it a time-extended strategy/behavior?
  YES → behaviors.scenic
  NO  → Continue...

Is it a control algorithm?
  YES → controllers.py
  NO  → Continue...

Is it an object class or region?
  YES → model.scenic
  NO  → Continue...

Is it about visualization?
  YES → workspace.py
  NO  → Continue...

Is it about simulator interface?
  YES → simulators.py
  NO  → Might belong in core Scenic
```

---

## Common Mistakes to Avoid

1. **❌ Replacing instead of extending**
   - Don't create a separate network; wrap the driving network
   
2. **❌ Duplicating instead of reusing**
   - Don't copy-paste FollowLaneBehavior; extend or compose it

3. **❌ Breaking compatibility**
   - Don't remove driving domain features; add racing features alongside

4. **❌ Creating disconnected regions**
   - Don't create racingLine from scratch; derive from road

5. **❌ Ignoring the protocol pattern**
   - Don't skip protocol classes; they enable simulator independence

6. **❌ Mixing concerns**
   - Don't put actions in model.scenic; put them in actions.py
   - Don't put road geometry in model.scenic; put it in tracks.py

---

## Summary Checklist

When creating or extending a domain:

- [ ] Follow the file structure (model.scenic, roads.py, actions.py, behaviors.scenic)
- [ ] Import everything from parent domain (`from parent.model import *`)
- [ ] Wrap, don't replace infrastructure (Network → RacingTrack wraps Network)
- [ ] Derive regions from parent regions (racingLine = road - pitLane)
- [ ] Extend classes, don't duplicate (RacingCar extends Car)
- [ ] Compose behaviors, don't copy-paste
- [ ] Use protocol classes for capabilities (Steers, Walks)
- [ ] Maintain simulator independence in domain layer
- [ ] Document the extension relationship clearly
- [ ] Test with multiple simulators

---

**Remember**: The power of Scenic domains comes from **layered composition**, not duplication!

