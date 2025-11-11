# dSPACE Simulation Loop Flow

## Complete Timestep Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                         SETUP PHASE                              │
│                                                                  │
│  1. ModelDesk: save/activate scenario copy                      │
│  2. Geometry: build road index + coord transform (geometry/pipeline.py) │
│  3. Place vehicles (modeldesk/placement.py)                     │
│  4. ControlDesk: connect/start measurement (controldesk/session.py) │
│  5. ⏸  PAUSE simulation (controldesk/session.pause)            │
│  6. Initialize VesiInterface control (controldesk/session.connect_and_prepare) │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                    SIMULATION LOOP (each timestep)              │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
        ┌───────────────────────────────────────┐
        │  1. BEHAVIORS compute actions         │
        │     - SetThrottleAction               │
        │     - SetBrakeAction                  │
        │     - SetSteerAction                  │
        │     - SetGearAction (one-shot)        │
        └───────────────────────────────────────┘
                            │
                            ▼
        ┌───────────────────────────────────────┐
        │  2. executeActions()                  │
        │     - Warm-up: ensure fellow arrays (controldesk/arrays.py) │
        │     - Write to ControlDesk:           │
        │       • Throttle → VesiInterface      │
        │       • Brake → VesiInterface         │
        │       • Steering → VesiInterface      │
        │       • Gear → VesiInterface          │
        │     - Clear control state             │
        └───────────────────────────────────────┘
                            │
                            ▼
        ┌───────────────────────────────────────┐
        │  3. step() ⚙️                         │
        │     - controldesk.session.step()      │
        │     - Physics advances Δt             │
        └───────────────────────────────────────┘
                            │
                            ▼
        ┌───────────────────────────────────────┐
        │  4. getProperties() 📖                │
        │     - For each vehicle:               │
        │       • controldesk.readback.read_*() │
        │         - x, y, z                     │
        │         - yaw_deg → yaw_rad           │
        │         - v, w → velocity vector      │
        │       • Update _backend               │
        │       • Return requested properties   │
        └───────────────────────────────────────┘
                            │
                            ▼
        ┌───────────────────────────────────────┐
        │  5. Scenic framework                  │
        │     - Updates obj.position            │
        │     - Updates obj.velocity            │
        │     - Updates obj.yaw                 │
        │     - Checks requirements             │
        │     - Runs monitors                   │
        └───────────────────────────────────────┘
                            │
                            ▼
                    [Loop continues]
```

## Key Data Flow

### Write Path (Control → ControlDesk)

```
Behavior Actions
      │
      ▼
obj._control_state / obj._oneshot_actions
      │
      ▼
executeActions()
      │
      ▼
setVehicleControl()
      │
      ▼
self._cd.set_var(path, value)
      │
      ▼
ControlDesk Variables
      │
      ▼
VesiInterface
      │
      ▼
dSPACE Simulation
```

### Read Path (ControlDesk → Scenic via readback)

```
dSPACE Simulation
      │
      ▼
ControlDesk Variables
      │
      ▼
controldesk.readback.read_ego_state(sim, obj)
controldesk.readback.read_fellow_state(sim, obj, dutils)
      │
      ▼
_readVehicleStateFromControlDesk()  # delegates to readback module
      │
      ▼
obj.dspaceActor (internal state)
      │
      ▼
getProperties()
      │
      ▼
Scenic obj.position, obj.velocity, obj.yaw
```

## Internal State (dspaceActor)

Each vehicle object has a `dspaceActor` attribute (defined in `DSPACERacingCar`):

```python
class DSpaceVehicleActor:
    scenic_obj: Object           # Parent Scenic object
    position: Vector(x, y, z)    # World coordinates (meters)
    linvel: Vector(vx, vy, vz)   # Linear velocity (m/s)
    angvel: Vector(wx, wy, wz)   # Angular velocity (rad/s)
    heading: float               # Yaw angle (radians)
    
    def set_control(self, control_dict):
        """Used by vehicle methods (setMaxSpeed, setTTL)"""
```

### dspaceActor Lifecycle

1. **Initialization**: `_initializeDSpaceActor(obj)`
   - Called in `createObjectInSimulator()`
   - Creates `DSpaceVehicleActor` instance
   - Sets initial position and heading
   - Integrates with `DSPACERacingCar.dspaceActor` attribute

2. **Update**: `_readVehicleStateFromControlDesk(obj)`
   - Called every timestep in `getProperties()`
   - Reads from ControlDesk
   - Converts units (deg→rad, speed→velocity vector)
   - Updates dspaceActor fields

3. **Usage**: `getProperties(obj, properties)`
   - Reads from dspaceActor
   - Returns requested properties
   - No direct ControlDesk access by caller

4. **Control Methods**: Vehicle control methods
   - `setMaxSpeed()`, `setTTL()` call `dspaceActor.set_control()`
   - Passes control parameters to simulator

## ControlDesk Variable Access

### Read Operations (getProperties, via readback)

```python
# Fellow vehicle (index 0-based)
x = self._cd.get_var("Platform()://.../FellowTrailer/x[0]")
y = self._cd.get_var("Platform()://.../FellowTrailer/y[0]")
yaw_deg = self._cd.get_var("Platform()://.../yaw_deg_out[0]")
v = self._cd.get_var("Platform()://.../v_Fellows[0]")
```

### Write Operations (executeActions → VehicleController)

```python
# VesiInterface control
self._cd.set_var("Platform()://.../Const_throttle_cmd/Value", 50.0)
self._cd.set_var("Platform()://.../Const_brake_cmd_front/Value", 0.0)
self._cd.set_var("Platform()://.../Const_steering_cmd/Value", -15.0)
```

## Error Handling Flow

```
step() called
    │
    ├─ ControlDesk available? ──No──> time.sleep(timestep)
    │                                  [Fallback mode]
    └─ Yes
        │
        └─ controldesk.session.step()
            │
            ├─ Success ──> Continue
            │
            └─ Exception ──> Print warning
                             time.sleep(timestep)
                             [Graceful degradation]

getProperties() called
    │
    ├─ _initializeVehicleBackend() ──> Ensure backend exists
    │
    ├─ ControlDesk available? ──No──> Use current backend state
    │                                  [Static positioning]
    └─ Yes
        │
        └─ _readVehicleStateFromControlDesk()
            │
            ├─ Success ──> Update backend, return properties
            │
            └─ Exception ──> Keep current backend state
                             [State persistence]
```

## Timing Diagram

```
Time:  0ms    10ms   20ms   30ms   40ms   50ms
       │      │      │      │      │      │
State: S0     S1     S2     S3     S4     S5
       │      │      │      │      │      │
       ├──────┼──────┼──────┼──────┼──────┤
       │      │      │      │      │      │
Write  W0     W1     W2     W3     W4     W5    (Controls)
       │      │      │      │      │      │
Step   │  →S1 │  →S2 │  →S3 │  →S4 │  →S5     (Physics)
       │      │      │      │      │      │
Read   R0     R1     R2     R3     R4     R5    (State)
       │      │      │      │      │      │
```

### Sequence Within Timestep

```
Timestep N:
├─ Read state from previous step (getProperties)
├─ Compute actions based on state
├─ Write controls (executeActions)
├─ Advance physics (step)
└─ [Timestep N+1 begins]
```

## Key Methods Summary

| Method | Purpose | When Called |
|--------|---------|-------------|
| `_pauseSimulation()` | Pause simulation for step control | Setup (once) |
| `_advanceSimulationStep()` | Execute one physics step | Every `step()` |
| `_initializeDSpaceActor()` | Create DSpaceVehicleActor instance | Object creation |
| `_readVehicleStateFromControlDesk()` | Read state from simulator | Every `getProperties()` |
| `_readEgoStateFromControlDesk()` | Read ego vehicle state | As needed |
| `_readFellowStateFromControlDesk()` | Read fellow vehicle state | As needed |
| `_getFellowIndex()` | Get fellow array index | As needed |

## Benefits of This Design

### ✅ Separation of Concerns
- Write path: `executeActions()` → ControlDesk
- Read path: ControlDesk → `getProperties()`
- Clear boundaries between control and sensing

### ✅ State Persistence
- `dspaceActor` maintains state between reads
- Graceful degradation if ControlDesk unavailable
- No data loss on temporary failures
- Integrates with vehicle model's control methods

### ✅ Encapsulation
- Helper methods hide COM complexity
- Clean public interface
- Easy to test and maintain

### ✅ Extensibility
- Easy to add new vehicle types
- Simple to extend state variables
- Flexible error handling

## Comparison: Before vs After

### Before ❌

```python
def step(self):
    time.sleep(self.timestep)  # Does nothing!

def getProperties(self, obj, properties):
    b = getattr(obj, "_backend", None)  # Always None!
    # Returns garbage values
```

### After ✅

```python
def step(self):
    if self._cd:
        self._advanceSimulationStep()  # Actually advances!
    else:
        time.sleep(self.timestep)

def getProperties(self, obj, properties):
    self._initializeDSpaceActor(obj)  # Uses obj.dspaceActor
    self._readVehicleStateFromControlDesk(obj)
    # Returns real values from simulation via dspaceActor
```

**Key Improvement**: Uses existing `dspaceActor` attribute from `DSPACERacingCar` class instead of creating separate `_backend`, providing better integration with the vehicle model architecture.

### 2025‑11: Warm‑Up Gating & External Signals Path

To handle initialization timing and model‑specific External Signals:

- The simulation now performs a short “warm‑up” phase at start. `executeActions` defers behavior execution until the plant’s `FellowMovement/FELLOW_POS_VEL/*` bulk arrays (e.g., `x[ ]/y[ ]/yaw_deg_out[ ]`) report non‑zero values, ensuring fellows are spawned and the plant is publishing state.
- Fellow longitudinal and lateral control are now applied via bulk updates to `Environment/Traffic/PlantModel/FellowMovement/External_Signals`:
  - `.../Const_v_Fellows_External[km|h]/Value[<idx>]` (km/h) and
  - `.../Const_d_Fellows_External[m]/Value[<idx>]`.
- The simulator performs a one‑time probe to determine the correct External Signals path (`km/h` vs `km|h`) and array base (0‑ vs 1‑based), writes to the proper index, and reads back the same element to verify the write.
- The fellow’s second segment is configured to `"Continue"` for both `LongitudinalType` and `LateralType`, and marked `Endless`, so external velocity/deviation drive motion without being overridden by a fixed profile.

## Testing Checklist

- [ ] Simulation advances correctly with ControlDesk
- [ ] Fallback works without ControlDesk
- [ ] Ego vehicle state reads correctly
- [ ] Fellow vehicles (multiple) read correctly
- [ ] Velocity vector computed correctly from speed + heading
- [ ] Heading converted correctly (deg → rad)
- [ ] dspaceActor persists state across timesteps
- [ ] dspaceActor integrates with setMaxSpeed/setTTL methods
- [ ] Error handling works gracefully
- [ ] No performance bottlenecks

## Related Documentation

- `STEP_AND_GETPROPERTIES_FIX.md` - Detailed implementation
- `DSPACE_CONTROL_INTERFACES.md` - ControlDesk variable paths
- `DSPACE_SIMULATOR_STRUCTURE.md` - Overall architecture
- `VEHICLE_CONTROL_IMPLEMENTATION.md` - Control flow details

