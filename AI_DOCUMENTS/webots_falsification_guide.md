# Falsification Guide for Webots Robotics Integration

This document explains how to run VerifAI falsification algorithms on Scenic scenarios using the Webots robotics simulator integration.

## Quick Start

### How Webots Robotics Integration Works

1. **Scenic Scenario** defines robot behaviors using high-level actions
2. **Webots Robotics Model** bridges Scenic's generic robotics domain with Webots
3. **Webots Simulator** executes physics simulation
4. **Communication** happens via Emitter/Receiver devices sending JSON motor commands

**Key flow:** Scenic behaviors → Actions → Robot methods → Webots motor commands → Physics simulation

### Quick Steps to Run Falsification

1. **Prepare your scenario** with `VerifaiRange` parameters
2. **Create a falsification script** (see example below)
3. **Run the script** from the command line

```bash
cd Scenic/examples/webots/robotics
python falsify_pololu.py --scenario pololu_falsify.scenic --iterations 100
```

### What Happens During Falsification

1. **VerifAI generates parameter values** using the chosen sampler (e.g., cross-entropy)
2. **Scenic creates scenes** with those parameter values
3. **Webots runs simulations** of robot behavior
4. **Monitor evaluates** each simulation against your specification
5. **Falsifier learns** which parameter values lead to violations
6. **Counterexamples saved** to error table CSV file

## Overview

The Webots robotics integration allows Scenic to control robots in Webots simulations. VerifAI falsification can systematically search for parameter values that cause specification violations (bugs) in your robot behaviors.

## Architecture Summary

The integration works in layers:

1. **Scenic Scenario** (`.scenic` file) - Defines robot behaviors and initial conditions
2. **Webots Robotics Model** - Bridges Scenic's generic robotics domain with Webots-specific integration
3. **Webots Simulator** - Physics engine that executes robot actions
4. **VerifAI Falsifier** - Systematically searches parameter space to find violations

## Key Components for Falsification

### 1. Scenic Scenario with External Parameters

To enable falsification, you need to mark parameters that should be varied by VerifAI using `VerifaiRange`:

```scenic
model scenic.simulators.webots.robotics_model
from scenic.domains.robotics.behaviors import SquareTrackBehavior
from scenic.core.external_params import VerifaiRange

# Set VerifAI sampler type for falsification
param verifaiSamplerType = 'ce'

# Use VerifaiRange for parameters you want to falsify
param ROBOT_X = VerifaiRange(-2.0, 2.0)        # Initial X position
param ROBOT_Y = VerifaiRange(-2.0, 2.0)        # Initial Y position  
param FORWARD_SPEED = VerifaiRange(50, 100)    # Robot forward speed (0-100)
param TURN_SPEED = VerifaiRange(40, 80)        # Robot turn speed (0-100)

workspace_region = RectangularRegion((0, 0, 0.016), 0, 4.5, 4.5)
workspace = Workspace(workspace_region)

# Create robot with falsifiable parameters
# Note: Must assign to 'ego' for VerifAI compatibility
ego = new WebotsPololuRobot at (globalParameters.ROBOT_X, globalParameters.ROBOT_Y, 0.016), 
    with behavior SquareTrackBehavior(
        forwardSpeed=globalParameters.FORWARD_SPEED, 
        turnSpeed=globalParameters.TURN_SPEED, 
        headingOffset=-90 deg
    )

terminate after 120 seconds
```

### 2. Specification Monitor

You need to define what constitutes a violation. VerifAI uses monitors that evaluate simulation results and return a "robustness" value (rho):
- **rho ≤ 0**: Specification violated (counterexample found)
- **rho > 0**: Specification satisfied

You can use:
- **MTL (Metric Temporal Logic)** specifications for temporal properties
- **Custom monitors** for complex properties

### 3. Falsifier Setup

The falsifier needs:
- A `ScenicSampler` to generate scenes from your scenario
- A monitor to evaluate specifications
- Falsifier parameters (iterations, sampler type, etc.)

## Key Components

- **ScenicSampler**: Generates scenes from your scenario
- **ScenicServer**: Runs Scenic simulations (connects to Webots)
- **mtl_falsifier** / **generic_falsifier**: Orchestrates the falsification process
- **Monitor**: Evaluates specifications (MTL or custom)

## Complete Example: Running Falsification

Here's a complete Python script to run falsification on a Webots robotics scenario:

```python
#!/usr/bin/env python3
"""
Example falsification script for Webots robotics scenarios.
"""

import verifai
from verifai.samplers.scenic_sampler import ScenicSampler
from verifai.falsifier import generic_falsifier
from verifai.scenic_server import ScenicServer
from verifai.monitor import specification_monitor
from dotmap import DotMap
import os

# Path to your Scenic scenario file
SCENARIO_PATH = "examples/webots/robotics/pololu_falsify.scenic"

# Custom monitor for complex properties
class WorkspaceMonitor(specification_monitor):
    def __init__(self, specification):
        super().__init__(specification)
        self.workspace_bounds = 2.25  # Half of 4.5m workspace
    
    def evaluate(self, result):
        """Evaluate simulation result. Returns robustness value."""
        if result is None:
            return -1.0  # Rejected simulation
        
        # Extract trajectory from simulation result
        try:
            trajectory = result.trajectory
        except AttributeError:
            return 0.0
        
        if not trajectory:
            return 0.0
        
        # Check each state in trajectory
        min_robustness = float('inf')
        for state in trajectory:
            # Extract robot position (adjust based on your result structure)
            robot_x = 0.0
            robot_y = 0.0
            
            if isinstance(state, dict):
                robot_x = state.get('robot_x', state.get('x', 0.0))
                robot_y = state.get('robot_y', state.get('y', 0.0))
            elif hasattr(state, 'objects') and len(state.objects) > 0:
                robot = state.objects[0]
                if hasattr(robot, 'position'):
                    pos = robot.position
                    robot_x = pos[0] if len(pos) > 0 else 0.0
                    robot_y = pos[1] if len(pos) > 1 else 0.0
            
            # Calculate distance to workspace boundary
            dist_x = self.workspace_bounds - abs(robot_x)
            dist_y = self.workspace_bounds - abs(robot_y)
            robustness = min(dist_x, dist_y)
            min_robustness = min(min_robustness, robustness)
        
        return min_robustness

# Falsifier parameters
falsifier_params = DotMap({
    'n_iters': 100,              # Number of iterations to run
    'verbosity': 1,              # Verbosity level (0-3)
    'save_error_table': True,    # Save counterexamples
    'save_safe_table': True,     # Save safe examples
    'error_table_path': 'error_table.csv',
    'safe_table_path': 'safe_table.csv',
    'fal_thres': 0.0,            # Threshold for violation (rho <= this)
    'ce_num_max': 10,            # Maximum number of counterexamples to find
})

# Sampler parameters (for cross-entropy method)
sampler_params = DotMap({
    'alpha': 0.9,                # Learning rate
    'thres': 0.0,                # Threshold for CE
    'init_num': 5,                # Initial samples for CE
})

falsifier_params.sampler_params = sampler_params

# Server options (passed to ScenicServer)
server_options = DotMap({
    'maxSteps': None,            # Max simulation steps (None = use scenario limit)
    'verbosity': 1,              # Scenic verbosity
    'maxIterations': 2000,       # Max rejection sampling iterations
    'simulator': None,           # Use scenario's default simulator
})

# Create Scenic sampler from scenario
print(f"Loading scenario from {SCENARIO_PATH}...")
sampler = ScenicSampler.fromScenario(
    SCENARIO_PATH,
    maxIterations=2000,
)

# Choose sampler type for VerifAI
# Options: 'random', 'halton', 'ce' (cross-entropy), 'mab' (multi-armed bandit)
SAMPLER_TYPE = 'ce'  # Cross-entropy for active learning

# Create monitor
monitor = WorkspaceMonitor(None)

# Create falsifier
print("Creating falsifier...")
falsifier = generic_falsifier(
    monitor=monitor,
    sampler=sampler,
    sampler_type=SAMPLER_TYPE,
    falsifier_params=falsifier_params,
    server_options=server_options,
    server_class=ScenicServer
)

# Run falsification
print("Running falsification...")
try:
    falsifier.run_falsifier()
except KeyboardInterrupt:
    print("\nFalsification interrupted by user")
except Exception as e:
    print(f"Error during falsification: {e}")
    raise

# Print results
print("\n" + "="*60)
print("FALSIFICATION RESULTS")
print("="*60)

if hasattr(falsifier, 'error_table') and falsifier.error_table:
    print(f"\nCounterexamples found: {len(falsifier.error_table.table)}")
    print("\nError Table (violations):")
    print(falsifier.error_table.table)
    
    if falsifier.save_error_table:
        print(f"\nError table saved to: {falsifier.error_table_path}")

if hasattr(falsifier, 'safe_table') and falsifier.safe_table:
    print(f"\nSafe samples: {len(falsifier.safe_table.table)}")
    if falsifier.verbosity >= 2:
        print("\nSafe Table:")
        print(falsifier.safe_table.table)

# Analyze error table (optional)
if hasattr(falsifier, 'error_table') and len(falsifier.error_table.table) > 0:
    print("\nAnalyzing counterexamples...")
    analysis_params = DotMap({
        'k_closest_params': DotMap({'k': 5}),
        'random_params': DotMap({'count': 5}),
    })
    falsifier.analyze_error_table(analysis_params=analysis_params)

print("\nFalsification complete!")
```

## Understanding the Specification

The specification defines what you're testing for. Common patterns:

### MTL Specifications

**Note:** MTL doesn't support function calls like `abs()`. For complex properties, use custom monitors instead.

```python
# Always (robot stays in bounds) - requires predicate definition
specification = ["G (robot_in_bounds)"]

# Eventually (robot reaches goal)
specification = ["F (distance_to_goal < 0.5)"]

# Always (if obstacle detected, then avoid)
specification = ["G (obstacle_detected -> avoid_action)"]

# Multiple specifications (all must hold)
specification = [
    "G (robot_in_bounds)",
    "G (robot_speed < 100)"
]
```

### Custom Monitors

For complex properties, create a custom monitor:

```python
class CollisionMonitor(verifai.monitor.specification_monitor):
    def evaluate(self, result):
        if result is None:
            return -1.0
        
        # Check for collisions in trajectory
        for state in result.trajectory:
            if state.get('collision', False):
                return -1.0  # Violation
        
        return 1.0  # No collision

monitor = CollisionMonitor(None)
```

### Example Specifications

```python
# Stay in bounds (using custom monitor)
# See WorkspaceMonitor example above

# Reach goal
specification = ["F (distance_to_goal < 0.5)"]

# Avoid collisions
specification = ["G (!collision)"]

# Multiple requirements
specification = [
    "G (robot_in_bounds)",
    "G (robot_speed < 100)"
]
```

## Sampler Types

VerifAI supports different sampling strategies:

1. **'random'** - Uniform random sampling
2. **'halton'** - Quasi-random Halton sequence (better coverage)
3. **'ce'** - Cross-entropy method (active learning, converges to violations)
4. **'mab'** - Multi-armed bandit (explores promising regions)

For falsification, **'ce'** (cross-entropy) is typically best as it actively learns which parameter values lead to violations.

## Accessing Simulation Results

To create effective monitors, you need to understand what data is available in the simulation result. The structure depends on your simulator, but typically:

```python
def evaluate(self, result):
    # result is a SimulationResult object
    trajectory = result.trajectory  # List of states over time
    
    for state in trajectory:
        # Access object properties
        robot = state.objects[0]  # First object (usually ego/robot)
        position = robot.position
        velocity = robot.velocity
        # ... other properties
        
    return robustness_value
```

For Webots robotics, you may need to check what properties are exposed in the simulation result. You can inspect this by:

```python
# In your monitor
def evaluate(self, result):
    print(f"Result type: {type(result)}")
    print(f"Result attributes: {dir(result)}")
    if hasattr(result, 'trajectory'):
        print(f"Trajectory length: {len(result.trajectory)}")
        if len(result.trajectory) > 0:
            print(f"First state: {result.trajectory[0]}")
    return 0.0
```

## Running the Falsifier

1. **Save your scenario** with `VerifaiRange` parameters
2. **Create a falsification script** (like the example above, or use `examples/webots/robotics/falsify_pololu.py`)
3. **Run the script**:
   ```bash
   python falsify_pololu.py --scenario pololu_falsify.scenic --iterations 100
   ```

The falsifier will:
- Generate scenes from your scenario
- Run simulations in Webots
- Evaluate each simulation against your specification
- Learn which parameter values cause violations (if using 'ce' sampler)
- Save counterexamples to error_table.csv

## Important Limitation: Webots Runtime Requirement

**⚠️ CRITICAL:** The Webots robotics integration is designed to run **from inside Webots** via the supervisor controller, not directly from Python scripts. This means:

- The falsification script cannot directly run Webots scenarios outside of Webots
- You need Webots to be running (either GUI or headless mode)
- The scenario must be loaded by Webots' supervisor controller

### Workarounds

1. **Use Webots in Headless Mode** (if available):
   ```bash
   webots --mode=fast --no-rendering --batch your_world.wbt
   ```
   Then connect the falsifier to Webots via the supervisor controller.

2. **Modify the Scenario for Standalone Testing**:
   - Create a version that doesn't require Webots runtime
   - Use a different simulator for falsification testing
   - Test behaviors in isolation

3. **Run Falsification from Within Webots**:
   - Integrate the falsification logic into the supervisor controller
   - Run multiple simulations sequentially within Webots
   - This requires significant modification to the current architecture

## Tips and Best Practices

1. **Start with simple specifications** - Test basic properties first
2. **Use appropriate parameter ranges** - `VerifaiRange` should cover the space you want to test
3. **Monitor verbosity** - Set `verbosity=1` or `2` to see what's happening
4. **Limit iterations initially** - Start with `n_iters=50` to test your setup
5. **Check Webots is accessible** - Ensure Webots can be launched (may need GUI or headless mode)
6. **Handle long simulations** - Use `maxSteps` to limit simulation length
7. **Save intermediate results** - Error tables help analyze what went wrong
8. **Use custom monitors for complex properties** - MTL doesn't support function calls

## Troubleshooting

### "scenario must be run from inside Webots"
- This is expected for Webots robotics scenarios
- You cannot run these scenarios directly from Python
- See "Important Limitation" section above for workarounds

### "Failed to create simulation"
- Check your `.wbt` world file path is correct
- Verify Webots can load the world file
- Check robot controllers are available
- Ensure Webots is running if trying to connect

### "No violations found"
- Your specification might be too lenient
- Try more iterations (`n_iters`)
- Try different sampler types
- Check that your monitor is correctly evaluating results

### "Simulation takes too long"
- Reduce `maxSteps` in server_options
- Reduce simulation time in scenario (`terminate after X seconds`)
- Use headless Webots mode if available

### "Can't access robot properties in monitor"
- Inspect the result structure (see "Accessing Simulation Results" above)
- Check what properties are available in `SimulationResult`
- You may need to modify the simulator to expose needed properties

### "MTL parse error" or "Rule didn't match"
- MTL doesn't support function calls like `abs()`
- Use custom monitors for complex properties
- Keep MTL specifications simple with basic predicates

## Advanced: Custom Server

If you need more control, you can create a custom server:

```python
from verifai.scenic_server import ScenicServer

class WebotsRoboticsServer(ScenicServer):
    def _simulate(self, scene):
        # Custom simulation logic
        result = super()._simulate(scene)
        # Post-process result if needed
        return result

# Use in falsifier
falsifier = generic_falsifier(
    monitor=monitor,
    ...,
    server_class=WebotsRoboticsServer
)
```

## Files and Examples

- **`falsify_pololu.py`** - Working example script in `examples/webots/robotics/`
- **`pololu_falsify.scenic`** - Example scenario with VerifaiRange parameters
- **This guide** - Complete documentation

## Next Steps

1. Review the example script: `examples/webots/robotics/falsify_pololu.py`
2. Modify the scenario and specification for your use case
3. Run falsification and analyze the error table to understand what causes violations
4. Consider the Webots runtime limitation and choose an appropriate workaround

## References

- VerifAI Documentation: https://verifai.readthedocs.io
- Scenic Documentation: https://scenic-lang.readthedocs.io
- MTL Syntax: See VerifAI's monitor documentation
- Webots Robotics Integration: See `webots_robotics_integration.md` for architecture details
