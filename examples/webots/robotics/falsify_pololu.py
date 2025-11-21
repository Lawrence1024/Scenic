#!/usr/bin/env python3
"""
Falsification example for Webots robotics scenarios.

This script demonstrates how to run VerifAI falsification on a Scenic
Webots robotics scenario to find parameter values that cause specification violations.

Usage:
    python falsify_pololu.py [--scenario PATH] [--iterations N] [--sampler TYPE]
"""

import argparse
import sys
import os

# Add Scenic to path if needed
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../../..'))

import verifai
from verifai.samplers.scenic_sampler import ScenicSampler
from verifai.falsifier import mtl_falsifier, generic_falsifier
from verifai.scenic_server import ScenicServer
from verifai.monitor import specification_monitor
from dotmap import DotMap


class WorkspaceMonitor(specification_monitor):
    """
    Custom monitor that checks if robot stays within workspace bounds.
    
    Returns robustness value:
    - Positive: robot stays in bounds (satisfied)
    - Negative: robot leaves bounds (violation)
    """
    
    def __init__(self, specification):
        super().__init__(specification)
        self.workspace_bounds = 2.25  # Half of 4.5m workspace
    
    def evaluate(self, result):
        """
        Evaluate simulation result.
        
        Args:
            result: SimulationResult from Scenic simulation
            
        Returns:
            float: Robustness value (negative = violation, positive = satisfied)
        """
        if result is None:
            return -1.0  # Rejected simulation
        
        # Extract trajectory from simulation result
        # The exact structure depends on the simulator implementation
        try:
            trajectory = result.trajectory
        except AttributeError:
            # If trajectory not available, try to get final state
            try:
                # Some simulators may expose final state differently
                final_state = result.finalState
                if final_state:
                    trajectory = [final_state]
                else:
                    return 0.0  # Can't evaluate
            except AttributeError:
                print("Warning: Could not access trajectory or finalState from result")
                print(f"Result type: {type(result)}")
                print(f"Result attributes: {dir(result)}")
                return 0.0
        
        if not trajectory:
            return 0.0
        
        # Check each state in trajectory
        min_robustness = float('inf')
        for state in trajectory:
            # Try to get robot position from state
            # This depends on how your simulator structures the state
            robot_x = 0.0
            robot_y = 0.0
            
            # Method 1: State is a dict
            if isinstance(state, dict):
                robot_x = state.get('robot_x', state.get('x', 0.0))
                robot_y = state.get('robot_y', state.get('y', 0.0))
            
            # Method 2: State has objects list
            elif hasattr(state, 'objects') and len(state.objects) > 0:
                robot = state.objects[0]
                if hasattr(robot, 'position'):
                    pos = robot.position
                    robot_x = pos[0] if len(pos) > 0 else 0.0
                    robot_y = pos[1] if len(pos) > 1 else 0.0
            
            # Method 3: State is a tuple/list
            elif isinstance(state, (tuple, list)) and len(state) > 0:
                if hasattr(state[0], 'position'):
                    pos = state[0].position
                    robot_x = pos[0] if len(pos) > 0 else 0.0
                    robot_y = pos[1] if len(pos) > 1 else 0.0
            
            # Calculate distance to workspace boundary
            # Robustness = minimum distance to any boundary
            dist_x = self.workspace_bounds - abs(robot_x)
            dist_y = self.workspace_bounds - abs(robot_y)
            robustness = min(dist_x, dist_y)
            
            min_robustness = min(min_robustness, robustness)
        
        return min_robustness


def create_falsifier(scenario_path, sampler_type='ce', n_iters=100, verbosity=1):
    """
    Create and configure a falsifier for Webots robotics scenario.
    
    Args:
        scenario_path: Path to .scenic scenario file
        sampler_type: VerifAI sampler type ('random', 'halton', 'ce', 'mab')
        n_iters: Number of falsification iterations
        verbosity: Verbosity level (0-3)
    
    Returns:
        Configured falsifier instance
    """
    
    # Load Scenic scenario
    print(f"Loading scenario from {scenario_path}...")
    if not os.path.exists(scenario_path):
        raise FileNotFoundError(f"Scenario file not found: {scenario_path}")
    
    sampler = ScenicSampler.fromScenario(
        scenario_path,
        maxIterations=2000,
    )
    
    # Define specification
    # Option 1: Use MTL specification (simple predicates only)
    # Note: MTL doesn't support function calls like abs(), so for complex properties
    # we use a custom monitor instead
    use_mtl = False  # Set to True to use MTL, False for custom monitor
    
    if use_mtl:
        # Simple MTL specification (no function calls)
        specification = ["G (robot_in_bounds)"]  # Would need predicate defined
        monitor = None
    else:
        # Use custom monitor for complex properties
        specification = None
        monitor = WorkspaceMonitor(None)
    
    # Falsifier parameters
    falsifier_params = DotMap({
        'n_iters': n_iters,
        'verbosity': verbosity,
        'save_error_table': True,
        'save_safe_table': True,
        'error_table_path': 'webots_robotics_errors.csv',
        'safe_table_path': 'webots_robotics_safe.csv',
        'fal_thres': 0.0,  # Threshold for violation
        'ce_num_max': 10,  # Max counterexamples to find
    })
    
    # Sampler parameters (for cross-entropy)
    if sampler_type == 'ce':
        sampler_params = DotMap({
            'alpha': 0.9,      # Learning rate
            'thres': 0.0,      # Threshold
            'init_num': 5,      # Initial samples
        })
        falsifier_params.sampler_params = sampler_params
    
    # Server options (for ScenicServer)
    server_options = DotMap({
        'maxSteps': None,          # Use scenario's time limit
        'verbosity': verbosity,
        'maxIterations': 2000,      # Max rejection sampling iterations
        'simulator': None,          # Use scenario's default simulator
    })
    
    # Create falsifier
    print(f"Creating falsifier with {sampler_type} sampler...")
    if specification is not None:
        falsifier = mtl_falsifier(
            specification=specification,
            sampler=sampler,
            sampler_type=sampler_type,
            falsifier_params=falsifier_params,
            server_options=server_options,
            server_class=ScenicServer
        )
    else:
        # Use generic_falsifier with custom monitor
        falsifier = generic_falsifier(
            monitor=monitor,
            sampler=sampler,
            sampler_type=sampler_type,
            falsifier_params=falsifier_params,
            server_options=server_options,
            server_class=ScenicServer
        )
    
    return falsifier


def main():
    parser = argparse.ArgumentParser(
        description='Run falsification on Webots robotics scenario'
    )
    parser.add_argument(
        '--scenario',
        type=str,
        default='pololu.scenic',
        help='Path to Scenic scenario file (default: pololu.scenic)'
    )
    parser.add_argument(
        '--iterations',
        type=int,
        default=50,
        help='Number of falsification iterations (default: 50)'
    )
    parser.add_argument(
        '--sampler',
        type=str,
        choices=['random', 'halton', 'ce', 'mab'],
        default='ce',
        help='Sampler type (default: ce)'
    )
    parser.add_argument(
        '--verbosity',
        type=int,
        default=1,
        choices=[0, 1, 2, 3],
        help='Verbosity level (default: 1)'
    )
    
    args = parser.parse_args()
    
    # Resolve scenario path
    scenario_path = args.scenario
    if not os.path.isabs(scenario_path):
        # Assume relative to script directory
        script_dir = os.path.dirname(os.path.abspath(__file__))
        scenario_path = os.path.join(script_dir, scenario_path)
    
    print("="*60)
    print("Webots Robotics Falsification")
    print("="*60)
    print(f"Scenario: {scenario_path}")
    print(f"Sampler: {args.sampler}")
    print(f"Iterations: {args.iterations}")
    print("="*60)
    
    try:
        # Create falsifier
        falsifier = create_falsifier(
            scenario_path,
            sampler_type=args.sampler,
            n_iters=args.iterations,
            verbosity=args.verbosity
        )
        
        # Run falsification
        print("\nRunning falsification...")
        falsifier.run_falsifier()
        
        # Print results
        print("\n" + "="*60)
        print("FALSIFICATION RESULTS")
        print("="*60)
        
        if hasattr(falsifier, 'error_table') and falsifier.error_table:
            num_errors = len(falsifier.error_table.table)
            print(f"\nCounterexamples found: {num_errors}")
            
            if num_errors > 0:
                print("\nError Table (violations):")
                print(falsifier.error_table.table)
                
                if falsifier.save_error_table:
                    print(f"\nError table saved to: {falsifier.error_table_path}")
            else:
                print("\nNo violations found!")
        
        if hasattr(falsifier, 'safe_table') and falsifier.safe_table:
            num_safe = len(falsifier.safe_table.table)
            print(f"\nSafe samples: {num_safe}")
            if args.verbosity >= 2 and num_safe > 0:
                print("\nSafe Table (first 10):")
                print(falsifier.safe_table.table.head(10))
        
        # Analyze error table if violations found
        if (hasattr(falsifier, 'error_table') and 
            falsifier.error_table and 
            len(falsifier.error_table.table) > 0):
            print("\nAnalyzing counterexamples...")
            analysis_params = DotMap({
                'k_closest_params': DotMap({'k': 5}),
                'random_params': DotMap({'count': 5}),
            })
            try:
                falsifier.analyze_error_table(analysis_params=analysis_params)
            except Exception as e:
                print(f"Warning: Error analysis failed: {e}")
        
        print("\n" + "="*60)
        print("Falsification complete!")
        print("="*60)
        
    except KeyboardInterrupt:
        print("\n\nFalsification interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\nError during falsification: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()

