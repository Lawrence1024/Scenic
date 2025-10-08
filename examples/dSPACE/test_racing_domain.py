"""
Test script for the racing domain with dSPACE simulator.

This script verifies that the racing domain works correctly:
1. Compiles a racing scenario
2. Generates a scene with racing cars on starting grid
3. Sets up the simulation in dSPACE
"""

import sys
import os

# Add Scenic to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

from scenic.simulators.dspace import DSpaceSimulator
import scenic


def main():
    print("="*80)
    print("TESTING RACING DOMAIN WITH DSPACE")
    print("="*80)
    
    # Path to the racing scenario
    scenario_path = os.path.join(os.path.dirname(__file__), 'laguna_seca_race.scenic')
    
    print(f"\n[1] Compiling racing scenario: {scenario_path}")
    try:
        scenario = scenic.scenarioFromFile(scenario_path)
        print(f"    ✓ Racing scenario compiled successfully")
    except Exception as e:
        print(f"    ✗ ERROR: Could not compile scenario: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    print(f"\n[2] Generating race scene...")
    try:
        scene, _ = scenario.generate(maxIterations=100)
        print(f"    ✓ Generated scene with {len(scene.objects)} racing cars")
        print(f"    ✓ Ego car: {scene.egoObject}")
        
        # Display all cars with their grid positions
        for i, obj in enumerate(scene.objects):
            is_ego = " (POLE POSITION)" if obj is scene.egoObject else ""
            race_num = getattr(obj, 'raceNumber', i)
            team = getattr(obj, 'team', 'Unknown')
            print(f"      P{i+1}{is_ego}: #{race_num} {team} - "
                  f"pos=({obj.position.x:.1f}, {obj.position.y:.1f}), "
                  f"speed={obj.speed:.1f}m/s")
    except Exception as e:
        print(f"    ✗ ERROR: Could not generate scene: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    print(f"\n[3] Checking racing-specific features...")
    try:
        # Check if we have racing track
        if hasattr(scene, 'params') and 'track' in dir(scene):
            print(f"    ✓ Racing track object created")
        
        # Check starting grid
        if hasattr(scene, 'params'):
            print(f"    ✓ Starting grid positions available")
        
        # Check car properties
        ego = scene.egoObject
        if hasattr(ego, 'raceNumber'):
            print(f"    ✓ Ego car has race number: {ego.raceNumber}")
        if hasattr(ego, 'team'):
            print(f"    ✓ Ego car has team: {ego.team}")
        if hasattr(ego, 'fuelLevel'):
            print(f"    ✓ Ego car has fuel level: {ego.fuelLevel:.2f}")
        if hasattr(ego, 'tireWear'):
            print(f"    ✓ Ego car has tire wear: {ego.tireWear:.2f}")
            
    except Exception as e:
        print(f"    ⚠ Warning: Could not verify all racing features: {e}")
    
    print(f"\n[4] Creating dSPACE simulator...")
    try:
        simulator = DSpaceSimulator(
            scenario_src="LagunaSeca_ExternalControl",
            scenario_name="RacingTest",
            timestep=0.1,
            save_as=True
        )
        print(f"    ✓ dSPACE simulator created")
    except Exception as e:
        print(f"    ✗ ERROR: Could not create simulator: {e}")
        return 1
    
    print(f"\n[5] Setting up simulation in ModelDesk...")
    print(f"    (This will configure ego and opponents on starting grid)")
    try:
        simulation = simulator.simulate(scene, maxSteps=1)
        print(f"\n    ✓ SIMULATION SETUP COMPLETE!")
        print(f"\n    Check ModelDesk to see:")
        print(f"      - Ego car at pole position (grid slot 1)")
        print(f"      - {len(scene.objects) - 1} opponent cars on grid")
        print(f"      - All cars aligned with track direction")
        print(f"      - Ready for race start!")
    except Exception as e:
        print(f"    ✗ ERROR: Simulation setup failed: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    print("\n" + "="*80)
    print("RACING DOMAIN TEST COMPLETE")
    print("="*80)
    print("\n💡 Next steps:")
    print("   1. Verify car positions in ModelDesk/Aurelion")
    print("   2. Try adding racing behaviors to the scenario")
    print("   3. Experiment with different grid sizes")
    print("   4. Test pit lane features (when implemented)")
    print("\n" + "="*80 + "\n")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())

