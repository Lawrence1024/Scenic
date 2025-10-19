#!/usr/bin/env python3
"""
Explore ControlDesk COM API to find methods for going online and starting experiments.
"""

from win32com.client import Dispatch
import pywintypes

def explore_object_methods(obj, name="Object"):
    """Explore methods and properties of a COM object."""
    print(f"\n{name} available methods and properties:")
    print("=" * 60)
    
    # Get all attributes
    try:
        # Use dir() to get all attributes
        attrs = [attr for attr in dir(obj) if not attr.startswith('_')]
        
        # Categorize by likely type
        methods = []
        properties = []
        
        for attr in attrs:
            try:
                val = getattr(obj, attr)
                if callable(val):
                    methods.append(attr)
                else:
                    properties.append(f"{attr} = {val}")
            except:
                methods.append(attr)  # Assume it's a method if we can't get the value
        
        if methods:
            print("\nMethods (potentially callable):")
            for method in sorted(methods):
                print(f"  - {method}()")
        
        if properties:
            print("\nProperties:")
            for prop in sorted(properties):
                print(f"  - {prop}")
                
    except Exception as e:
        print(f"Error exploring object: {e}")

def main():
    print("Exploring ControlDesk COM API")
    print("=" * 60)
    
    try:
        # Connect to ControlDesk
        print("\n1. Connecting to ControlDesk...")
        cd = Dispatch("ControlDeskNG.Application")
        print("[OK] Connected")
        
        # Explore Application object
        print("\n" + "=" * 60)
        print("EXPLORING: ControlDesk Application")
        explore_object_methods(cd, "Application")
        
        # Get project
        print("\n" + "=" * 60)
        proj = cd.ActiveProject
        print(f"\n2. Active Project: {proj.Name}")
        print("EXPLORING: Project")
        explore_object_methods(proj, "Project")
        
        # Get experiment
        print("\n" + "=" * 60)
        exp = proj.ActiveExperiment
        print(f"\n3. Active Experiment: {exp.Name}")
        print("EXPLORING: Experiment")
        explore_object_methods(exp, "Experiment")
        
        # Look for platform-related objects
        print("\n" + "=" * 60)
        print("\n4. Looking for Platform object...")
        try:
            if hasattr(exp, 'Platform'):
                platform = exp.Platform
                print(f"[OK] Found Platform")
                print("EXPLORING: Platform")
                explore_object_methods(platform, "Platform")
        except Exception as e:
            print(f"No Platform object: {e}")
        
        # Try to find online/start methods
        print("\n" + "=" * 60)
        print("\n5. Searching for Start/Online methods...")
        
        potential_start_methods = []
        for obj_name, obj in [("Application", cd), ("Project", proj), ("Experiment", exp)]:
            for attr in dir(obj):
                if any(keyword in attr.lower() for keyword in ['start', 'online', 'connect', 'run', 'go']):
                    potential_start_methods.append(f"{obj_name}.{attr}")
        
        if potential_start_methods:
            print("Potential methods to start/go online:")
            for method in sorted(potential_start_methods):
                print(f"  - {method}")
        else:
            print("No obvious start/online methods found")
        
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()

