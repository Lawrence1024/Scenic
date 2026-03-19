"""
Test script for the blocking (callback-based) CoSim step API.

Runs RunCallbackBasedCoSimulation so that VEOS drives the simulation and
notifies us via a callback when each simulation step completes. This is the
"blocking" version: the main thread blocks in RunCallbackBasedCoSimulation
until the simulation ends; each time a step completes, on_end_step_simulation
is invoked.

Usage (from this directory or repo root with PYTHONPATH set):
  python test_step_callback.py [SERVER_NAME] [PORT_MAPPER_PORT]

SERVER_NAME must match the "Name" in CoSimServer.json (default: DsVeosCoSimNgExample).
PORT_MAPPER_PORT is the host port that maps to the container's port mapper (e.g. 11111 for 11111:111).

Examples:
  python test_step_callback.py
  python test_step_callback.py DsVeosCoSimNgExample 11111

If it hangs at "Connecting to ...":
  - Ensure the veos container is running and ports are published (e.g. 11111:111
    and 50000-50100:50000-50100 so the host port range matches the container
    CoSim server ports).
  - Check: Test-NetConnection -ComputerName 127.0.0.1 -Port 11111
  - Try the heartbeat example with the same args; if that also hangs, fix
    connectivity first.
"""

from pathlib import Path
import sys

# Add Python bindings so we can import Bindings and CoSimClient
_cosim_dir = Path(__file__).resolve().parent
sys.path.insert(0, str(_cosim_dir / "DsVeosCoSim-PythonBindings" / "src"))

from Bindings import DLLConfiguration, simulation_time_to_seconds
from CoSimClient import CoSimClient, ConnectConfig


def main():
    service_name = "DsVeosCoSimNgExample"
    port_mapper_port = None
    if len(sys.argv) > 1:
        service_name = sys.argv[1]
    if len(sys.argv) > 2:
        port_mapper_port = int(sys.argv[2])

    if port_mapper_port is not None:
        connect_config = ConnectConfig(
            remoteIpAddress="127.0.0.1",
            serverName=service_name,
            remotePort=port_mapper_port,
        )
    else:
        connect_config = ConnectConfig(serverName=service_name)

    step_count = [0]  # use list so closure can mutate

    def on_end_step(simulation_time_ns: int) -> None:
        """Called by the CoSim framework when a VEOS step has completed (blocking callback)."""
        step_count[0] += 1
        t_sec = simulation_time_to_seconds(simulation_time_ns)
        print(f"[step complete] step={step_count[0]} simulation_time_ns={simulation_time_ns} ({t_sec:.5f} s)", flush=True)

    print("Connecting to VEOS CoSim (this may take a few seconds)...", flush=True)
    with CoSimClient().Connect(connect_config) as connection:
        print("Connected. Starting callback-based co-simulation (blocking).", flush=True)
        config = DLLConfiguration()
        config.on_end_step_simulation = on_end_step

        print("You will be notified on each step completion. Press Ctrl+C to stop.", flush=True)
        try:
            connection.RunCallbackBasedCoSimulation(config)
        except KeyboardInterrupt:
            print("\nInterrupted by user.")
        print(f"Total steps received: {step_count[0]}")


if __name__ == "__main__":
    main()
