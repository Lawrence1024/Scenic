from pathlib import Path
import sys

sys.path.append(str(Path(__file__).parent.parent / "src"))
from Bindings import Severity, Command, formatted_print, simulation_time_to_seconds
from CoSimClient import CoSimClient, CoSimSession, ConnectConfig


def handle_command(session: CoSimSession, simulation_time: int, command: Command):
    match command:
        case Command.STEP:
            formatted_print(
                Severity.INFO,
                f"STEP command received at simulation time {simulation_time_to_seconds(simulation_time):.5f} seconds.",
            )
        case Command.START:
            formatted_print(
                Severity.INFO,
                f"START command received at simulation time {simulation_time_to_seconds(simulation_time):.5f} seconds.",
            )
        case Command.STOP:
            formatted_print(
                Severity.INFO,
                f"STOP command received at simulation time {simulation_time_to_seconds(simulation_time):.5f} seconds.",
            )
        case Command.PAUSE:
            formatted_print(
                Severity.INFO,
                f"PAUSE command received at simulation time {simulation_time_to_seconds(simulation_time):.5f} seconds.",
            )
        case Command.CONTINUE:
            formatted_print(
                Severity.INFO,
                f"CONTINUE command received at simulation time {simulation_time_to_seconds(simulation_time):.5f} seconds.",
            )
        case Command.TERMINATE:
            formatted_print(
                Severity.INFO,
                f"TERMINATE command received at simulation time {simulation_time_to_seconds(simulation_time):.5f} seconds.",
            )


def main():
    service_name = "DsVeosCoSimNgExample"
    if len(sys.argv) > 1:
        service_name = sys.argv[1]

    with CoSimClient().Connect(ConnectConfig(serverName=service_name)) as connection:
        connection.AttachToSimulation(handle_command)


if __name__ == "__main__":
    main()
