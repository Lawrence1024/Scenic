from datetime import datetime
from pathlib import Path
import sys

sys.path.append(str(Path(__file__).parent.parent / "src"))
from Bindings import Command
from CoSimClient import CoSimClient, CoSimSession, ConnectConfig


def handle_command(session: CoSimSession, simulation_time: int, command: Command):
    match command:
        case Command.STEP:  # "StepSize": 0.01
            for controller in session.connection.GetCanControllers():
                msg = session.MakeCanMessage(
                    data=bytes(datetime.now().strftime("%H:%M:%S"), "utf-8")
                ).FromController(controller)
                session.TransmitCanMessage(msg)


def main():
    service_name = "DsVeosCoSimNgExample"
    if len(sys.argv) > 1:
        service_name = sys.argv[1]

    with CoSimClient().Connect(ConnectConfig(serverName=service_name)) as connection:
        connection.AttachToSimulation(handle_command)


if __name__ == "__main__":
    main()
