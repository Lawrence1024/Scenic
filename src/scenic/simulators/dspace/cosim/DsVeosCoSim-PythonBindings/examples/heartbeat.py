"""Heartbeat example: connect to CoSim server and send CAN messages on each step.
Usage: python heartbeat.py [SERVER_NAME] [PORT_MAPPER_PORT]
SERVER_NAME must match CoSimServer.json 'Name'; PORT_MAPPER_PORT is host port for mapper (e.g. 11111)."""
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
    port_mapper_port = None
    if len(sys.argv) > 1:
        service_name = sys.argv[1]
    if len(sys.argv) > 2:
        port_mapper_port = int(sys.argv[2])

    if port_mapper_port is not None:
        config = ConnectConfig(
            remoteIpAddress="127.0.0.1",
            serverName=service_name,
            remotePort=port_mapper_port,
        )
    else:
        config = ConnectConfig(serverName=service_name)

    with CoSimClient().Connect(config) as connection:
        connection.AttachToSimulation(handle_command)


if __name__ == "__main__":
    main()
