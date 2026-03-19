"""Minimal connect-only script for Python 3.6+ (no match/case). Run inside VEOS container to capture Connected at ... :port.
Set COSIM_PORT_MAPPER to the port mapper port (default 111 in container). Server name must match CoSimServer.json 'Name'."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent / "DsVeosCoSim-PythonBindings" / "src"))

from CoSimClient import CoSimClient, ConnectConfig

def main():
    import os
    port = os.environ.get("COSIM_PORT_MAPPER", "111")
    port = int(port)
    config = ConnectConfig(
        remoteIpAddress="127.0.0.1",
        serverName="DsVeosCoSimNgExample",
        remotePort=port,
    )
    print("Connecting with serverName=DsVeosCoSimNgExample at 127.0.0.1:%s ..." % port, flush=True)
    with CoSimClient().Connect(config) as connection:
        print("Connected successfully. Connection object:", connection, flush=True)
    print("Disconnected.", flush=True)

if __name__ == "__main__":
    main()
