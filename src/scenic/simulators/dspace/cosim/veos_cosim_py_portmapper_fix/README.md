## Important note for your current error

If you see:

```text
Could not find port for server 'CoSimServerScenic'.
Could not find port for VeosCoSim server.
```

that means the library could not reach the VEOS port mapper. According to the vendor documentation, the client uses the port mapper by default and the port mapper listens on TCP port **27072** unless the environment variable `VEOSCOSIM_PORTMAPPER_PORT` was changed. If your working EXE is started in an environment where that variable is set, Python must use the same value.

This package now supports:

```powershell
py print_time_callbacks.py --backend bridge --remote-ip 192.168.100.101 --server-name CoSimServerScenic --portmapper-port 27072
```

Replace `27072` with the real value if your VEOS setup changed the port mapper port.

# veos_cosim_py

A sibling Python package for `VeosCoSim_Client` that does **not** modify the vendor folder and does **not** launch another CoSim client process.

## Folder layout

Put this folder parallel to the uploaded vendor folder:

```text
<parent>/
├─ VeosCoSim_Client/
└─ veos_cosim_py/
```

For example:

```text
C:\Users\bklfh\Documents\Scenic\Scenic\src\scenic\simulators\dspace\cosim\
├─ VeosCoSim_Client\
└─ veos_cosim_py\
```

## Build the bridge DLL (recommended)

Open a developer PowerShell or a shell where `cmake` and `cl` work.

From inside `veos_cosim_py` run:

```powershell
.\build_bridge.bat
```

The build now auto-detects either of these vendor layouts:

```text
..\VeosCoSim_Client\client\x64\Release
..\VeosCoSim_Client\VeosCoSim_Client\client\x64\Release
```

That builds:

```text
bridge\build\Release\veos_cosim_bridge.dll
```

## Timer callback test

After building the bridge DLL, run:

```powershell
py print_time_callbacks.py --backend bridge --remote-ip 192.168.100.101 --server-name CoSimServerScenic
```

If you want to try the DLL route anyway:

```powershell
py print_time_callbacks.py --backend direct --remote-ip 192.168.100.101 --server-name CoSimServerScenic
```
