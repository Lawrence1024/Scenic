# VeosCoSim_Client

This folder contains the vendor VEOS CoSimulation SDK and example client code.

It is the reference point for:
- headers
- static libraries
- example client source files

This folder should generally be treated as vendor-owned code.

---

## Folder purpose

This folder provides the pieces needed to build CoSim client executables.

The most important parts are:

### SDK header
```text
client/x64/Release/include/VeosCoSim.h
```

This file defines:
- `VeosCoSim_CreateMI`
- `VeosCoSim_ConnectMI`
- `VeosCoSim_StartNonBlockingMI`
- `VeosCoSim_GetNextCommandMI`
- `VeosCoSim_FinishCommandMI`
- callback types
- enums and result codes

### Static library
```text
client/x64/Release/lib/VeosCoSimApplStatic.lib
```

This is the library used to build working client executables in this setup.

### Example sources
```text
examples/client/VeosCoSimTestClient.cpp
examples/client/ClientServerTestHelper.cpp
examples/client/Generator.cpp
```

These are the reference sources used to build the standard example client.

---

## How this folder is used by the rest of the project

The IPC bridge in `veos_cosim_ipc_bridge/` depends on this folder for:
- SDK includes
- static library linking
- example code reference

The bridge is not intended to replace this folder.  
Instead, it builds on top of it.

---

## Build the main example EXE

Run this from:

```powershell
cd C:\Users\bklfh\Documents\Scenic\Scenic\src\scenic\simulators\dspace\cosim\VeosCoSim_Client\examples\client
```

Then build:

```powershell
cl /std:c++17 /EHsc /MD ^
  /I "C:\Users\bklfh\Documents\Scenic\Scenic\src\scenic\simulators\dspace\cosim\VeosCoSim_Client\client\x64\Release\include" ^
  VeosCoSimTestClient.cpp ClientServerTestHelper.cpp Generator.cpp ^
  /link ^
  /LIBPATH:"C:\Users\bklfh\Documents\Scenic\Scenic\src\scenic\simulators\dspace\cosim\VeosCoSim_Client\client\x64\Release\lib" ^
  VeosCoSimApplStatic.lib Ws2_32.lib ^
  /OUT:"VeosCoSimTestClient.exe"
```

---

## Run the main example EXE

From the same folder:

```powershell
.\VeosCoSimTestClient.exe --name "CoSimServerScenic" --host 192.168.100.101
```

This is useful for verifying:
- the SDK build works
- the VEOS server is reachable
- the server name is correct
- the host is correct

---

## Important build notes

### Release build path
The currently working setup uses the Release include/lib paths:

```text
client/x64/Release/include
client/x64/Release/lib
```

### Static linking
The currently working setup uses:

```text
VeosCoSimApplStatic.lib
```

This was important in practice.  
Earlier attempts using other combinations were less reliable.

### Visual Studio shell
Use a VS Developer Command Prompt or a shell where `cl` is available.

---

## Important files and what they do

### `VeosCoSimTestClient.cpp`
The main example client.  
This is the closest vendor reference for:
- connecting to VEOS
- registering callbacks
- running the command loop

### `ClientServerTestHelper.cpp`
Helper functions used by the example.

### `Generator.cpp`
Support file used by the example.

### `VeosCoSim.h`
The main API contract for the client SDK.

---

## How to interface with this folder

### When you want to validate the SDK itself
Rebuild and run `VeosCoSimTestClient.exe`.

### When you want to build a custom client
Use the same include/lib structure and static library as the working example build.

### When you want Python visibility
Do not modify this folder directly.  
Instead, use the custom code in:

```text
..\veos_cosim_ipc_bridge
```

That folder wraps the same VEOS client logic with local IPC to Python.

---

## Caution

Do not run multiple VEOS clients against the same server instance unless you are sure the setup supports it.

For the current workflow, use either:
- the vendor example EXE, or
- the IPC-enabled EXE

but not both at once.
