# dSPACE CoSim (VEOS) — README

This folder documents how the **CoSim client** (Python bindings + native `DsVeosCoSim`) connects to **dSPACE VEOS**, and what we learned while wiring it to Docker and the host.

**Last updated:** 2026-03-19

This README incorporates findings from **container inspection** (`docker exec` into the `veos` container) and from the **official CoSim docs** in `cosim-client/doc/` (basics, tutorial prepare.md, step1–step9).

---

## What we learned (summary)

1. **ASM_Traffic alone is not enough for CoSim**  
   The default OSA loaded in the container (`ASM_Traffic.osa`) had no CoSim server registered with the port mapper. The client must use an OSA that **includes a CoSim server** (JSON interface imported into the OSA). This repo adds **`CoSimServer.json`** and **`load_cosim_osa.ps1`** to build and load **`DsVeosCoSim.osa`** with server name **`DsVeosCoSimNgExample`**.

2. **Port mapper vs CoSim server vs “model” port**  
   - **Port mapper** (this environment): TCP **111** inside the container (not the documented default **27027**). The CoSim client must reach it (e.g. host **`11111:111`** if 111 is taken on the host).  
   - **CoSim server**: After the mapper lookup, the client connects to another TCP port. Publish a **symmetric** range on the host, e.g. **`50000-50100:50000-50100/tcp`**, not an offset range like `50100-50200:50000-50100` (the client uses the same port number on the host as inside the container).  
   - **VEOS “model simulator” for AURELION** (separate from CoSim): In an AURELION Manager project (e.g. **IAC_Project**), system configuration **“VEOS CTun”** points `modelSimulator` at VEOS on **`port: 2017`**. That is **not** the CoSim port mapper; AURELION and CoSim can both use the same VEOS instance, but you publish **2017** for AURELION-style access and **111 + CoSim server port(s)** for CoSim.

3. **Unload / load can fail while another app holds the device**  
   `veos-sim unload` / `veos-sim load` failed with “Access denied” when **ControlDesk** / **dSPACEXILServer** had the simulator.    **Restart Docker** (or close those apps), then run **`load_cosim_osa.ps1`** so the CoSim OSA can load cleanly.  
   **Why (container inspection):** The container sets **`DS_VEOS_LOAD_OSA=/home/dspace/VEOS/ASM_Traffic.osa`**. Supervisord runs **veosloadosa**, which does `veos sim load` with that path. **dSPACEXILServer** (XIL/MAPort) then attaches using **MAPortConfig** (e.g. `ASM_Traffic.sdf`). So the simulator is already loaded and owned by XIL; a later `veos-sim load` from **load_cosim_osa.ps1** is rejected. The official docs state that you cannot access co-simulation variables with experiment software such as ControlDesk—so CoSim and XIL/ControlDesk are mutually exclusive for the same loaded simulation.  
   **Workaround:** Restart Docker and run **load_cosim_osa.ps1** as soon as the container is up (before the stack finishes loading ASM_Traffic and starting XIL), or use a **CoSim-only** setup: set **`DS_VEOS_LOAD_OSA`** to the CoSim OSA path and ensure that file exists when **veosloadosa** runs (entrypoint copy or mount); MAPort may need to be disabled or reconfigured (image-dependent).

4. **Native library**
   - **Windows host:** `DsVeosCoSim-PythonBindings/src/DsVeosCoSim.dll` (build from `veos-cosim-client` / third_party).  
   - **Linux (inside container):** `libDsVeosCoSim.so` — can be built in the container (`cmake -DBUILD_SHARED_LIBS=ON`).  
   The Python **`Bindings.py`** tolerates optional DLL symbols (Container / FR / some sim-time APIs) so a minimal client build can still load.

5. **Scenic scenarios and AURELION**  
   An **AURELION Manager** project such as **`C:\Users\bklfh\Documents\dspace\AURELIONManager\IAC_Project`** holds **MOD_Traffic** scenarios under **`Parameterization\MOD_Traffic\Pool\Environment\Scenario\`** (many **`Scenic_*.xml`** / **`.mot`** files in dSPACE ScenarioAccess format). That is the **traffic / scenario** side; **CoSim** here is a separate TCP API to step and exchange data with VEOS once the right OSA is loaded.

---

## How this fits your toolchain

| Piece | Role |
|-------|------|
| **Scenic** (this repo) | Generates scenarios; can run a **CoSim client** from the host to drive VEOS when ports and OSA are correct. |
| **IAC_Project** (AURELION Manager) | Selects system config (e.g. VEOS host/port **2017**), scenarios, sensors; not the same as CoSim. |
| **VEOS in Docker** | Runs the OSA (ASM_Traffic, or **DsVeosCoSim.osa** after `load_cosim_osa.ps1`). |
| **CoSim client** (`heartbeat.py`, `test_step_callback.py`, …) | Connects to port mapper → CoSim server by **`serverName`**. |

---

## Quick start (CoSim OSA + client from host)

1. **Compose:** Publish port mapper and CoSim range, e.g.  
   `11111:111/tcp` and `50000-50100:50000-50100/tcp` (adjust if your environment uses different ports).  
   If you use **AURELION** from the host against Docker VEOS, also publish **`2017:2017/tcp`** (or the IP/port your `SystemConfigurations\*.json` uses).

2. **Restart Docker** so no stale “access denied” state (see above).

3. From **`src/scenic/simulators/dspace/cosim`**:  
   `.\load_cosim_osa.ps1`

4. Run client (example):  
   `python test_step_callback.py DsVeosCoSimNgExample 11111`  
   or  
   `cd DsVeosCoSim-PythonBindings` then  
   `python examples/heartbeat.py DsVeosCoSimNgExample 11111`

---

## Files in this folder (quick reference)

| File / folder | Purpose |
|---------------|---------|
| **`CoSimServer.json`** | CoSim server interface (`DsVeosCoSimNgExample`). |
| **`load_cosim_osa.ps1`** | Copy JSON → container, `veos model import`, `veos-sim load` + `start`. |
| **`test_step_callback.py`** | Blocking callback-based step test from host. |
| **`connect_and_log.py`** | Minimal connect (Python 3.6+); use in container with `COSIM_PORT_MAPPER=111`. |
| **`cosim_evaluation.md`** | Readiness notes and port guidance. |
| **`cosim_veos_inspect.log`** | Captured inspection (listeners, processes, connect attempts). |
| **`DsVeosCoSim-PythonBindings/`** | Python + ctypes bindings and `third_party/veos-cosim-client`. |
| **`cosim-client/doc/`** | Official CoSim docs: `documentation.md`, `basics/`, `tutorial/prepare.md`, step1–step9. |

---

## Loading the CoSim OSA (detail)

A CoSim server JSON and an OSA that contains it are in this folder so VEOS can run a CoSim server the client can connect to.

- **`CoSimServer.json`** — CoSim server description (name **`DsVeosCoSimNgExample`**, step 0.01, CAN, one outgoing signal). Aligns with the dSPACE prepare tutorial (`cosim-client/doc/tutorial/prepare.md`).
- **`load_cosim_osa.ps1`** — Copies the JSON into the **`veos`** container, runs **`veos model import`**, copies **`DsVeosCoSim.osa`** to **`/home/dspace/VEOS/`**, runs **`veos-sim load`** and **`veos-sim start`**.

**Restart Docker** (or the `veos` service) if unload/load was blocked by another application. Then run **`.\load_cosim_osa.ps1`** from this folder.

After load, if the host client still hangs, verify the **CoSim server** port returned by the mapper is included in your published **`ports:`** (symmetric mapping as above).

---

## Setup checklist (from official docs)

From **cosim-client/doc** (basics-servers.md, basics-clients.md, tutorial prepare.md):

- **JSON `Name`** must match the client **`serverName`** (e.g. `DsVeosCoSimNgExample` in `CoSimServer.json` and in client scripts).
- **Port mapper:** Default is **27027**; this image uses **111**. Set **`VEOS_COSIM_PORTMAPPER_PORT`** for both VEOS (before VEOS Kernel starts) and clients when not using the default. Our client scripts take the mapper port as the second argument (e.g. `11111` when host publishes `11111:111`).
- **CoSim server port:** With dynamic port (no `TcpPort` in JSON), the client gets the server port from the mapper. Publish a **symmetric** port range (e.g. `50000-50100:50000-50100`) so the port the mapper returns is reachable from the host.
- **Order:** Load OSA → run client (or start sim first if using optional client); then `veos-sim start` if not already started. Cleanup: `veos-sim unload`.
- **Optional client:** Add `"IsClientOptional": true` to the JSON if the simulation should start without a client connected (see step9-optional-client.md).

---

## Docker compose: ports (checklist)

Under **`veos:`** you typically need:

```yaml
ports:
  - "11111:111/tcp"                    # port mapper (use 111:111 if free on host)
  - "50000-50100:50000-50100/tcp"      # CoSim server (symmetric range)
  - "2017:2017/tcp"                    # optional: VEOS model port for AURELION (see IAC_Project system config)
```

**Client usage:** If you mapped **`11111:111`**, pass **`11111`** as the port-mapper port (second argument to **`heartbeat.py`** / **`test_step_callback.py`**, or **`ConnectConfig(remotePort=...)`**).

---

## Detailed troubleshooting log (historical)

Date: 2026-03-19

### Current state (at time of logging)

1. Docker containers running, including **`veos`** (healthy).
2. dSPACE **`portmapper`** running *inside* the **`veos`** container.
3. Without **`ports:`** on **`veos`**, the **host** could not reach container IPs for CoSim-related ports (`10.6.0.2:27027`, `10.6.0.2:111` failed from host).
4. Inside the container: **`127.0.0.1:111`** open; **`127.0.0.1:27027`** closed.
5. **CoSim server TCP port** for a given OSA must be discovered after a CoSim server exists (mapper + symmetric publish), or read from client logs once connect succeeds.

### What we tried (with results)

#### 1) Verify VEOS container is up
```sh
docker ps --format "table {{.Names}}\t{{.Image}}\t{{.Status}}"
```
- **`veos`** healthy.

#### 2) Host → container IP reachability
- **`10.6.0.2:27027`**, **`10.6.0.2:111`** failed without compose **`ports:`**.

#### 3) Port mapper process
```sh
docker exec veos bash -lc "ps aux | awk 'NR==1{print} /portmapper/ {print}'"
```
- **`/opt/dspace/common/portmapper/bin/portmapper -v`** running.

#### 4) 27027 / 111 inside container (`/dev/tcp`)
- **`127.0.0.1:27027`** → closed  
- **`127.0.0.1:111`** → open  

#### 5) TCP listeners (`/proc/net/tcp` / Python parse)
- Many listeners including **111**, **2017**, **8090**, **22222**, … (see **`cosim_veos_inspect.log`**). Not necessarily **50000–50100** for a given OSA.

#### 6) `veos-sim info`
- **`ASM_Traffic.osa`** loaded; **`ASM_Traffic`**, ExternalControl, RaceControl.

#### 7) CoSim strings in ASM_Traffic files
- No obvious **`cosim`** / **`DsVeosCoSim`** in **`ASM_Traffic.xml`** / **`.sdf`** (simple text search).

---

## What to do next (reference)

**Goals:** (1) Port mapper port on host, (2) CoSim server port(s) on host, (3) correct **`serverName`**.

### Step 1: In-container client (optional)

From inside **`veos`**, with **`libDsVeosCoSim.so`** and Python bindings on **`PYTHONPATH`**, run **`connect_and_log.py`** or **`heartbeat.py`** with **`COSIM_PORT_MAPPER=111`** (or pass mapper port **111** via **`ConnectConfig`**) and capture **`Connected to ... at 127.0.0.1:<port>`**.

**Note:** Container Python may be &lt; 3.10 — use **`connect_and_log.py`** or avoid **`match`** in scripts.

### Step 2: Publish ports in `dspace_art_stack.yml`

See **[Docker compose: ports (checklist)](#docker-compose-ports-checklist)** above.

### Step 3: Host client

```powershell
python examples/heartbeat.py DsVeosCoSimNgExample 11111
```

(with **`11111`** replaced by your host port for container **111**).

---

## Notes / caveats

- Without TCP **`ports:`** on **`veos`**, host-side CoSim clients usually cannot complete the mapper + server handshake.
- CoSim uses **TCP**; order is typically **port mapper** then **CoSim server**.
- Documented default port mapper is **27027**; this environment uses **111**. Set **`VEOS_COSIM_PORTMAPPER_PORT`** (e.g. to `111`) for both the VEOS process and the CoSim client when not using the default; our scripts pass the mapper port as the second argument (host port when connecting from host).
- Full inspection lines: **`cosim_veos_inspect.log`**. Readiness narrative: **`cosim_evaluation.md`**.
- **Official docs:** **`cosim-client/doc/`** — start with **documentation.md**, then **basics/** (basics-servers.md, basics-clients.md), **tutorial/prepare.md**, and step1–step9 for connect order, callbacks, and optional client.
