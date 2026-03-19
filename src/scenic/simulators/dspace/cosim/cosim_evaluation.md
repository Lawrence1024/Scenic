# CoSim readiness evaluation (from logs in this folder)

**Logs used:** `cosim_veos_inspect.log`, `cosim_heartbeat_in_container.log` (and in-container connect run).

---

## IAC_Project / AURELION (context)

From **`C:\Users\bklfh\Documents\dspace\AURELIONManager\IAC_Project`** (example):

- **Animation/AURELION/SystemConfigurations/** — e.g. **“VEOS CTun”** uses **`modelSimulator.type: VEOS`**, **`port: 2017`**, **`host`** (e.g. `192.168.100.101`). That is the **VEOS model** connection for AURELION, **not** the CoSim port mapper.
- **Parameterization/MOD_Traffic/Pool/Environment/Scenario/** — many **`Scenic_*.xml`** traffic scenarios (dSPACE ScenarioAccess). Scenic-generated content for the toolchain; separate from the CoSim TCP client in this repo.
- For host access to both: publish **2017** (AURELION) and **111 + CoSim server ports** (CoSim) in compose. See **README.md**.

---

## 0. CoSim OSA setup (done in repo)

- **`CoSimServer.json`** – CoSim server interface with name `DsVeosCoSimNgExample` (so existing scripts work without change).
- **OSA creation** – `veos model import` was run in the container; it created `DsVeosCoSim.osa` (and a copy under `/home/dspace/VEOS/DsVeosCoSim.osa`).
- **Loading** – `veos-sim unload` / `veos-sim load` failed with “Access denied” while ASM_Traffic was running (another app holding the device). So the CoSim OSA is **not** loaded yet.
- **Next step for you:** **Restart Docker** (so no application is loaded), then run **`load_cosim_osa.ps1`** from this folder. That script will create the OSA from the JSON, load it, and start the simulation. After that, run the client (e.g. `test_step_callback.py DsVeosCoSimNgExample 11111`). If the client still hangs, publish the CoSim server port (see section 3 below).

---

## 1. Findings from inspection

| Check | Result |
|-------|--------|
| **OSA loaded** | ASM_Traffic (ExternalControl, RaceControl). No CoSim component name in `veos-sim info`. |
| **CoSim in OSA files?** | No `cosim` / `DsVeosCoSim` strings in ASM_Traffic.xml or ASM_Traffic.sdf. No .json interface files in /home/dspace/VEOS/. |
| **Port mapper** | Running; 127.0.0.1:111 OPEN inside container. |
| **TCP listen ports (container)** | 111, 2017, 8090, 22222, 22350, 31815, 33205, ... (full list in cosim_veos_inspect.log). **No ports in 50000–50100** in the observed list. |
| **Client from host** | Connects to port mapper at 127.0.0.1:11111 (if you use 11111:111), then hangs—likely when connecting to the CoSim server port returned by the mapper (that port may not be published on the host, or the server name may not be registered). |
| **Client from inside container** | With port 111: "Connecting to ... at 127.0.0.1:111..." then **hangs**; no "Connected to ... at ... :PORT". So either the port mapper does not have a server named `DsVeosCoSimNgExample`, or the CoSim server port is never returned. |

---

## 2. Conclusion: is CoSim ready?

**Not yet.** Reasons:

1. **No CoSim server visible in the loaded OSA**  
   The running application is ASM_Traffic with no CoSim-related strings in its files and no CoSim component in `veos-sim info`. So either:
   - The OSA does not include a CoSim server, or
   - The server has another name (not `DsVeosCoSimNgExample`).

2. **We could not get the CoSim server port**  
   From inside the container, using the port mapper at 111, the client never reached a "Connected to ... at ... :PORT" line. So we do not know which TCP port to publish for the CoSim server.

3. **Host-side hang**  
   From the host, the client reaches the port mapper (11111) then hangs, consistent with either:
   - The CoSim server port returned by the mapper not being published on the host (e.g. you have 50100–50200:50000–50100 but the server might be on one of 22222, 31815, …), or
   - No CoSim server registered under `DsVeosCoSimNgExample`.

---

## 3. Port publishing recommendation (for when a CoSim server exists)

- **Port mapper:** You already use a host port (e.g. 11111) because 111 is in use. Keep:
  ```yaml
  - "11111:111/tcp"
  ```
  Run the client with that host port as the second argument (e.g. `... 11111`).

- **CoSim server port:** We did **not** observe a CoSim server port. When you have a working CoSim server in VEOS:
  - Either run the client from inside the container with port 111 and capture the "Connected to ... at 127.0.0.1:**PORT**" line, then publish that port:
    ```yaml
    - "PORT:PORT/tcp"   # e.g. "22222:22222/tcp" if that is the port
    ```
  - Or publish the **full range** of listen ports we saw, so whichever one is the CoSim server is reachable from the host:
    ```yaml
    - "11111:111/tcp"
    - "2017:2017/tcp"
    - "8090:8090/tcp"
    - "22222:22222/tcp"
    - "22350:22350/tcp"
    # ... add the rest from cosim_veos_inspect.log if needed, or a broad range
    ```
  - Or use a **symmetric range** that includes those ports, e.g.:
    ```yaml
    - "20000-60000:20000-60000/tcp"
    ```
    (Only if your compose/network allows it.)

---

## 4. Next steps (in order)

1. **Confirm a CoSim server in VEOS**  
   In VEOS Model Console / config, check whether the loaded OSA (or another OSA you can load) actually has a CoSim server and note its **exact name** (e.g. `DsVeosCoSimNgExample` or something else). If possible, add or import the CoSim JSON interface and load that OSA.

2. **Discover the CoSim server port**  
   Once a CoSim server is running and registered with the port mapper, run from inside the container (with port 111):
   ```bash
   docker exec veos bash -c "cd /tmp && python3 connect_and_log.py 2>&1"
   ```
   and capture the line "Connected to ... at 127.0.0.1:**PORT**". Then publish that **PORT** on the host (e.g. `"PORT:PORT/tcp"` in `dspace_art_stack.yml`).

3. **Publish that port (and 11111:111)** in `dspace_art_stack.yml`, restart the stack, then run the client from the host with the correct server name and port mapper port (e.g. 11111).

4. **Re-run the step callback test** (`test_step_callback.py`) from the host once the connection succeeds.
