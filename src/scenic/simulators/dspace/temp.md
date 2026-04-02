# ROS 2 bag recording with Scenic dSPACE

This document describes how **optional** ROS 2 bag recording is tied to **each Scenic simulation sample** when using the dSPACE simulator.

## Behavior

- **Default:** no recording. Nothing runs unless you opt in with a Scenic `param`.
- **Start:** at the end of `DSpaceSimulation.setup()`, after ModelDesk/ControlDesk setup and race “go” signals—immediately before the simulation begins stepping.
- **Stop:** at the **beginning** of `DSpaceSimulation.destroy()`, before ControlDesk teardown. Scenic’s `Simulation.run()` always calls `destroy()` in a `finally` block, so each sample gets a matching stop.

Implementation lives under [`ros2_bag/`](ros2_bag/): [`config.py`](ros2_bag/config.py) (scene params) and [`recorder.py`](ros2_bag/recorder.py). Recording uses **native `docker exec <container> bash -c '…'`**, the same engine and container model as **`DSpaceSimulation._call_art_stack_reset`** (ART reset). No WSL indirection.

## Enabling recording

Set in your `.scenic` file:

```scenic
param record_ros2_bag = True
```

If this param is **absent** or **false**, no `ros2 bag record` process is started.

### Other scene params (only when `record_ros2_bag` is true)

| Param | Purpose |
|--------|---------|
| `ros2_bag_container` | Docker container name. If omitted, uses the simulator’s `art_stack_container` (see `DSpaceSimulator`). |
| `ros2_bag_parent_dir` | Directory **inside** the container for bags (default `/ros_bags`). Mount this to the host (e.g. `~/ros_ws/ros_bags:/ros_bags`). |
| `ros2_bag_topics` | Optional list of topic names; if set, records those topics only. If omitted, uses `ros2 bag record -a`. |
| `ros2_bag_setup_source` | Shell line to source the ROS workspace before `ros2 bag record`. **Default:** `source /opt/race_common/install/setup.bash` (matches ART reset in `simulator.py`). |

The legacy params `ros2_bag_use_wsl` and `ros2_bag_wsl_distro` are **ignored** if present; use the same `docker` on your host PATH that successfully runs ART reset.

## Docker / container expectations

- The container must have ROS 2 CLI available and a workspace that matches `ros2_bag_setup_source`.
- Recording runs as: `nohup ros2 bag record … &` with the PID in **`/tmp/rosbag_record.pid`** inside the container. Each sample writes to **`${ros2_bag_parent_dir}/rosbag_<YYYYMMDD_HHMMSS>/`** (and a matching `.log` next to it). That timestamp is **wall-clock second** resolution: two samples starting in the same second could target the same folder name.
- **Stop** sends **SIGINT** to that PID so the bag finalizes cleanly (same idea as Ctrl+C).

## Limitations

- **Host setup:** The container must be reachable from whatever runs `docker exec` (the same as ART tooling).
- **Process groups:** some ROS versions spawn children; stopping the recorded PID is usually enough. A process-group-based stop can be added later if needed.

## Example

See `examples/racing/art_fellow_combined.scenic` in the repo root for `param record_ros2_bag = True` (container defaults to `art_stack_container` unless overridden).

## Manual smoke test (no Scenic)

From the repo root (with Scenic importable):

```bash
python -m scenic.simulators.dspace.ros2_bag.test_record_5s
```

This starts recording, sleeps **5 seconds** (override with `--duration`), then stops—exercising the same `Ros2BagRecorder` start/stop path as the simulator. Use `--container`, `--parent-dir`, `--topics`, and `--setup-source` as needed.
