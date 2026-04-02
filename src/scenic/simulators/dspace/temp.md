Yes — in your case, the cleanest setup is:

* keep the container doing its normal job
* start recording on demand with `docker exec`
* stop recording on demand by sending a graceful signal to the recorder process
* wrap both actions in Python so you can do it from Windows

A good pattern is to avoid `tmux` entirely unless you want an interactive session manager. Since you want “start recording” and “stop recording” commands, the simplest reliable approach is:

1. inside the container, start `ros2 bag record` in the background and save its PID
2. later, read that PID and send `SIGINT` so the bag closes cleanly

That gives you explicit control.

## Inside-container commands

Assuming:

* container name: `art_driving_stach`
* mounted host path: `~/ros_ws/ros_bags:/ros_bags`

### Start recording

```bash
docker exec art_driving_stach bash -lc '
source install/setup.bash
mkdir -p /ros_bags
STAMP=$(date +%Y%m%d_%H%M%S)
BAG_DIR=/ros_bags/rosbag_$STAMP
LOG_FILE=/ros_bags/rosbag_$STAMP.log

if [ -f /tmp/rosbag_record.pid ] && kill -0 $(cat /tmp/rosbag_record.pid) 2>/dev/null; then
  echo "rosbag recorder already running with PID $(cat /tmp/rosbag_record.pid)"
  exit 1
fi

nohup ros2 bag record -a -o "$BAG_DIR" > "$LOG_FILE" 2>&1 &
echo $! > /tmp/rosbag_record.pid
echo "started recording: PID=$!, BAG_DIR=$BAG_DIR"
'
```

### Stop recording gracefully

```bash
docker exec art_driving_stach bash -lc '
if [ ! -f /tmp/rosbag_record.pid ]; then
  echo "no pid file found"
  exit 1
fi

PID=$(cat /tmp/rosbag_record.pid)

if ! kill -0 "$PID" 2>/dev/null; then
  echo "process $PID is not running"
  rm -f /tmp/rosbag_record.pid
  exit 1
fi

kill -INT "$PID"
wait "$PID" 2>/dev/null || true
rm -f /tmp/rosbag_record.pid
echo "stopped recording: PID=$PID"
'
```

### Check status

```bash
docker exec art_driving_stach bash -lc '
if [ -f /tmp/rosbag_record.pid ]; then
  PID=$(cat /tmp/rosbag_record.pid)
  if kill -0 "$PID" 2>/dev/null; then
    echo "recording, PID=$PID"
  else
    echo "stale pid file"
  fi
else
  echo "not recording"
fi
'
```

## Why this is better than killing the container

Because `kill -INT` is basically the same as pressing `Ctrl+C`, which lets `ros2 bag record` flush and finalize metadata properly. That is much safer than killing the whole container or hard-killing the process.

## Python version

If you want this all doable in Python from Windows, the easiest way is to have Python call `wsl` and then `docker exec`.

Here is a usable script:

```python
import subprocess
import sys


CONTAINER = "art_driving_stach"


def run_wsl_cmd(cmd: str):
    full_cmd = ["wsl", "bash", "-lc", cmd]
    result = subprocess.run(full_cmd, capture_output=True, text=True)
    return result.returncode, result.stdout.strip(), result.stderr.strip()


def start_recording():
    cmd = rf"""
docker exec {CONTAINER} bash -lc '
source install/setup.bash
mkdir -p /ros_bags
STAMP=$(date +%Y%m%d_%H%M%S)
BAG_DIR=/ros_bags/rosbag_$STAMP
LOG_FILE=/ros_bags/rosbag_$STAMP.log

if [ -f /tmp/rosbag_record.pid ] && kill -0 $(cat /tmp/rosbag_record.pid) 2>/dev/null; then
  echo "rosbag recorder already running with PID $(cat /tmp/rosbag_record.pid)"
  exit 1
fi

nohup ros2 bag record -a -o "$BAG_DIR" > "$LOG_FILE" 2>&1 &
echo $! > /tmp/rosbag_record.pid
echo "started recording: PID=$!, BAG_DIR=$BAG_DIR"
'
"""
    return run_wsl_cmd(cmd)


def stop_recording():
    cmd = rf"""
docker exec {CONTAINER} bash -lc '
if [ ! -f /tmp/rosbag_record.pid ]; then
  echo "no pid file found"
  exit 1
fi

PID=$(cat /tmp/rosbag_record.pid)

if ! kill -0 "$PID" 2>/dev/null; then
  echo "process $PID is not running"
  rm -f /tmp/rosbag_record.pid
  exit 1
fi

kill -INT "$PID"
rm -f /tmp/rosbag_record.pid
echo "stopped recording: PID=$PID"
'
"""
    return run_wsl_cmd(cmd)


def status_recording():
    cmd = rf"""
docker exec {CONTAINER} bash -lc '
if [ -f /tmp/rosbag_record.pid ]; then
  PID=$(cat /tmp/rosbag_record.pid)
  if kill -0 "$PID" 2>/dev/null; then
    echo "recording, PID=$PID"
  else
    echo "stale pid file"
    exit 1
  fi
else
  echo "not recording"
fi
'
"""
    return run_wsl_cmd(cmd)


if __name__ == "__main__":
    if len(sys.argv) != 2 or sys.argv[1] not in {"start", "stop", "status"}:
        print("Usage: python rosbag_control.py [start|stop|status]")
        sys.exit(1)

    action = sys.argv[1]
    if action == "start":
        code, out, err = start_recording()
    elif action == "stop":
        code, out, err = stop_recording()
    else:
        code, out, err = status_recording()

    if out:
        print(out)
    if err:
        print(err, file=sys.stderr)

    sys.exit(code)
```

## How you would use it

From Windows PowerShell:

```bash
python rosbag_control.py start
python rosbag_control.py status
python rosbag_control.py stop
```

That script will:

* enter WSL
* run `docker exec ...`
* start or stop recording inside the running container

## Even cleaner option: put scripts inside the container

If you can add two helper scripts into the container, this gets much cleaner.

### `/usr/local/bin/start_rosbag.sh`

```bash
#!/usr/bin/env bash
set -e

source install/setup.bash
mkdir -p /ros_bags
STAMP=$(date +%Y%m%d_%H%M%S)
BAG_DIR=/ros_bags/rosbag_$STAMP
LOG_FILE=/ros_bags/rosbag_$STAMP.log

if [ -f /tmp/rosbag_record.pid ] && kill -0 $(cat /tmp/rosbag_record.pid) 2>/dev/null; then
  echo "rosbag recorder already running with PID $(cat /tmp/rosbag_record.pid)"
  exit 1
fi

nohup ros2 bag record -a -o "$BAG_DIR" > "$LOG_FILE" 2>&1 &
echo $! > /tmp/rosbag_record.pid
echo "started recording: PID=$!, BAG_DIR=$BAG_DIR"
```

### `/usr/local/bin/stop_rosbag.sh`

```bash
#!/usr/bin/env bash
set -e

if [ ! -f /tmp/rosbag_record.pid ]; then
  echo "no pid file found"
  exit 1
fi

PID=$(cat /tmp/rosbag_record.pid)

if ! kill -0 "$PID" 2>/dev/null; then
  echo "process $PID is not running"
  rm -f /tmp/rosbag_record.pid
  exit 1
fi

kill -INT "$PID"
rm -f /tmp/rosbag_record.pid
echo "stopped recording: PID=$PID"
```

Then your commands become much shorter:

```bash
wsl bash -lc "docker exec art_driving_stach start_rosbag.sh"
wsl bash -lc "docker exec art_driving_stach stop_rosbag.sh"
```

And Python just calls those.

## Recommendation

For your workflow, I’d use the PID-file approach, not `tmux`.

Best practical setup:

* mount `~/ros_ws/ros_bags:/ros_bags`
* start with `docker exec ... nohup ros2 bag record ... &`
* store PID in `/tmp/rosbag_record.pid`
* stop with `kill -INT $(cat /tmp/rosbag_record.pid)`
* wrap both in Python using `subprocess.run(["wsl", ...])`

One important note: if `ros2 bag record` spawns child processes in your ROS version, stopping by PID may occasionally miss part of the process tree. Usually it works fine, but if you want, I can give you a slightly more robust version that starts the recorder in its own process group and stops the whole group cleanly.
