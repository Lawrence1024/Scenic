"""Start/stop ``ros2 bag record`` inside a Docker container (same pattern as ART reset)."""

from __future__ import annotations

import logging
import subprocess

from scenic.simulators.dspace.ros2_bag.config import Ros2BagConfig

logger = logging.getLogger(__name__)

PID_FILE = "/tmp/rosbag_record.pid"


def _bash_sq(s: str) -> str:
    """POSIX-style single-quoted string for use inside bash scripts (safe on Windows hosts)."""
    return "'" + s.replace("'", "'\\''") + "'"


def _docker_exec_bash_c(config: Ros2BagConfig, script: str) -> tuple[int, str, str]:
    """Run *script* in the container with ``docker exec … bash -c`` (matches ART tooling)."""
    proc = subprocess.run(
        ["docker", "exec", config.container, "bash", "-c", script],
        capture_output=True,
        text=True,
        timeout=120,
    )
    out = (proc.stdout or "").strip()
    err = (proc.stderr or "").strip()
    return proc.returncode, out, err


class Ros2BagRecorder:
    """One recorder per Scenic simulation sample (one ``DSpaceSimulation`` instance).

    Each sample runs a separate ``ros2 bag record`` into ``rosbag_<YYYYMMDD_HHMMSS>/`` under
    the configured parent directory. The active recorder PID is tracked in ``PID_FILE`` inside
    the container (one recording at a time per container).
    """

    def __init__(self, config: Ros2BagConfig):
        self._config = config
        self._started = False

    @property
    def started(self) -> bool:
        return self._started

    def start(self) -> bool:
        if self._started:
            print("[ROS2 bag] start ignored: already marked started for this sample")
            return True
        cfg = self._config
        parent_bash = _bash_sq(cfg.bag_parent_dir)
        pid_file_bash = _bash_sq(PID_FILE)
        if cfg.record_all_topics:
            record_cmd = 'nohup ros2 bag record -a -o "$BAG_DIR"'
        else:
            topics_part = " ".join(_bash_sq(t) for t in cfg.topics)
            record_cmd = f'nohup ros2 bag record -o "$BAG_DIR" {topics_part}'

        # One line for ``bash -c`` avoids Windows CRLF/newline quirks in argv construction.
        script = (
            f"set -e; {cfg.setup_source_line}; "
            f"PARENT={parent_bash}; mkdir -p \"$PARENT\"; "
            "STAMP=$(date +%Y%m%d_%H%M%S); "
            'BAG_DIR="$PARENT/rosbag_$STAMP"; '
            'LOG_FILE="$PARENT/rosbag_$STAMP.log"; '
            f"PID_FILE={pid_file_bash}; "
            f'if [ -f "$PID_FILE" ] && [ -s "$PID_FILE" ]; then '
            f'oldpid=$(tr -d "[:space:]" < "$PID_FILE"); '
            f'if [ -n "$oldpid" ] && kill -0 "$oldpid" 2>/dev/null; then '
            f'echo "rosbag recorder already running PID=$oldpid"; exit 1; fi; '
            f'rm -f "$PID_FILE"; fi; '
            f'{record_cmd} > "$LOG_FILE" 2>&1 & echo $! > "$PID_FILE"; '
            f'echo "BAG_DIR=$BAG_DIR"; printf "PID="; cat "$PID_FILE"; echo'
        )

        code, out, err = _docker_exec_bash_c(cfg, script)
        if out:
            print(f"[ROS2 bag] {out}")
        if err:
            print(f"[ROS2 bag] stderr: {err}")
        if code != 0:
            logger.warning("ros2 bag start failed (code=%s)", code)
            print(f"[ROS2 bag] start failed with exit code {code}")
            return False
        self._started = True
        return True

    def stop(self) -> None:
        if not self._started:
            return
        cfg = self._config
        pid_file_bash = _bash_sq(PID_FILE)
        script = (
            "set +e; "
            f"PID_FILE={pid_file_bash}; "
            f'if [ ! -f "$PID_FILE" ] || [ ! -s "$PID_FILE" ]; then '
            'echo "no pid file"; exit 0; fi; '
            f'PID=$(tr -d "[:space:]" < "$PID_FILE"); '
            f'if [ -z "$PID" ]; then echo "empty pid file"; rm -f "$PID_FILE"; exit 0; fi; '
            f'if ! kill -0 "$PID" 2>/dev/null; then echo "stale pid $PID"; rm -f "$PID_FILE"; exit 0; fi; '
            f'kill -INT "$PID" 2>/dev/null || true; '
            "for _ in 1 2 3 4 5 6 7 8 9 10; do "
            f'if ! kill -0 "$PID" 2>/dev/null; then break; fi; sleep 0.2; done; '
            f'rm -f "$PID_FILE"; '
            f'echo "stopped PID=$PID"'
        )

        code, out, err = _docker_exec_bash_c(cfg, script)
        if out:
            print(f"[ROS2 bag] {out}")
        if err:
            print(f"[ROS2 bag] stderr: {err}")
        if code != 0:
            logger.warning("ros2 bag stop non-zero exit (code=%s)", code)
        self._started = False
