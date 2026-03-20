from __future__ import annotations

import json
import socket
import threading
import time


class SyncStepBridge:
    def __init__(self, host="127.0.0.1", port=50555):
        self.host = host
        self.port = int(port)

        self._server = None
        self._conn = None
        self._thread = None
        self._closed = False

        self._cv = threading.Condition()
        self._connected = False

        self._ready_count = 0       # latest TIME_TRIGGER that is blocked and waiting
        self._released_count = 0    # latest TIME_TRIGGER Scenic has released

    def start(self):
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind((self.host, self.port))
        self._server.listen(1)

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def close(self):
        with self._cv:
            self._closed = True
            self._cv.notify_all()
        try:
            if self._conn:
                self._conn.close()
        except Exception:
            pass
        try:
            if self._server:
                self._server.close()
        except Exception:
            pass

    def wait_connected(self, timeout=None):
        end = None if timeout is None else time.time() + timeout
        with self._cv:
            while not self._connected and not self._closed:
                remaining = None if end is None else max(0.0, end - time.time())
                if remaining == 0.0:
                    raise TimeoutError("SyncStepBridge: wait_connected timed out")
                self._cv.wait(remaining)
            if not self._connected:
                raise RuntimeError("SyncStepBridge closed before connection")

    def wait_until_ready(self, after_count=0, timeout=None):
        end = None if timeout is None else time.time() + timeout
        with self._cv:
            while self._ready_count <= after_count and not self._closed:
                remaining = None if end is None else max(0.0, end - time.time())
                if remaining == 0.0:
                    raise TimeoutError(
                        f"SyncStepBridge: wait_until_ready timed out after_count={after_count}"
                    )
                self._cv.wait(remaining)
            if self._closed:
                raise RuntimeError("SyncStepBridge closed")
            return self._ready_count

    def release_step(self, count=None):
        with self._cv:
            if count is None:
                count = self._ready_count
            if count <= 0:
                raise RuntimeError("SyncStepBridge: no blocked TIME_TRIGGER to release")
            self._released_count = max(self._released_count, count)
            self._cv.notify_all()

    def step(self, timeout=None):
        # Wait until VEOS is blocked and ready for one Scenic step.
        current = self.wait_until_ready(after_count=self._released_count, timeout=timeout)

        # Release exactly this blocked trigger.
        self.release_step(current)

        # Wait until VEOS advances and blocks again at the next trigger.
        next_count = self.wait_until_ready(after_count=current, timeout=timeout)
        return next_count

    def _run(self):
        conn, addr = self._server.accept()
        with self._cv:
            self._conn = conn
            self._connected = True
            self._cv.notify_all()

        buf = b""
        with conn:
            while not self._closed:
                chunk = conn.recv(4096)
                if not chunk:
                    break
                buf += chunk

                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip()
                    if not line:
                        continue

                    text = line.decode("utf-8", errors="replace")

                    try:
                        obj = json.loads(text)
                    except json.JSONDecodeError:
                        # For malformed non-step messages, just acknowledge and continue.
                        conn.sendall(b"ACK\n")
                        continue

                    event = obj.get("event")

                    if event == "TIME_TRIGGER":
                        count = int(obj["count"])

                        with self._cv:
                            self._ready_count = max(self._ready_count, count)
                            self._cv.notify_all()

                            while self._released_count < count and not self._closed:
                                self._cv.wait()

                        if self._closed:
                            return

                        # This is the actual release of one VEOS step.
                        conn.sendall(b"STEP\n")
                    else:
                        # Logs / other messages do not control stepping.
                        conn.sendall(b"ACK\n")