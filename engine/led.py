import errno
import json
import math
import os
import queue
import socket
import stat
import threading
import time
from pathlib import Path

from engine.runtime_utils import print_log
from hardware.gpio import Light


BLINK_DURATION = 9.0
BLINK_HALF_PERIOD = 0.3
REFRESH_INTERVAL = 1.0
SOURCE_TIMEOUT = 3.0


class LedState:
    def __init__(self):
        self.states = {}

    def set(self, source: str, command: str, now=None):
        now = time.monotonic() if now is None else now
        if command == "off":
            self.states.pop(source, None)
        elif command == "on":
            self.states[source] = {"mode": "on"}
        elif command == "blink":
            self.states[source] = {"mode": "blink", "until": now + BLINK_DURATION}
        else:
            raise ValueError(f"Unknown LED command: {command}")

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        self.states = {
            source: state for source, state in self.states.items()
            if state["mode"] == "on" or state["until"] > now
        }
        return {source: dict(state) for source, state in self.states.items()}

    def mode(self, now=None):
        states = self.snapshot(now).values()
        if any(state["mode"] == "blink" for state in states):
            return "blink"
        if any(state["mode"] == "on" for state in states):
            return "on"
        return "off"


class LedOutput:
    def __init__(self, light):
        self.light = light
        self.mode = "off"
        self.is_on = False
        self.blink_started = None

    def apply(self, mode: str, now=None):
        now = time.monotonic() if now is None else now
        if mode != self.mode:
            self.mode = mode
            if mode == "blink":
                self.blink_started = now

        should_be_on = mode == "on" or (
            mode == "blink" and int((now - self.blink_started) / BLINK_HALF_PERIOD) % 2 == 0
        )
        if should_be_on != self.is_on:
            if should_be_on:
                self.light.on()
            else:
                self.light.off()
            self.is_on = should_be_on


class LedArbiter:
    def __init__(self):
        self.nodes = {}

    def update(self, payload, now=None):
        now = time.monotonic() if now is None else now
        if not isinstance(payload, dict) or payload.get("node") not in {"A", "B"}:
            raise ValueError("LED message must identify node A or B")
        states = payload.get("states")
        if not isinstance(states, dict):
            raise ValueError("LED states must be an object")

        accepted = {}
        for source, state in states.items():
            if source not in {"object", "button"} or (source == "button" and payload["node"] != "B"):
                raise ValueError("Unknown LED source")
            if not isinstance(state, dict):
                raise ValueError("Invalid LED state")
            if state.get("mode") == "on" and set(state) == {"mode"}:
                accepted[source] = state
            elif state.get("mode") == "blink" and set(state) == {"mode", "until"}:
                until = state["until"]
                if isinstance(until, (int, float)) and not isinstance(until, bool) and math.isfinite(until):
                    if until > now:
                        accepted[source] = {"mode": "blink", "until": min(until, now + BLINK_DURATION)}
                else:
                    raise ValueError("Invalid LED blink deadline")
            else:
                raise ValueError("Invalid LED mode")

        self.nodes[payload["node"]] = (accepted, now)

    def mode(self, now=None):
        now = time.monotonic() if now is None else now
        on = False
        for states, updated_at in self.nodes.values():
            if now - updated_at > SOURCE_TIMEOUT:
                continue
            for state in states.values():
                if state["mode"] == "blink" and state["until"] > now:
                    return "blink"
                if state["mode"] == "on":
                    on = True
        return "on" if on else "off"


def SharedLedHandler(stop_event, led_queue, node_id: str, socket_path: str):
    state = LedState()
    failed = False
    next_send = 0.0
    with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as client:
        while not stop_event.is_set():
            now = time.monotonic()
            try:
                source, command = led_queue.get(timeout=max(0.0, next_send - now))
                state.set(source, command)
            except queue.Empty:
                pass

            now = time.monotonic()
            if now < next_send and not led_queue.empty():
                continue
            payload = {"node": node_id, "states": state.snapshot(now)}
            try:
                client.sendto(json.dumps(payload).encode("utf-8"), socket_path)
                if failed:
                    print_log(f"LED controller connection restored on node {node_id}")
                failed = False
            except OSError as error:
                if not failed:
                    print_log(f"LED controller unavailable on node {node_id}: {error}")
                failed = True
            next_send = now + REFRESH_INTERVAL

        try:
            client.sendto(json.dumps({"node": node_id, "states": {}}).encode("utf-8"), socket_path)
        except OSError:
            pass


class LedController:
    def __init__(self, socket_path: str):
        self.path = Path(socket_path)
        self.socket = None
        self.thread = None
        self.stop_event = threading.Event()
        self.arbiter = LedArbiter()
        self.light = None
        self.socket_inode = None

    def _unlink_own_socket(self):
        if self.socket_inode is None:
            return
        try:
            if self.path.lstat().st_ino == self.socket_inode:
                self.path.unlink()
        except FileNotFoundError:
            pass
        self.socket_inode = None

    def _clear_stale_socket(self):
        try:
            info = self.path.lstat()
        except FileNotFoundError:
            return
        if not stat.S_ISSOCK(info.st_mode) or info.st_uid != os.geteuid():
            raise RuntimeError(f"LED socket path is occupied: {self.path}")
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as probe:
            try:
                probe.connect(str(self.path))
            except OSError as error:
                if error.errno not in {errno.ECONNREFUSED, errno.ENOENT}:
                    raise
            else:
                raise RuntimeError(f"LED controller is already running at {self.path}")
        self.path.unlink()

    def start(self, file_bag=None):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._clear_stale_socket()
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        bound = False
        try:
            self.socket.bind(str(self.path))
            bound = True
            self.socket_inode = self.path.lstat().st_ino
            self.socket.settimeout(0.1)
            self.light = Light()
        except Exception:
            self.socket.close()
            self.socket = None
            if bound:
                self._unlink_own_socket()
            raise
        self.thread = threading.Thread(target=self._loop, daemon=False)
        self.thread.start()

    def _loop(self):
        output = LedOutput(self.light)
        try:
            while not self.stop_event.is_set():
                try:
                    data = self.socket.recv(4096)
                    self.arbiter.update(json.loads(data))
                except socket.timeout:
                    pass
                except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as error:
                    print_log(f"Invalid LED message: {error}")
                output.apply(self.arbiter.mode())
        except OSError as error:
            if not self.stop_event.is_set():
                print_log(f"LED controller failure: {error}")
        finally:
            self.light.off()

    def stop(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join()
        try:
            if self.light is not None:
                try:
                    self.light.off()
                finally:
                    self.light.close()
                    self.light = None
        finally:
            if self.socket is not None:
                self.socket.close()
                self.socket = None
            self._unlink_own_socket()
