"""Per-frame RealSense acquisition trace for investigating stalled streams.

The trace is observational: a logging failure must never change detection.
"""

from datetime import datetime
import hashlib
import json
import os
import time
import uuid

from engine.runtime_utils import print_log, runtime_path


def _safe_log(message):
    try:
        print_log(message)
    except OSError:
        pass


def device_details(rs, device):
    """Read device identity without assuming every camera exposes every field."""
    details = {}
    camera_info = getattr(rs, "camera_info", None)
    for name in ("serial_number", "firmware_version", "usb_type_descriptor"):
        key = getattr(camera_info, name, None)
        if key is None:
            continue
        try:
            if device.supports(key):
                details[name] = device.get_info(key)
        except Exception as error:
            details[f"{name}_error"] = str(error)
    return details


def _frame_details(frame, data):
    if not frame:
        return None

    details = {}
    for field, method in (
        ("number", "get_frame_number"),
        ("timestamp_ms", "get_timestamp"),
        ("timestamp_domain", "get_frame_timestamp_domain"),
    ):
        try:
            value = getattr(frame, method)()
            details[field] = str(value) if field == "timestamp_domain" else value
        except Exception as error:
            details[field] = None
            details[f"{field}_error"] = str(error)

    if data is not None:
        try:
            details["digest_sha256"] = hashlib.sha256(memoryview(data)).hexdigest()
        except Exception as error:
            details["digest_error"] = str(error)
    return details


class RealSenseFrameDiagnostics:
    """Append one JSON record per frameset to a daily, flushed trace file."""

    def __init__(self, node_id, details, playback=False):
        self.node_id = node_id
        self.session = uuid.uuid4().hex
        self.path = None
        self._day = None
        self._handle = None
        self._disabled = False
        self._frame_index = 0
        self._previous_depth = None
        self._previous_color = None
        self._depth_repeat_count = 0
        self._warning_emitted = False
        self.record_event("camera_open", device=details, playback=playback)

    def _write(self, record):
        if self._disabled:
            return
        now = datetime.now().astimezone()
        day = now.strftime("%Y%m%d")
        try:
            if self._day != day:
                if self._handle is not None:
                    self._handle.close()
                self.path = runtime_path(".runtime", f"realsense_frames_{day}.jsonl")
                self.path.parent.mkdir(parents=True, exist_ok=True)
                self._handle = self.path.open("a", encoding="utf-8", buffering=1)
                self._day = day
            row = {
                "host_time": now.isoformat(timespec="milliseconds"),
                "host_monotonic_ns": time.monotonic_ns(),
                "pid": os.getpid(),
                "node": self.node_id,
                "session": self.session,
                **record,
            }
            self._handle.write(json.dumps(row, separators=(",", ":"), default=str) + "\n")
            self._handle.flush()
        except OSError as error:
            self._disabled = True
            _safe_log(f"RealSense diagnostics unavailable: {error}")
            self.close()

    def record_event(self, event, **fields):
        self._write({"event": event, **fields})

    def reset_frame_history(self):
        """Avoid comparing new pipeline frames with a previous stream session."""
        self._previous_depth = None
        self._previous_color = None
        self._depth_repeat_count = 0
        self._warning_emitted = False

    def record_frame(self, depth_frame, color_frame, depth_data, color_data, wait_ms,
                     acquired_host_time=None, acquired_monotonic_ns=None,
                     color_frame_error=None, color_data_error=None):
        self._frame_index += 1
        if self._disabled:
            return self._frame_index
        depth = _frame_details(depth_frame, depth_data)
        color = _frame_details(color_frame, color_data)

        previous_depth = self._previous_depth
        previous_color = self._previous_color
        same_depth_frame = (
            depth is not None
            and previous_depth is not None
            and depth.get("number") is not None
            and depth.get("timestamp_ms") is not None
            and depth["number"] == previous_depth.get("number")
            and depth["timestamp_ms"] == previous_depth.get("timestamp_ms")
            and depth.get("timestamp_domain") == previous_depth.get("timestamp_domain")
        )
        same_depth_payload = (
            depth is not None
            and previous_depth is not None
            and depth.get("digest_sha256") is not None
            and depth["digest_sha256"] == previous_depth.get("digest_sha256")
        )
        color_advanced = (
            color is not None
            and previous_color is not None
            and color.get("number") is not None
            and previous_color.get("number") is not None
            and color["number"] != previous_color["number"]
        )
        self._depth_repeat_count = self._depth_repeat_count + 1 if same_depth_frame else 0
        self._write({
            "event": "frame",
            "frame_index": self._frame_index,
            "acquired_host_time": acquired_host_time,
            "acquired_monotonic_ns": acquired_monotonic_ns,
            "wait_ms": round(wait_ms, 3),
            "depth": depth,
            "color": color,
            "color_frame_error": color_frame_error,
            "color_data_error": color_data_error,
            "same_depth_frame": bool(same_depth_frame),
            "same_depth_payload": bool(same_depth_payload),
            "color_advanced": bool(color_advanced),
            "depth_repeat_count": self._depth_repeat_count,
        })
        if self._depth_repeat_count >= 5 and not self._warning_emitted:
            self._warning_emitted = True
            _safe_log(f"RealSense depth frame repeated; trace: {self.path}")
        elif self._warning_emitted and not same_depth_frame:
            self._warning_emitted = False
            _safe_log(f"RealSense depth frame resumed; trace: {self.path}")

        self._previous_depth = depth
        self._previous_color = color
        return self._frame_index

    def close(self):
        if self._handle is not None:
            try:
                self._handle.close()
            except OSError as error:
                _safe_log(f"RealSense diagnostics close failure: {error}")
            self._handle = None
