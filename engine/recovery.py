import json
import os
from datetime import datetime, timedelta
from pathlib import Path

import config as CONFIG
from engine.runtime_utils import print_log


class PlaneStore:
    def __init__(self, node_id: str):
        self.path = Path(f".runtime/table_plane_{node_id}.json")

    def load(self):
        try:
            with self.path.open("r", encoding="utf-8") as handle:
                plane = json.load(handle)["plane"]
            if not isinstance(plane, list) or len(plane) != 4:
                raise ValueError("invalid plane format")
            return [float(value) for value in plane]
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError):
            return None

    def save(self, plane) -> bool:
        if plane is None:
            return False

        temp_path = self.path.with_suffix(".tmp")
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            payload = {"plane": [float(value) for value in plane]}
            with temp_path.open("w", encoding="utf-8") as handle:
                json.dump(payload, handle)
                handle.flush()
                os.fsync(handle.fileno())
            temp_path.replace(self.path)
            return True
        except (OSError, TypeError, ValueError) as error:
            print_log(f"Night recovery plane save failure: {error}")
            try:
                temp_path.unlink()
            except OSError:
                pass
            return False


class NightRecovery:
    def __init__(self, node_id: str, event_queue, work_state):
        self.node_id = node_id
        self.event_queue = event_queue
        self.work_state = work_state
        self.plane_store = PlaneStore(node_id)
        self.started_at = datetime.now()
        self.last_run_date = None

    def load_plane(self):
        return self.plane_store.load()

    def _started_during_recovery_window(self, now) -> bool:
        recovery_start = now.replace(
            hour=CONFIG.NIGHT_RECOVERY_HOUR,
            minute=CONFIG.NIGHT_RECOVERY_MINUTE,
            second=0,
            microsecond=0,
        )
        recovery_end = recovery_start + timedelta(seconds=CONFIG.NIGHT_RECOVERY_DURATION_SECONDS)
        return recovery_start <= self.started_at < recovery_end

    def should_run(self, now=None) -> bool:
        if not CONFIG.NIGHT_RECOVERY_ENABLED:
            return False

        now = now or datetime.now()
        if self.last_run_date == now.date():
            return False
        if self.started_at.date() == now.date() and self._started_during_recovery_window(now):
            return False
        return now.hour == CONFIG.NIGHT_RECOVERY_HOUR and now.minute == CONFIG.NIGHT_RECOVERY_MINUTE

    def start(self, camera) -> None:
        now = datetime.now()
        self.last_run_date = now.date()

        if self.work_state.get():
            print_log(f"Night recovery anomaly on node {self.node_id}: work_state was True")
        self.work_state.changeState(False)

        try:
            discarded = self.event_queue.discard_all()
            if discarded > 0:
                print_log(f"Night recovery discarded {discarded} pending event(s) on node {self.node_id}")
        except OSError as error:
            print_log(f"Night recovery queue purge failure on node {self.node_id}: {error}")

        if camera is not None:
            if self.plane_store.save(getattr(camera, "plane", None)):
                print_log(f"Night recovery saved table plane on node {self.node_id}")
            camera.close()

    def wait_for_reopen(self, stop_event) -> bool:
        return not stop_event.wait(CONFIG.NIGHT_RECOVERY_DURATION_SECONDS)
