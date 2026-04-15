import json
import os
import threading
import time
import uuid
from pathlib import Path
from queue import Empty, Queue


class PersistentEventQueue:
    def __init__(self, spool_dir: str, serializer, deserializer):
        self.spool_dir = Path(spool_dir)
        self.spool_dir.mkdir(parents=True, exist_ok=True)
        self.serializer = serializer
        self.deserializer = deserializer
        self._lock = threading.Lock()
        self._notifications = Queue()

    def _list_entries(self):
        return sorted(self.spool_dir.glob("*.json"))

    def contains_item_id(self, item_id: str) -> bool:
        if item_id is None:
            return False
        return (self.spool_dir / f"{item_id}.json").exists()

    def put(self, item, item_id: str = None):
        payload = self.serializer(item)
        filename = f"{item_id}.json" if item_id is not None else f"{time.time_ns()}_{uuid.uuid4().hex}.json"
        target = self.spool_dir / filename
        temp_target = self.spool_dir / f".{filename}.tmp"

        with self._lock:
            if item_id is not None and target.exists():
                return False
            with temp_target.open("w", encoding="utf-8") as handle:
                json.dump(payload, handle)
                handle.flush()
                os.fsync(handle.fileno())
            temp_target.replace(target)

        self._notifications.put(target.name)
        return True

    def has_pending(self) -> bool:
        return any(self.spool_dir.glob("*.json"))

    def wait_for_item(self, timeout: float = 0.5) -> bool:
        if self.has_pending():
            return True

        try:
            self._notifications.get(timeout=timeout)
            return True
        except Empty:
            return self.has_pending()

    def peek(self):
        entries = self._list_entries()
        if not entries:
            return None, None

        path = entries[0]
        with path.open("r", encoding="utf-8") as handle:
            payload = json.load(handle)
        return path, self.deserializer(payload)

    def ack(self, token):
        if token is None:
            return
        try:
            token.unlink()
        except FileNotFoundError:
            pass
