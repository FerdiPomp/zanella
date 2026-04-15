import threading
from queue import Queue
import time

import config as CONFIG
from engine.event import EVENTS, ITEMS, WORKSPACES, Event, EventXLayer
from engine.handler import ButtonPressHandler, LightHandler, ObjectDetectionHandler, QrHandler
from engine.network import HTTPReceiver, HTTPSender, MQTTSender
from engine.persistence import PersistentEventQueue
from engine.runtime_utils import print_log
from hardware.camera import SharedQRState


QR_EVENTS = ["QR_APPEND", "QR_REMOVED"]


class BaseNode:
    def __init__(self, node_id, queue_dir: str):
        self.node_id = node_id
        self.event_queue = PersistentEventQueue(queue_dir, self._serialize_event, self._deserialize_event)
        self.threads = []
        self.stop_event = threading.Event()

    def _serialize_event(self, event):
        return event.__dict__

    def _deserialize_event(self, payload):
        return Event(
            source=payload["source"],
            type=payload["type"],
            timestamp=payload["timestamp"],
            payload=payload.get("payload"),
            qr=payload.get("qr"),
        )

    def _start_network_thread(self, target):
        threading.Thread(target=target, daemon=True).start()

    def _start_worker_threads(self):
        for thread in self.threads:
            thread.start()

    def start(self):
        raise NotImplementedError

    def stop(self):
        self.stop_event.set()
        for thread in self.threads:
            thread.join()


class EdgeNode(BaseNode):
    def __init__(self, node_id):
        super().__init__(node_id, queue_dir=f".runtime/http_out_{node_id}")
        self.sender = HTTPSender() if CONFIG.ONLINE_SENDER else None
        self.led_queue = Queue()

    def _network_loop(self):
        if CONFIG.ONLINE_SENDER:
            while True:
                if not self.event_queue.wait_for_item(timeout=0.5):
                    continue
                token, event = self.event_queue.peek()
                if event is None:
                    continue
                delivery_id = token.stem if token is not None else None
                if self.sender.send(event, delivery_id=delivery_id):
                    self.event_queue.ack(token)
                else:
                    time.sleep(1)
        else:
            while True:
                if not self.event_queue.wait_for_item(timeout=0.5):
                    continue
                token, event = self.event_queue.peek()
                if event is None:
                    continue
                print(event)
                self.event_queue.ack(token)

    def _append_object_thread(self, event_type, file_bag):
        self.threads.append(
            threading.Thread(
                target=ObjectDetectionHandler,
                args=(self.stop_event, self.event_queue, self.node_id, event_type, self.led_queue, file_bag),
                daemon=False,
            )
        )

    def _append_led_thread_if_enabled(self):
        if CONFIG.THERE_IS_LED:
            self.threads.append(
                threading.Thread(
                    target=LightHandler,
                    args=(self.stop_event, self.led_queue),
                    daemon=False,
                )
            )


class EnterNode(EdgeNode):
    def start(self, file_bag: str = None):
        self._append_object_thread("ENTER_DETECT", file_bag)
        self._append_led_thread_if_enabled()
        self._start_network_thread(self._network_loop)
        self._start_worker_threads()


class ExitNode(EdgeNode):
    def _append_button_thread_if_enabled(self):
        if CONFIG.THERE_IS_BUTTON:
            self.threads.append(
                threading.Thread(
                    target=ButtonPressHandler,
                    args=(self.stop_event, self.event_queue, self.node_id, self.led_queue),
                    daemon=False,
                )
            )

    def start(self, file_bag: str = None):
        self._append_object_thread("EXIT_DETECT", file_bag)
        self._append_led_thread_if_enabled()
        self._append_button_thread_if_enabled()
        self._start_network_thread(self._network_loop)
        self._start_worker_threads()


class EnvNode(BaseNode):
    def __init__(self, node_id, workspace: str, broker_psw, topic):
        super().__init__(node_id, queue_dir=f".runtime/http_in_{node_id}")
        self.sender = MQTTSender(broker_psw, topic) if CONFIG.ONLINE_SENDER_ENV else None
        self.receiver = HTTPReceiver() if CONFIG.ONLINE_RECIEVER else None
        self.shared_qr_state = SharedQRState()
        self.workspace = workspace

    def _resolve_qr_for_event(self, event: Event):
        qr, qr_time = self.shared_qr_state.get()
        if event.timestamp < qr_time:
            print_log("yes")
            qr = None
            prev_qr, prev_time = self.shared_qr_state.get_prev()
            for index in range(len(prev_time)):
                if event.timestamp > prev_time[-index - 1]:
                    qr = prev_qr[-index - 1]
                    break
        return qr

    def _build_event(self, event: Event):
        # NOTE: se arriva un evento prima che sia rilevato il cambio di qr, si è fottuti.
        if event.type not in QR_EVENTS:
            qr = self._resolve_qr_for_event(event)
        else:
            qr = event.qr

        return EventXLayer(
            workplace=WORKSPACES[self.workspace],
            event_id=EVENTS[event.type],
            mes_data=qr,
            good_items=ITEMS[event.type][0],  # puo essere molto meno complicato di cosi, basta mettere if .. in [..]
            bad_items=ITEMS[event.type][1],  # qui è sempre 0
            timestamp=event.timestamp,
        )

    def _emit_event(self, event: Event):
        new_event = self._build_event(event)
        if CONFIG.ONLINE_SENDER_ENV:
            self.sender.send(new_event)
            return True
        else:
            print(new_event)
            return True

    def _network_loop(self):
        while True:
            if not self.event_queue.wait_for_item(timeout=0.5):
                continue
            token, event = self.event_queue.peek()
            if event is None:
                continue
            try:
                if self._emit_event(event):
                    self.event_queue.ack(token)
            except Exception as error:
                print_log(f"EnvNode emit failure: {error}")
                time.sleep(1)

    def start(self, file_bag: str = None):
        if CONFIG.ONLINE_RECIEVER:
            self.receiver.start(self.event_queue)

        self.threads.append(
            threading.Thread(
                target=QrHandler,
                args=(self.stop_event, self.event_queue, self.node_id, self.shared_qr_state, file_bag),
                daemon=False,
            )
        )

        self._start_network_thread(self._network_loop)
        self._start_worker_threads()
