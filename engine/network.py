import json
import threading
import time
from pathlib import Path

import config as CONFIG
from engine.event import Event
from engine.runtime_utils import print_log

HTTP_EVENT_ID_HEADER = "Event-ID"


def _event_from_dict(data):
    return Event(
        source=data["source"],
        type=data["type"],
        timestamp=data["timestamp"],
        payload=data.get("payload"),
    )


def _event_to_payload(event: Event):
    return {
        "source": event.source,
        "type": event.type,
        "timestamp": event.timestamp,
        "payload": event.payload,
    }


class MQTTSender:
    def __init__(self, broker_psw, topic):
        import paho.mqtt.client as mqtt

        self.topic = topic
        self.client = mqtt.Client(
            transport="websockets",
            protocol=mqtt.MQTTv311,
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
        )
        self.client.username_pw_set("univrcameras", broker_psw)
        self.client.tls_set(ca_certs="certificate.crt")
        self.client.connect(CONFIG.BROKER_IP, CONFIG.MQTT_PORT)
        self.client.loop_start()
        time.sleep(0.5)

    def send(self, event):
        # TODO: validate json before sending
        print(event)
        payload = json.dumps(event.__dict__)
        self.client.publish(
            self.topic,
            payload=payload,
            qos=1,
        )


class HTTPSender:
    def __init__(self, server_ip=CONFIG.SERVER_URL, port=CONFIG.SERVER_PORT, endpoint="/event", timeout=1):
        import requests

        self.requests = requests
        self.session = self.requests.Session()
        self.server_ip = server_ip
        self.port = port
        self.endpoint = endpoint
        self.timeout = timeout
        self.url = f"http://{self.server_ip}:{self.port}{self.endpoint}"

    def send(self, event: Event, delivery_id: str = None) -> bool:
        """
        Invia l'evento al server.
        Un singolo tentativo: il retry viene gestito a livello del nodo.
        """
        print(f"Sending:{event}")
        payload = _event_to_payload(event)
        headers = {}
        if delivery_id is not None:
            headers[HTTP_EVENT_ID_HEADER] = delivery_id

        return self._post(payload, headers)

    def send_work_state(self, work_state: bool) -> bool:
        return self._post({"work_state": work_state})

    def _post(self, payload, headers=None) -> bool:
        try:
            response = self.session.post(self.url, json=payload, headers=headers or {}, timeout=self.timeout)
            return response.status_code == 200
        except self.requests.RequestException:
            try:
                self.session.close()
            except Exception:
                pass
            self.session = self.requests.Session()
            return False


class HTTPReceiver:
    def __init__(self, host=CONFIG.SERVER_URL, port=CONFIG.SERVER_PORT):
        from flask import Flask, jsonify, request
        from werkzeug.serving import make_server

        self.Flask = Flask
        self.jsonify = jsonify
        self.request = request
        self.make_server = make_server
        self.host = host
        self.port = port
        self.app = self.Flask(__name__)
        self.server = None
        self.thread = None
        self.stop_event = threading.Event()
        self.seen_dir = Path(".runtime/http_seen")
        self.seen_dir.mkdir(parents=True, exist_ok=True)

    def _is_duplicate(self, delivery_id: str) -> bool:
        if delivery_id is None:
            return False
        return (self.seen_dir / delivery_id).exists()

    def _mark_seen(self, delivery_id: str) -> None:
        if delivery_id is None:
            return
        (self.seen_dir / delivery_id).touch(exist_ok=True)

    def _register_routes(self, event_queue=None, work_state=None):
        if event_queue is not None:
            @self.app.route("/event", methods=["POST"])
            def receive_event():
                data = self.request.json
                delivery_id = self.request.headers.get(HTTP_EVENT_ID_HEADER)
                try:
                    if self._is_duplicate(delivery_id):
                        return self.jsonify({"status": "ok"})
                    if hasattr(event_queue, "contains_item_id") and event_queue.contains_item_id(delivery_id):
                        return self.jsonify({"status": "ok"})
                    event_queue.put(_event_from_dict(data), item_id=delivery_id)
                    self._mark_seen(delivery_id)
                    return self.jsonify({"status": "ok"})
                except Exception as e:
                    return self.jsonify({"status": "error", "msg": str(e)}), 400

        if work_state is not None:
            @self.app.route("/work_state", methods=["POST"])
            def receive_work_state():
                value = self.request.json.get("work_state")
                if not isinstance(value, bool):
                    return self.jsonify({"status": "error", "msg": "work_state must be boolean"}), 400
                work_state.changeState(value)
                return self.jsonify({"status": "ok"})

    def _serve_forever(self):
        while not self.stop_event.is_set():
            try:
                self.server = self.make_server(self.host, self.port, self.app, threaded=True)
                if self.stop_event.is_set():
                    self.server.server_close()
                    return
                self.server.serve_forever()
            except OSError:
                time.sleep(1)
            except Exception as error:
                print_log(f"HTTPReceiver failure: {error}")
                time.sleep(1)

    def start(self, event_queue=None, work_state=None):
        """
        Avvia il server Flask in un thread separato
        e riempie la queue con Event.
        """
        self._register_routes(event_queue, work_state)
        self.thread = threading.Thread(
            target=self._serve_forever,
            daemon=True,
        )
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.server is not None:
            self.server.shutdown()
            self.server.server_close()
        if self.thread is not None:
            self.thread.join()
