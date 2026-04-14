import json
import threading
import time

import config as CONFIG
from engine.event import Event
from engine.runtime_utils import print_log


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
        self.server_ip = server_ip
        self.port = port
        self.endpoint = endpoint
        self.timeout = timeout
        self.url = f"http://{self.server_ip}:{self.port}{self.endpoint}"

    def send(self, event: Event):
        """
        Invia l'evento al server.
        Retry automatico se il server non risponde.
        """
        print(f"Sending:{event}")
        payload = _event_to_payload(event)

        while True:
            try:
                response = self.requests.post(self.url, json=payload, timeout=self.timeout)
                if response.status_code == 200:
                    return
            except self.requests.RequestException:
                pass  # ignore e retry
            time.sleep(1)


class HTTPReceiver:
    def __init__(self, host=CONFIG.SERVER_URL, port=CONFIG.SERVER_PORT):
        from flask import Flask, jsonify, request

        self.Flask = Flask
        self.jsonify = jsonify
        self.request = request
        self.host = host
        self.port = port
        self.app = self.Flask(__name__)

    def _register_routes(self, event_queue):
        @self.app.route("/event", methods=["POST"])
        def receive_event():
            data = self.request.json
            try:
                event_queue.put(_event_from_dict(data))
                return self.jsonify({"status": "ok"})
            except Exception as e:
                return self.jsonify({"status": "error", "msg": str(e)}), 400

    def start(self, event_queue):
        """
        Avvia il server Flask in un thread separato
        e riempie la queue con Event.
        """
        self._register_routes(event_queue)
        threading.Thread(
            target=lambda: self.app.run(host=self.host, port=self.port),
            daemon=True,
        ).start()
