import requests
import queue
import socket
import json
import threading
import time
import paho.mqtt.client as mqtt
from flask import Flask, request, jsonify

from engine.event import Event

import config as CONFIG

class USBReceiver:
    def __init__(self, host=CONFIG.SERVER_URL, port=CONFIG.SERVER_PORT):
        self.host = host
        self.port = port

    def start(self, event_queue):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind((self.host, self.port))
        server.listen(5)

        def client_handler(conn):
            with conn:
                buffer = ""
                while True:
                    data = conn.recv(1024)
                    if not data:
                        break

                    buffer += data.decode()
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)

                        data = json.loads(line)
                        event = Event(
                            source=data["source"],
                            type=data["type"],
                            timestamp=data["timestamp"],
                            payload=data.get("payload")
                        )
                        event_queue.put(event)

        def accept_loop():
            while True:
                conn, _ = server.accept()
                threading.Thread(
                    target=client_handler,
                    args=(conn,),
                    daemon=True
                ).start()

        threading.Thread(target=accept_loop, daemon=True).start()

class USBSender:
    def __init__(self, server_ip=CONFIG.SERVER_URL, port=CONFIG.SERVER_PORT):
        self.server_ip = server_ip
        self.port = port
        self.socket = None

    def _connect(self):
        while True:
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.connect((self.server_ip, self.port))
                return
            except Exception:
                time.sleep(1)

    def send(self, event):
        if self.socket is None:
            self._connect()

        try:
            msg = json.dumps(event.__dict__) + "\n"
            self.socket.sendall(msg.encode())
        except Exception:
            self.socket.close()
            self.socket = None
            self._connect()

class MQTTSender:
    def __init__(self, broker_psw, topic):
        self.topic = topic
        self.client = mqtt.Client()
        client.username_pw_set("univrcameras", broker_psw)
        client.tls_set(ca_certs="certificate.crt")
        self.client.connect(CONFIG.BROKER_IP, CONFIG.MQTT_PORT)
        self.client.loop_start()

    def send(self, event):
        #TODO: validate json before sending
        payload = json.dumps(event.__dict__)
        self.client.publish(
            self.topic,
            payload=payload,
            qos=1
        )

class HTTPSender:
    def __init__(self, server_ip=CONFIG.SERVER_URL, port=CONFIG.SERVER_PORT, endpoint="/event", timeout=1):
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
        payload = {
            "source": event.source,
            "type": event.type,
            "timestamp": event.timestamp,
            "payload": event.payload
        }

        while True:
            try:
                r = requests.post(self.url, json=payload, timeout=self.timeout)
                if r.status_code == 200:
                    return
            except requests.RequestException:
                pass  # ignore e retry
            time.sleep(1)
        
class HTTPReceiver:
    def __init__(self, host=CONFIG.SERVER_URL, port=CONFIG.SERVER_PORT):
        self.host = host
        self.port = port
        self.app = Flask(__name__)

    def start(self, event_queue):
        """
        Avvia il server Flask in un thread separato
        e riempie la queue con Event.
        """
        @self.app.route("/event", methods=["POST"])
        def receive_event():
            data = request.json
            try:
                event = Event(
                    source=data["source"],
                    type=data["type"],
                    timestamp=data["timestamp"],
                    payload=data.get("payload")
                )
                event_queue.put(event)
                return jsonify({"status": "ok"})
            except Exception as e:
                return jsonify({"status": "error", "msg": str(e)}), 400

        threading.Thread(
            target=lambda: self.app.run(host=self.host, port=self.port),
            daemon=True
        ).start()