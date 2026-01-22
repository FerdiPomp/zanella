import threading
from queue import Queue
import time

from engine.event import Event, EventXLayer, EVENTS, WORKSPACES, ITEMS
from hardware.camera import SharedQRState
from engine.handler import ObjectDetectionHandler, ButtonPressHandler, QrHandler
from engine.network import USBSender, USBReceiver, MQTTSender

ONLINE_SENDER = False
ONLINE_RECIEVER = False


class BaseNode:
    def __init__(self, node_id):
        self.node_id = node_id
        self.event_queue = Queue()

    def start(self):
        raise NotImplementedError

class EnterNode(BaseNode):
    def __init__(self, node_id):
        super().__init__(node_id)
        self.sender = USBSender()
    def _network_loop(self):
        if ONLINE_SENDER:
            while True:
                event = self.event_queue.get()
                self.sender.send(event)
        else:
            while True:
                event = self.event_queue.get()
                print(event)


    def start(self, file_bag:str=None, led_pin : int = None):
        threading.Thread(
                target=ObjectDetectionHandler,
                args=(self.event_queue, self.node_id, 'ENTER_DETECT', file_bag, led_pin),
                daemon=True
            ).start()

        threading.Thread(
            target=self._network_loop,
            daemon=True
        ).start()

class ExitNode(BaseNode):
    def __init__(self, node_id):
        super().__init__(node_id)
        self.sender = USBSender()
    
    def _network_loop(self):
        if ONLINE_SENDER:
            while True:
                event = self.event_queue.get()
                self.sender.send(event)
        else:
            while True:
                event = self.event_queue.get()
                print(event)

    def start(self, file_bag:str=None, led_pin : int = None, button_pin:int = None):
        threading.Thread(
                target=ObjectDetectionHandler,
                args=(self.event_queue, self.node_id, 'EXIT_DETECT', file_bag, led_pin),
                daemon=True
            ).start()
        
        threading.Thread(
                target=ButtonPressHandler,
                args=(self.event_queue, self.node_id, button_pin),
                daemon=True
            ).start()
        
        threading.Thread(
            target=self._network_loop,
            daemon=True
        ).start()

class EnvNode(BaseNode):
    def __init__(self, node_id, workspace:str, broker_ip, topic ):
        super().__init__(node_id)
        if ONLINE_SENDER:
            self.sender = MQTTSender(broker_ip, topic)
        self.receiver = USBReceiver()
        self.shared_qr_state = SharedQRState()
        self.workspace = workspace

    def _build_event(self, event: Event):

        #NOTE: se arriva un evento prima che sia rilevato il cambio di qr, si è fottuti.
        if not(event.type in ['QR_APPEND', 'QR_REMOVED']):
            qr, qr_time = self.shared_qr_state.get()
            if event.timestamp < qr_time:
                print('yes')
                qr = None
                prev_qr, prev_time = self.shared_qr_state.get_prev()
                for i in range(len(prev_time)):
                    if event.timestamp > prev_time[-i-1]:
                        qr = prev_qr[-i-1]
                        break
        else:
            qr = event.qr

        new_event = EventXLayer(workspace = WORKSPACES[self.workspace],
                                event_id = EVENTS[event.type],
                                mes_data = qr, 
                                good_items= ITEMS[event.type][0], #puo essere molto meno complicato di cosi, basta mettere if .. in [..]
                                bad_items= ITEMS[event.type][1],  #qui è sempre 0
                                timestamp= event.timestamp)

        return new_event

    def _network_loop(self):
        if ONLINE_SENDER:
            while True:
                event = self.event_queue.get()
                new_event = self._build_event(event)
                self.sender.send(new_event)
        else:
            while True:
                event = self.event_queue.get()
                new_event = self._build_event(event)
                print(new_event)            

    def start(self, file_bag:str=None):
        if ONLINE_RECIEVER:
            self.receiver.start(self.event_queue)

        threading.Thread(
                target=QrHandler,
                args=(self.event_queue, self.node_id, self.shared_qr_state, file_bag),
                daemon=True
            ).start()
    
        threading.Thread(
            target=self._network_loop,
            daemon=True
        ).start()