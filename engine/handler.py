import time
from hardware.camera import ObjCamera, QrCamera, SharedQRState
from engine.event import Event

NO_OBJECT = 0
OBJECT_PRESENT = 1

NO_PRESS = 0
PRESSED = 1

NO_QR = 0
QR = 1

THERE_IS_LED = False
JETSON = False

if JETSON:
    from hardware.gpio import Light, Button

def ObjectDetectionHandler(stop_event, event_queue, node_id, event_type: str, file_bag : str = None, led_pin : int = None):
    assert event_type in ['ENTER_DETECT', 'EXIT_DETECT']
    
    camera = ObjCamera(file_bag)
    if THERE_IS_LED:
        light = Light(led_pin)
    
    state = NO_OBJECT
    min_off_time = 0
    pending_since = None

    while not stop_event.is_set():
        object_present = camera.find_object()

        if object_present is not None:
            now = time.time()
            # Stato: NO_OBJECT
            if state == NO_OBJECT:
                if object_present:
                    state = OBJECT_PRESENT
                    if THERE_IS_LED:
                        light.on()
                    event_queue.put(Event(source=node_id, type=event_type, timestamp=now))
                    #return "ENTER"

            # Stato: OBJECT_PRESENT
            elif state == OBJECT_PRESENT:
                if not object_present:
                    if pending_since is None:
                        pending_since = now
                    elif now - pending_since >= min_off_time:
                        state = NO_OBJECT
                        if THERE_IS_LED:
                            light.off()
                        pending_since = None
                        #return "EXIT"
                else:
                    pending_since = None
        #time.sleep(0.1)

def ButtonPressHandler(stop_event, event_queue, node_id, button_pin:int = None):
    button = Button(button_pin)

    state = NO_PRESS
    min_on_time = 0.5
    min_off_time = 2
    pending_since = None

    while not stop_event.is_set():
        press = button.pressed()
        now = time.time()

        # Stato: NO_PRESS
        if state == NO_PRESS:
            if press:
                if pending_since is None:
                    pending_since = now
                elif now - pending_since >= min_on_time:
                    state = PRESSED
                    pending_since = None
                    event_queue.put(Event(source=node_id, type='PRESSED', timestamp=now))
                    #return "ENTER"
            else:
                pending_since = None

        # Stato: PRESSED
        elif state == PRESSED:
            if not press:
                if pending_since is None:
                    pending_since = now
                elif now - pending_since >= min_off_time:
                    state = NO_PRESS
                    pending_since = None
                    #return "EXIT"
            else:
                pending_since = None

def QrHandler(stop_event, event_queue, node_id, shared_qr_state:SharedQRState, file_bag : str = None):
    camera = QrCamera(shared_qr_state, file_bag)

    state = NO_QR
    min_on_time = 0.5
    min_off_time = 0.5
    min_change_time = 0.5
    pending_since = None
    pending_change = None
    last_qr = None

    while not stop_event.is_set():
        qr, occlusion = camera.read_qr()
        now = time.time()

        # Stato: NO_QR
        if state == NO_QR:
            if qr is not None:
                if last_qr is not None:
                    if not last_qr == qr:
                        pending_since = None

                if pending_since is None:
                    pending_since = now
                    last_qr = qr
                elif now - pending_since >= min_on_time:
                    state = QR
                    pending_since = None
                    shared_qr_state.update(qr, now)
                    event_queue.put(Event(source=node_id, type='QR_APPEND', timestamp=now, qr=qr))
            else:
                pending_since = None
                last_qr = None

        # Stato: QR
        elif state == QR:
            if (not occlusion):
                if (qr is None ):
                    if pending_since is None:
                        pending_since = now
                    elif now - pending_since >= min_off_time:
                        state = NO_QR
                        pending_since = None
                        shared_qr_state.update(None, now)
                        event_queue.put(Event(source=node_id, type='QR_REMOVED', timestamp=now, qr=last_qr))
                        last_qr = None
                else:
                    pending_since = None
                    if not (qr == last_qr):
                        if pending_change is None:
                            pending_change = now
                        elif now - pending_change >= min_change_time:
                            shared_qr_state.update(None, now)
                            event_queue.put(Event(source=node_id, type='QR_REMOVED', timestamp=now, qr = last_qr))
                            shared_qr_state.update(qr, now)
                            event_queue.put(Event(source=node_id, type='QR_APPEND', timestamp=now, qr = qr))
                            last_qr = qr
                            pending_change = None
                    else:
                        pending_change = None

            
                

            