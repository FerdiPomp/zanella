import time
from hardware.camera import ObjCamera, QrCamera, SharedQRState, ZEDQrCamera
from engine.event import Event

NO_OBJECT = 0
OBJECT_PRESENT = 1

NO_PRESS = 0
PRESSED = 1

NO_QR = 0
QR = 1


def ObjectDetectionHandler(stop_event, event_queue, node_id, event_type: str, file_bag : str = None, led_pin : int = None):
    assert event_type in ['ENTER_DETECT', 'EXIT_DETECT']

    is_enter_node = (node_id == 'A')
    camera = ObjCamera(is_enter_node, file_bag)
    if CONFIG.THERE_IS_LED:
        light = Light(led_pin)
    
    state = CONFIG.NO_OBJECT
    pending_since = None

    while not stop_event.is_set():
        object_present = camera.find_object()

        if object_present is not None:
            now = time.time()
            # Stato: NO_OBJECT
            if state == CONFIG.NO_OBJECT:
                if object_present:
                    state = CONFIG.OBJECT_PRESENT
                    if CONFIG.THERE_IS_LED:
                        light.on()
                    event_queue.put(Event(source=node_id, type=event_type, timestamp=now))
                    #return "ENTER"

            # Stato: OBJECT_PRESENT
            elif state == CONFIG.OBJECT_PRESENT:
                if not object_present:
                    if pending_since is None:
                        pending_since = now
                    elif now - pending_since >= CONFIG.OBJ_MIN_OFF_TIME:
                        state = CONFIG.NO_OBJECT
                        if CONFIG.THERE_IS_LED:
                            light.off()
                        pending_since = None
                        #return "EXIT"
                else:
                    pending_since = None
        #time.sleep(0.1)

def ButtonPressHandler(stop_event, event_queue, node_id, button_pin:int = None):
    button = Button(button_pin)

    state = CONFIG.NO_PRESS
    pending_since = None

    #TODO: ADD LED UP-DOWN inside this thing
    while not stop_event.is_set():
        press = button.pressed()
        now = time.time()

        # Stato: NO_PRESS
        if state == CONFIG.NO_PRESS:
            if press:
                if pending_since is None:
                    pending_since = now
                elif now - pending_since >= CONFIG.BUT_MIN_ON_TIME:
                    state = CONFIG.PRESSED
                    pending_since = None
                    event_queue.put(Event(source=node_id, type='PRESSED', timestamp=now))
                    #return "ENTER"
            else:
                pending_since = None

        # Stato: PRESSED
        elif state == CONFIG.PRESSED:
            if not press:
                if pending_since is None:
                    pending_since = now
                elif now - pending_since >= CONFIG.BUT_MIN_OFF_TIME:
                    state = NO_PRESS
                    pending_since = None
                    #return "EXIT"
            else:
                pending_since = None

def QrHandler(stop_event, event_queue, node_id, shared_qr_state:SharedQRState, is_zed:bool, file_bag : str = None):
    if is_zed:
        camera = ZEDQrCamera(shared_qr_state, file_bag)
    else:
        camera = QrCamera(shared_qr_state, file_bag)

    state = CONFIG.NO_QR
    pending_since = None
    pending_change = None
    last_qr = None

    while not stop_event.is_set():
        qr, occlusion = camera.read_qr()
        now = time.time()

        # Stato: NO_QR
        if state == CONFIG.NO_QR:
            if qr is not None:
                if last_qr is not None:
                    if not last_qr == qr:
                        pending_since = None

                if pending_since is None:
                    pending_since = now
                    last_qr = qr
                elif now - pending_since >= CONFIG.QR_MIN_ON_TIME:
                    state = CONFIG.QR
                    pending_since = None
                    shared_qr_state.update(qr, now)
                    event_queue.put(Event(source=node_id, type='QR_APPEND', timestamp=now, qr=qr))
            else:
                pending_since = None
                last_qr = None

        # Stato: QR
        elif state == CONFIG.QR:
            #TODO: resetting pending state if an occlusion occurs ?
            if (not occlusion):
                if (qr is None ):
                    if pending_since is None:
                        pending_since = now
                    elif now - pending_since >= CONFIG.QR_MIN_OFF_TIME:
                        state = CONFIG.NO_QR
                        pending_since = None
                        shared_qr_state.update(None, now)
                        event_queue.put(Event(source=node_id, type='QR_REMOVED', timestamp=now, qr=last_qr))
                        last_qr = None
                else:
                    pending_since = None
                    if not (qr == last_qr):
                        if pending_change is None:
                            pending_change = now
                        elif now - pending_change >= CONFIG.QR_MIN_CHANGE_TIME:
                            shared_qr_state.update(None, now)
                            event_queue.put(Event(source=node_id, type='QR_REMOVED', timestamp=now, qr = last_qr))
                            shared_qr_state.update(qr, now)
                            event_queue.put(Event(source=node_id, type='QR_APPEND', timestamp=now, qr = qr))
                            last_qr = qr
                            pending_change = None
                    else:
                        pending_change = None


# def SimButtonHandler(stop_event, event_queue, node_id):
#     button = SimButton()
#     state = CONFIG.NO_PRESS

#     #TODO: ADD LED UP-DOWN inside this thing
#     while not stop_event.is_set():
#         press = False
#         for line in sys.stdin:
#         if line.strip().lower() == 'f':
#             press = True
#             event_queue.put(Event(source=node_id, type='PRESSED', timestamp=now))
#             now = time.time()
#             break
                

            