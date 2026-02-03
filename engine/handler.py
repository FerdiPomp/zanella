import time
import cv2
import numpy as np

from hardware.camera import ObjCamera, QrCamera, SharedQRState, ZEDQrCamera
from engine.event import Event
import config as CONFIG
#from hardware.gpio import Button, Light

import pygame


def ObjectDetectionHandler(stop_event, event_queue, node_id, event_type: str, file_bag : str = None, led_pin : int = None):
    assert event_type in ['ENTER_DETECT', 'EXIT_DETECT']

    is_enter_node = (node_id == 'A')
    camera = ObjCamera(is_enter_node, file_bag)
    if CONFIG.THERE_IS_LED:
        light = Light(led_pin)
    
    state = CONFIG.NO_OBJECT
    pending_since = None

    while not stop_event.is_set():
        object_present, img, obj_mask = camera.find_object()

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
        if CONFIG.IS_DEMO:
            vis = cv2.applyColorMap(cv2.convertScaleAbs(img, alpha=0.03), cv2.COLORMAP_JET)
            x0, y0, x1, y1 = camera.ROI
            cv2.rectangle(vis, (x0,y0), (x1,y1), (0,255,0), 2)
            h, w = img.shape
            vis_height = np.zeros((h, w, 3), dtype=np.uint8)
            vis_height[obj_mask] = (0, 0, 255)
            if state == CONFIG.OBJECT_PRESENT:
                vis = cv2.addWeighted(vis, 1.0, vis_height, 1.0, 0)
                led_color = (0,255,0)
            else:
                led_color = (0,0,255)

            cv2.rectangle(vis,(0,0),(70,70),led_color,-1)
            cv2.imshow("Depth", vis)
            if cv2.waitKey(1) == 27:
                break

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
                    event_queue.put(Event(source=node_id, type='BUTTON_PRESSED', timestamp=now))
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

def QrHandler(stop_event, event_queue, node_id, shared_qr_state:SharedQRState, file_bag : str = None):
    if CONFIG.IS_ZED:
        camera = ZEDQrCamera(shared_qr_state, file_bag)
    else:
        camera = QrCamera(shared_qr_state, file_bag)

    state = CONFIG.NO_QR
    pending_since = None
    pending_change = None
    last_qr = None

    while not stop_event.is_set():
        qr, occlusion, vis = camera.read_qr()
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
        
        if CONFIG.IS_DEMO and vis is not None:
            cv2.imshow("QR Wall", vis)
            if cv2.waitKey(1) == 27:
                break

def SimButtonHandler(stop_event, event_queue, node_id):
    pygame.init()
    screen = pygame.display.set_mode((300, 200))
    
    pygame.display.set_caption("Button test")
    pygame.event.set_grab(False) 
    pygame.mouse.set_visible(True)


    class SimButton:
        def __init__(self, key_code):
            self.key_code = key_code

        def pressed(self):
            keys = pygame.key.get_pressed()
            return keys[self.key_code]

    button = SimButton(pygame.K_f)

    state = CONFIG.NO_PRESS
    pending_since = None

    led_on = False
    while not stop_event.is_set():
        for event in pygame.event.get():
            continue
            #print(event)
        press = button.pressed()
        now = time.time()
        
        # ---- Stato: NO_PRESS ----
        if state == CONFIG.NO_PRESS:
            if press:
                if pending_since is None:
                    pending_since = now
                    
                elif now - pending_since >= CONFIG.BUT_MIN_ON_TIME:
                    state = CONFIG.PRESSED
                    pending_since = None
                    led_on = True
                    event_queue.put(Event(source=node_id, type='BUTTON_PRESSED', timestamp=now))
            else:
                pending_since = None

        # ---- Stato: PRESSED ----
        elif state == CONFIG.PRESSED:
            if not press:
                #led_on = False
                if pending_since is None:
                    pending_since = now
                elif now - pending_since >= CONFIG.BUT_MIN_OFF_TIME:
                    state = CONFIG.NO_PRESS
                    pending_since = None
                    led_on = False
            else:
                pending_since = None

        
        color = (0,255,0) if led_on else (250,0,0)
        pygame.draw.circle(screen, color, (150,100), 50)
        pygame.display.flip()

        time.sleep(0.05)

    pygame.quit()

                

            