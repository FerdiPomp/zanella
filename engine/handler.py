import time
import cv2
import numpy as np
import os

from hardware.camera import ObjCamera, QrCamera, SharedQRState, ZEDQrCamera
from engine.event import Event
import config as CONFIG
from hardware.gpio import Button, Light
import gpiod
from gpiod.line import Direction, Value

import pygame


def ObjectDetectionHandler(stop_event, event_queue, node_id, event_type: str, led_queue = None,file_bag : str = None):
    assert event_type in ['ENTER_DETECT', 'EXIT_DETECT']

    is_enter_node = (node_id == 'A')
    camera = ObjCamera(is_enter_node, file_bag)
    
    state = CONFIG.NO_OBJECT
    pending_since = None

    video_seq = []
    record = False

    while not stop_event.is_set():
        object_present, img, obj_mask = camera.find_object()

        if object_present is not None:
            now = time.time()
            # Stato: NO_OBJECT
            if state == CONFIG.NO_OBJECT:
                if object_present:
                    record = True
                    state = CONFIG.OBJECT_PRESENT
                    if led_queue is not None:
                        led_queue.put("on")
                    event_queue.put(Event(source=node_id, type=event_type, timestamp=now))
                    #return "ENTER"

            # Stato: OBJECT_PRESENT
            elif state == CONFIG.OBJECT_PRESENT:
                if not object_present:
                    if pending_since is None:
                        pending_since = now
                    elif now - pending_since >= CONFIG.OBJ_MIN_OFF_TIME:
                        state = CONFIG.NO_OBJECT
                        record = False
                        if led_queue is not None:
                            led_queue.put("off")
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

        if CONFIG.DEBUGGING:
            if record and img is not None:
                video_seq.append(img)
            if not record and len(video_seq)>0:
                save_img(video_seq, '_obj_camera', 'recorded')
                video_seq = []

def ButtonPressHandler(stop_event, event_queue, node_id, led_queue = None):
    button = Button(CONFIG.BUTTON_PIN)

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
                    if led_queue is not None:
                        led_queue.put("on")
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
                    if led_queue is not None:
                        led_queue.put("off")
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

    video_seq = []
    record = False
    while not stop_event.is_set():
        qr, occlusion, vis = camera.read_qr()
        now = time.time()

        # Stato: NO_QR
        if state == CONFIG.NO_QR:
            if qr is not None:
                if last_qr is not None:
                    if not last_qr == qr:
                        pending_since = None
                        record = False

                if pending_since is None:
                    pending_since = now
                    last_qr = qr
                    record = True
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
                        record = False
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
                            if len(video_seq)>0:
                                save_img(video_seq, '_qr_camera', 'recorded')
                                video_seq = []
                    else:
                        pending_change = None
        
        if CONFIG.IS_DEMO and vis is not None:
            cv2.imshow("QR Wall", vis)
            if cv2.waitKey(1) == 27:
                break

        if CONFIG.DEBUGGING:
            if record and vis is not None:
                video_seq.append(vis.copy())
            if not record and len(video_seq)>0:
                save_img(video_seq, '_qr_camera', 'recorded')
                video_seq = []
            
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

def LightHandler(stop_event, led_queue):
    with gpiod.request_lines(
        "/dev/gpiochip0",
        consumer="Led_line",
        config={CONFIG.LED_PIN: gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.INACTIVE)}
    ) as request:
        while not stop_event.is_set():
            cmd = queue.get()
            if cmd == "on":
                request.set_value(CONFIG.LED_PIN, Value.ACTIVE)
            elif cmd == "off":
                request.set_value(CONFIG.LED_PIN, Value.INACTIVE)
        request.set_value(CONFIG.LED_PIN, Value.INACTIVE)

def save_img(img_seq:list, file_name:str, dir_name:str):
    if not os.path.exists(dir_name):
        os.mkdir(dir_name)
    
    full_name = str(time.time_ns()) + file_name
    video = np.array(img_seq)
    np.save(os.path.join(dir_name,full_name), video)


            