import queue
import numpy as np
import time

import config as CONFIG
from engine.event import Event
from engine.runtime_utils import print_log
from engine.runtime_utils import save_numpy_artifact
from hardware.camera import ObjCamera, QrCamera, SharedQRState, ZEDQrCamera, SharedState
from hardware.gpio import Button, Light


OBJECT_EVENTS = ["ENTER_DETECT", "EXIT_DETECT"]


def _send_led_command(led_queue, command: str) -> None:
    if led_queue is not None:
        led_queue.put(command)


def _queue_event(event_queue, node_id, event_type: str, timestamp: float, qr=None) -> None:
    event_queue.put(Event(source=node_id, type=event_type, timestamp=timestamp, qr=qr))


def _save_sequence_if_ready(record: bool, sequence: list, file_name: str, dir_name: str) -> list:
    if record or len(sequence) == 0:
        return sequence
    save_numpy_artifact(np.array(sequence), dir_name, file_name)
    return []


def ObjectDetectionHandler(stop_event, event_queue, node_id, event_type: str, work_state: SharedState, led_queue=None, file_bag: str = None):
    assert event_type in OBJECT_EVENTS

    is_enter_node = node_id == "A"
    camera = ObjCamera(is_enter_node, file_bag)

    state = CONFIG.NO_OBJECT
    pending_since = None

    video_seq = []
    record = False

    while not stop_event.is_set():
        object_present, img, obj_mask = camera.find_object()

        if object_present is not None:
            now = time.time()
            if state == CONFIG.NO_OBJECT:
                if object_present:
                    record = True
                    state = CONFIG.OBJECT_PRESENT
                    if work_state.get():
                        _send_led_command(led_queue, "on")
                        _queue_event(event_queue, node_id, event_type, now)
                    else:
                        _send_led_command(led_queue, "blink")
            elif state == CONFIG.OBJECT_PRESENT:
                if not object_present:
                    if pending_since is None:
                        pending_since = now
                    elif now - pending_since >= CONFIG.OBJ_MIN_OFF_TIME:
                        state = CONFIG.NO_OBJECT
                        record = False
                        _send_led_command(led_queue, "off")
                        pending_since = None
                else:
                    pending_since = None

        if CONFIG.DEBUGGING:
            if record and img is not None:
                video_seq.append(img)
            video_seq = _save_sequence_if_ready(record, video_seq, "_obj_camera", "recorded")


def ButtonPressHandler(stop_event, event_queue, node_id, work_state: SharedState, led_queue=None):
    button = Button()

    state = CONFIG.NO_PRESS
    pending_since = None

    while not stop_event.is_set():
        press = button.pressed()
        now = time.time()

        if state == CONFIG.NO_PRESS:
            if press:
                if pending_since is None:
                    pending_since = now
                elif now - pending_since >= CONFIG.BUT_MIN_ON_TIME:
                    state = CONFIG.PRESSED
                    pending_since = None
                    if work_state.get():
                        _queue_event(event_queue, node_id, "BUTTON_PRESSED", now)
                        _send_led_command(led_queue, "on")
                    else:
                        _send_led_command(led_queue, "blink")
            else:
                pending_since = None
        elif state == CONFIG.PRESSED:
            if not press:
                if pending_since is None:
                    pending_since = now
                elif now - pending_since >= CONFIG.BUT_MIN_OFF_TIME:
                    state = CONFIG.NO_PRESS
                    pending_since = None
                    _send_led_command(led_queue, "off")
            else:
                pending_since = None


def _build_qr_camera(shared_qr_state: SharedQRState, file_bag: str = None):
    if CONFIG.IS_ZED:
        return ZEDQrCamera(shared_qr_state, file_bag)
    return QrCamera(shared_qr_state, file_bag)


def _emit_qr_append(event_queue, shared_qr_state, node_id, qr, timestamp, set_work_state=None):
    shared_qr_state.update(qr, timestamp)
    if set_work_state is not None:
        set_work_state(True)
    _queue_event(event_queue, node_id, "QR_APPEND", timestamp, qr=qr)


def _emit_qr_removed(event_queue, shared_qr_state, node_id, qr, timestamp, set_work_state=None):
    shared_qr_state.update(None, timestamp)
    if set_work_state is not None:
        set_work_state(False)
    _queue_event(event_queue, node_id, "QR_REMOVED", timestamp, qr=qr)


def QrHandler(stop_event, event_queue, node_id, shared_qr_state: SharedQRState, set_work_state=None, file_bag: str = None):
    camera = _build_qr_camera(shared_qr_state, file_bag)

    state = CONFIG.NO_QR
    pending_since = None
    pending_change = None
    last_qr = None

    video_seq = []
    record = False
    while not stop_event.is_set():
        try:
            qr, occlusion, vis = camera.read_qr()
        except Exception as error:
            print_log(f"QR camera failure: {error}")
            time.sleep(CONFIG.ZED_GRAB_RETRY_DELAY)
            camera = _build_qr_camera(shared_qr_state, file_bag)
            continue
        now = time.time()

        if state == CONFIG.NO_QR:
            if qr is not None:
                if last_qr is not None and not last_qr == qr:
                    pending_since = None
                    record = False

                if pending_since is None:
                    pending_since = now
                    last_qr = qr
                    record = True
                elif now - pending_since >= CONFIG.QR_MIN_ON_TIME:
                    state = CONFIG.QR
                    pending_since = None
                    _emit_qr_append(event_queue, shared_qr_state, node_id, qr, now, set_work_state)
            else:
                pending_since = None
                last_qr = None
        elif state == CONFIG.QR:
            if not occlusion:
                if qr is None:
                    if pending_since is None:
                        pending_since = now
                    elif now - pending_since >= CONFIG.QR_MIN_OFF_TIME:
                        state = CONFIG.NO_QR
                        pending_since = None
                        record = False
                        _emit_qr_removed(event_queue, shared_qr_state, node_id, last_qr, now, set_work_state)
                        last_qr = None
                else:
                    pending_since = None
                    if not (qr == last_qr):
                        if pending_change is None:
                            pending_change = now
                        elif now - pending_change >= CONFIG.QR_MIN_CHANGE_TIME:
                            _emit_qr_removed(event_queue, shared_qr_state, node_id, last_qr, now)
                            _emit_qr_append(event_queue, shared_qr_state, node_id, qr, now, set_work_state)
                            last_qr = qr
                            pending_change = None
                            if len(video_seq) > 0:
                                save_numpy_artifact(np.array(video_seq), "recorded", "_qr_camera")
                                video_seq = []
                    else:
                        pending_change = None

        if CONFIG.DEBUGGING:
            if record and vis is not None:
                video_seq.append(vis.copy())
            video_seq = _save_sequence_if_ready(record, video_seq, "_qr_camera", "recorded")


def LightHandler(stop_event, led_queue):
    light = Light()
    try:
        while not stop_event.is_set():
            try:
                command = led_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            if command == "on":
                light.on()
            elif command == "off":
                light.off()

            if command == "blink":
                for i in range(15):
                    light.on()
                    time.sleep(0.3)
                    light.off()
                    time.sleep(0.3)
        light.off()
    finally:
        del light
