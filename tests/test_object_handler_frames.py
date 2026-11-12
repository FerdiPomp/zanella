import queue
import threading
import unittest
import weakref
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np

import configuration as CONFIG

if not CONFIG._loaded:
    CONFIG.load(Path(__file__).resolve().parent.parent / "config.yaml")

from engine.handler import ObjectDetectionHandler


class FakeObjectCamera:
    def __init__(self, stop_event):
        self.stop_event = stop_event
        self.calls = 0
        self.closed = False
        self.depth_refs = []
        self.max_prior_depth_views = 0

    def find_object(self):
        self.max_prior_depth_views = max(
            self.max_prior_depth_views,
            sum(ref() is not None for ref in self.depth_refs),
        )
        self.calls += 1
        depth = np.array([[self.calls]], dtype=np.uint16).view()
        self.depth_refs.append(weakref.ref(depth))
        if self.calls == 44:
            self.stop_event.set()
        object_present = self.calls <= 30 or self.calls >= 43
        return object_present, depth, np.array([[object_present]])

    def close(self):
        self.closed = True


class ObjectHandlerFrameLifetimeTests(unittest.TestCase):
    def test_enter_and_exit_do_not_accumulate_depth_frames(self):
        for node_id, event_type in (("A", "ENTER_DETECT"), ("B", "EXIT_DETECT")):
            with self.subTest(node_id=node_id):
                stop_event = threading.Event()
                event_queue = queue.Queue()
                led_queue = queue.Queue()
                work_state = Mock()
                work_state.get.return_value = True
                camera = FakeObjectCamera(stop_event)

                with patch.dict(CONFIG.__dict__, {"DEBUGGING": True, "OBJ_MIN_OFF_TIME": 10}), \
                     patch("engine.handler.NightRecovery") as recovery_type, \
                     patch("engine.handler._build_obj_camera", return_value=camera) as build_camera, \
                     patch("engine.handler.save_numpy_artifact") as save_artifact, \
                     patch("engine.handler.time.time", side_effect=lambda: float(camera.calls)):
                    recovery_type.return_value.should_run.return_value = False
                    recovery_type.return_value.load_plane.return_value = None
                    ObjectDetectionHandler(
                        stop_event, event_queue, node_id, event_type, work_state, led_queue
                    )

                self.assertTrue(camera.closed)
                self.assertEqual(camera.calls, 44)
                self.assertLessEqual(camera.max_prior_depth_views, 1)
                self.assertEqual([event_queue.get_nowait().type for _ in range(event_queue.qsize())],
                                 [event_type, event_type])
                self.assertEqual([led_queue.get_nowait() for _ in range(led_queue.qsize())],
                                 [("object", "on"), ("object", "off"),
                                  ("object", "on"), ("object", "off")])
                build_camera.assert_called_once_with(node_id == "A", None, None)
                save_artifact.assert_not_called()


if __name__ == "__main__":
    unittest.main()
