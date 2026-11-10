import contextlib
import importlib.util
import io
import os
import select
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import yaml

import configuration as CONFIG

if not CONFIG._loaded:
    CONFIG.load(Path(__file__).resolve().parent.parent / "config.yaml")

from engine.network import HTTPReceiver, HTTPSender
from engine.node import EnvNode
from engine.persistence import PersistentEventQueue
from engine.recovery import PlaneStore
from engine.runtime_utils import RuntimeLease, build_dated_path, runtime_path
from hardware.camera import ObjCamera, RealSenseCamera, ZEDObjCamera, ZEDQrCamera


class WorkState:
    def __init__(self):
        self.value = False

    def changeState(self, value):
        self.value = value


class SharedBoxConfigurationTests(unittest.TestCase):
    def setUp(self):
        self.path = Path(__file__).resolve().parent.parent / "config.yaml"
        self.values = yaml.safe_load(self.path.read_text(encoding="utf-8"))

    def test_old_configuration_keeps_original_ports_and_paths(self):
        for key in ("WORK_STATE_PORT", "TABLE_NODE_ENDPOINTS", "RUNTIME_ROOT", "CAMERA_SERIAL_NUMBER"):
            self.values.pop(key)
        CONFIG._validate_configuration(self.values, self.path)
        with patch.dict(CONFIG.__dict__, {"RUNTIME_ROOT": "."}):
            self.assertEqual(runtime_path(".runtime", "http_out_A"), Path(".runtime/http_out_A"))
            self.assertEqual(runtime_path("recorded"), Path("recorded"))

        with tempfile.TemporaryDirectory() as temporary:
            old_config = Path(temporary) / "old.yaml"
            old_config.write_text(yaml.safe_dump(self.values), encoding="utf-8")
            spec = importlib.util.spec_from_file_location("old_configuration", self.path.with_name("configuration.py"))
            old_module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(old_module)
            old_module.load(old_config)
            self.assertEqual(old_module.WORK_STATE_PORT, self.values["SERVER_PORT"])
            self.assertEqual(old_module.TABLE_NODE_ENDPOINTS, {})
            self.assertEqual(old_module.RUNTIME_ROOT, ".")
            self.assertEqual(old_module.CAMERA_SERIAL_NUMBER, "")

    def test_new_endpoints_and_camera_serial_are_validated(self):
        self.values["TABLE_NODE_IPS"] = []
        self.values["TABLE_NODE_ENDPOINTS"] = {
            "A": {"host": "192.168.1.20", "port": 9101},
            "B": {"host": "192.168.1.20", "port": 9102},
        }
        self.values["IS_ZED"] = True
        self.values["CAMERA_SERIAL_NUMBER"] = "12345678"
        CONFIG._validate_configuration(self.values, self.path)

        self.values["TABLE_NODE_ENDPOINTS"]["B"]["port"] = 9101
        with self.assertRaisesRegex(ValueError, "duplicate destinations"):
            CONFIG._validate_configuration(self.values, self.path)
        self.values["TABLE_NODE_ENDPOINTS"]["B"]["port"] = 9102

        self.values["CAMERA_SERIAL_NUMBER"] = "not-a-zed-serial"
        with self.assertRaisesRegex(ValueError, "ZED CAMERA_SERIAL_NUMBER"):
            CONFIG._validate_configuration(self.values, self.path)
        self.values["CAMERA_SERIAL_NUMBER"] = 12345678
        with self.assertRaisesRegex(ValueError, "quoted string"):
            CONFIG._validate_configuration(self.values, self.path)
        self.values["CAMERA_SERIAL_NUMBER"] = "12345678"

        self.values["WORK_STATE_PORT"] = 65536
        with self.assertRaisesRegex(ValueError, "WORK_STATE_PORT"):
            CONFIG._validate_configuration(self.values, self.path)

    def test_shared_box_requires_runtime_root_and_live_serial(self):
        import main

        settings = {
            "IS_ZED": True,
            "ZED_ENV_HAS_DEPTH": True,
            "LED_SOCKET_PATH": "/tmp/tracevision-led.sock",
            "RUNTIME_ROOT": ".",
            "CAMERA_SERIAL_NUMBER": "12345678",
        }
        with patch.dict(CONFIG.__dict__, settings), patch.object(main, "_require_module"):
            with self.assertRaisesRegex(RuntimeError, "separate RUNTIME_ROOT"):
                main.validate_runtime_dependencies("A")
            CONFIG.RUNTIME_ROOT = "/tmp/tracevision-a"
            CONFIG.CAMERA_SERIAL_NUMBER = ""
            with self.assertRaisesRegex(RuntimeError, "CAMERA_SERIAL_NUMBER"):
                main.validate_runtime_dependencies("A")
            main.validate_runtime_dependencies("A", file_bag="recording.svo2")


class RuntimeIsolationTests(unittest.TestCase):
    def test_same_root_is_rejected_until_first_process_releases_it(self):
        with tempfile.TemporaryDirectory() as temporary:
            root_a = str(Path(temporary) / "A")
            root_b = str(Path(temporary) / "B")
            with patch.dict(CONFIG.__dict__, {"RUNTIME_ROOT": root_a}):
                first = RuntimeLease()
                try:
                    with self.assertRaisesRegex(RuntimeError, "already in use"):
                        RuntimeLease()
                    with patch.dict(CONFIG.__dict__, {"RUNTIME_ROOT": root_b}):
                        second = RuntimeLease()
                        second.close()
                finally:
                    first.close()
                again = RuntimeLease()
                again.close()

    def test_spool_plane_dedup_and_debug_have_separate_roots(self):
        with tempfile.TemporaryDirectory() as temporary:
            roots = [Path(temporary) / "A", Path(temporary) / "B"]
            paths = []
            for node_id, root in zip(("A", "B"), roots):
                with patch.dict(CONFIG.__dict__, {"RUNTIME_ROOT": str(root)}):
                    spool = PersistentEventQueue(runtime_path(".runtime", f"http_out_{node_id}"), lambda x: x, lambda x: x)
                    spool.put({"node": node_id})
                    plane = PlaneStore(node_id)
                    self.assertTrue(plane.save([0, 0, -1, 1]))
                    receiver = HTTPReceiver(host="127.0.0.1", port=0)
                    debug = build_dated_path("recorded", "_test", ".npy").parent
                    paths.append((spool.spool_dir, plane.path, receiver.seen_dir, debug))
                    self.assertTrue(spool.has_pending())
                    self.assertEqual(plane.load(), [0.0, 0.0, -1.0, 1.0])
            for a, b in zip(*paths):
                self.assertNotEqual(a, b)
                self.assertEqual(a.parts[: len(roots[0].parts)], roots[0].parts)
                self.assertEqual(b.parts[: len(roots[1].parts)], roots[1].parts)


class WorkStateHttpTests(unittest.TestCase):
    def test_two_separate_configured_processes_start_together(self):
        root = Path(__file__).resolve().parent.parent
        defaults = yaml.safe_load((root / "config.yaml").read_text(encoding="utf-8"))
        child_code = """
import configuration as CONFIG
import sys
CONFIG.load(sys.argv[1])
from engine.node import EnterNode, ExitNode
node = (EnterNode if sys.argv[2] == 'A' else ExitNode)(sys.argv[2])
node._append_object_thread = lambda *args: None
node._append_led_thread_if_enabled = lambda: None
node._start_network_thread = lambda target: None
node.start()
print('READY', flush=True)
sys.stdin.readline()
node.stop()
"""
        with tempfile.TemporaryDirectory() as temporary:
            reservations = [socket.socket(), socket.socket()]
            for reservation in reservations:
                reservation.bind(("127.0.0.1", 0))
            ports = [reservation.getsockname()[1] for reservation in reservations]
            for reservation in reservations:
                reservation.close()

            children = []
            try:
                for node_id, port in zip(("A", "B"), ports):
                    values = dict(defaults)
                    values.update({
                        "WORK_STATE_PORT": port,
                        "RUNTIME_ROOT": str(Path(temporary) / node_id),
                        "ONLINE_SENDER": False,
                        "THERE_IS_LED": False,
                        "THERE_IS_BUTTON": False,
                    })
                    config_path = Path(temporary) / f"{node_id}.yaml"
                    config_path.write_text(yaml.safe_dump(values), encoding="utf-8")
                    child = subprocess.Popen(
                        [sys.executable, "-u", "-c", child_code, str(config_path), node_id],
                        cwd=root,
                        stdin=subprocess.PIPE,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        text=True,
                        env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"},
                    )
                    children.append(child)
                    ready, _, _ = select.select([child.stdout], [], [], 10)
                    self.assertTrue(ready, "Table process did not start within 10 seconds")
                    self.assertEqual(child.stdout.readline().strip(), "READY")

                for node_id, port in zip(("A", "B"), ports):
                    self.assertTrue(HTTPSender("127.0.0.1", port, "/work_state").send_work_state(True))
                    self.assertTrue((Path(temporary) / node_id / ".runtime" / f"http_out_{node_id}").exists())
            finally:
                for child in children:
                    if child.poll() is None:
                        child.stdin.write("\n")
                        child.stdin.flush()
                        try:
                            child.wait(timeout=3)
                        except subprocess.TimeoutExpired:
                            child.terminate()
                            child.wait(timeout=3)
                    child.stdin.close()
                    child.stdout.close()
                    child.stderr.close()
                for child in children:
                    self.assertEqual(child.returncode, 0)

    def test_two_ports_receive_independent_state_and_conflict_fails_fast(self):
        with tempfile.TemporaryDirectory() as temporary, patch.dict(CONFIG.__dict__, {"RUNTIME_ROOT": temporary}):
            a, b = WorkState(), WorkState()
            receiver_a = HTTPReceiver(host="127.0.0.1", port=0)
            receiver_b = HTTPReceiver(host="127.0.0.1", port=0)
            receiver_a.start(work_state=a)
            receiver_b.start(work_state=b)
            port_a = receiver_a.server.server_address[1]
            port_b = receiver_b.server.server_address[1]
            self.assertNotEqual(port_a, port_b)
            try:
                self.assertTrue(HTTPSender("127.0.0.1", port_a, "/work_state").send_work_state(True))
                self.assertTrue(a.value)
                self.assertFalse(b.value)
                self.assertTrue(HTTPSender("127.0.0.1", port_b, "/work_state").send_work_state(True))
                self.assertTrue(b.value)
                with contextlib.redirect_stderr(io.StringIO()):
                    with self.assertRaisesRegex(RuntimeError, "Unable to bind HTTP receiver"):
                        HTTPReceiver(host="127.0.0.1", port=port_a).start(work_state=WorkState())
            finally:
                receiver_a.stop()
                receiver_b.stop()

            restarted = HTTPReceiver(host="127.0.0.1", port=port_a)
            restarted.start(work_state=WorkState())
            restarted.stop()

    def test_env_node_sends_to_both_ports_on_same_host(self):
        with tempfile.TemporaryDirectory() as temporary:
            a, b = WorkState(), WorkState()
            receiver_a = HTTPReceiver(host="127.0.0.1", port=0)
            receiver_b = HTTPReceiver(host="127.0.0.1", port=0)
            receiver_a.start(work_state=a)
            receiver_b.start(work_state=b)
            settings = {
                "RUNTIME_ROOT": str(Path(temporary) / "C"),
                "ONLINE_SENDER_ENV": False,
                "ONLINE_RECIEVER": False,
                "TABLE_NODE_IPS": (),
                "TABLE_NODE_ENDPOINTS": {
                    "A": {"host": "127.0.0.1", "port": receiver_a.server.server_address[1]},
                    "B": {"host": "127.0.0.1", "port": receiver_b.server.server_address[1]},
                },
                "WORK_STATE_SYNC_INTERVAL": 0.05,
            }
            try:
                with patch.dict(CONFIG.__dict__, settings):
                    env = EnvNode("C", workspace=None, broker_psw=None, topic=None)
                    self.assertNotEqual(env.work_state_senders[0].url, env.work_state_senders[1].url)
                    env._set_work_state(True)
                    worker = threading.Thread(target=env._work_state_loop)
                    worker.start()
                    try:
                        for _ in range(100):
                            if a.value and b.value:
                                break
                            time.sleep(0.01)
                        self.assertTrue(a.value and b.value)
                    finally:
                        env.stop_event.set()
                        env.work_state_changed.set()
                        worker.join(timeout=2)
                        self.assertFalse(worker.is_alive())
            finally:
                receiver_a.stop()
                receiver_b.stop()


class CameraSerialTests(unittest.TestCase):
    def test_realsense_selects_serial_for_live_and_not_for_playback(self):
        class FakeConfig:
            def __init__(self):
                self.serial = None
                self.recording = None

            def enable_device(self, serial):
                self.serial = serial

            def enable_device_from_file(self, path, repeat_playback):
                self.recording = (path, repeat_playback)

            def enable_stream(self, *args):
                pass

        class FakePipeline:
            def __init__(self):
                self.started = []

            def start(self, config):
                self.started.append(config)
                sensor = SimpleNamespace(get_depth_scale=lambda: 0.001)
                device = SimpleNamespace(first_depth_sensor=lambda: sensor)
                return SimpleNamespace(get_device=lambda: device)

            def stop(self):
                pass

        fake_rs = SimpleNamespace(
            config=FakeConfig,
            pipeline=FakePipeline,
            pointcloud=lambda: object(),
            stream=SimpleNamespace(depth=1, color=2),
            format=SimpleNamespace(z16=1, bgr8=2),
        )
        with patch.dict("sys.modules", {"pyrealsense2": fake_rs}), patch.dict(CONFIG.__dict__, {"CAMERA_SERIAL_NUMBER": "00123456"}):
            live = RealSenseCamera(rgb=False)
            self.assertEqual(live.config.serial, "00123456")
            self.assertIs(live.pipeline.started[0], live.config)
            live.close()
            playback = RealSenseCamera(rgb=False, file_bag="sample.bag")
            self.assertIsNone(playback.config.serial)
            self.assertEqual(playback.config.recording, ("sample.bag", False))
            playback.close()

    def test_realsense_timeout_restarts_with_same_selected_config(self):
        class FakePipeline:
            def __init__(self):
                self.started = []

            def start(self, config):
                self.started.append(config)

            def stop(self):
                pass

        depth_frame = SimpleNamespace(get_data=lambda: np.zeros((1, 1), dtype=np.uint16))
        frames = SimpleNamespace(get_depth_frame=lambda: depth_frame)
        camera = ObjCamera.__new__(ObjCamera)
        camera.pipeline = FakePipeline()
        camera.config = SimpleNamespace(serial="00123456")
        camera._wait_for_frames = Mock(side_effect=[RuntimeError("timeout"), frames])
        camera._build_height_map_from_depth_frame = lambda frame: np.zeros((1, 1))
        camera._update_object_state = lambda mask: (False, False)
        with patch("hardware.camera.time.sleep"), patch.dict(CONFIG.__dict__, {"DEBUGGING": False}):
            camera.find_object()
        self.assertEqual(camera.pipeline.started, [camera.config])
        self.assertEqual(camera.pipeline.started[0].serial, "00123456")
        camera.close()

    def test_zed_reopen_keeps_serial_and_playback_overrides_it(self):
        class FakeInit:
            def __init__(self):
                self.serial = None
                self.recording = None

            def set_from_serial_number(self, serial):
                self.serial = serial

            def set_from_svo_file(self, path):
                self.recording = path

        class FakeCamera:
            def __init__(self):
                self.params = None

            def open(self, params):
                self.params = params
                return 0

            def close(self):
                pass

        fake_sl = SimpleNamespace(
            InitParameters=FakeInit,
            Camera=FakeCamera,
            DEPTH_MODE=SimpleNamespace(NEURAL=1),
            UNIT=SimpleNamespace(METER=1),
            ERROR_CODE=SimpleNamespace(SUCCESS=0),
        )
        with patch.dict(CONFIG.__dict__, {"CAMERA_SERIAL_NUMBER": "12345678"}):
            camera = ZEDObjCamera.__new__(ZEDObjCamera)
            camera.sl = fake_sl
            camera.file_bag = None
            camera.zed = None
            camera._open_camera()
            first = camera.zed
            self.assertEqual(first.params.serial, 12345678)
            camera._open_camera()
            self.assertIsNot(camera.zed, first)
            self.assertEqual(camera.zed.params.serial, 12345678)
            camera.file_bag = "sample.svo2"
            self.assertIsNone(camera._build_init_params().serial)
            self.assertEqual(camera._build_init_params().recording, "sample.svo2")
            camera.close()

            env_camera = ZEDQrCamera.__new__(ZEDQrCamera)
            env_camera.sl = SimpleNamespace(**fake_sl.__dict__, RESOLUTION=SimpleNamespace(HD2K=1))
            env_camera.is_zed_one = False
            env_camera.file_bag = None
            self.assertEqual(env_camera._build_init_params().serial, 12345678)
            env_camera.zed = None


if __name__ == "__main__":
    unittest.main()
