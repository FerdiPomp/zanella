import json
import socket
import tempfile
import threading
import time
import unittest
from pathlib import Path
from queue import Queue
from unittest.mock import patch

from engine import led


class FakeLight:
    def __init__(self):
        self.is_on = False
        self.history = []

    def on(self):
        self.is_on = True
        self.history.append("on")

    def off(self):
        self.is_on = False
        self.history.append("off")

    def close(self):
        pass


def wait_until(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("Timed out waiting for LED state")


class LedArbiterTests(unittest.TestCase):
    def test_all_table_combinations(self):
        expected = {
            ("off", "off"): "off",
            ("off", "on"): "on",
            ("off", "blink"): "blink",
            ("on", "off"): "on",
            ("on", "on"): "on",
            ("on", "blink"): "blink",
            ("blink", "off"): "blink",
            ("blink", "on"): "blink",
            ("blink", "blink"): "blink",
        }
        for (a, b), result in expected.items():
            with self.subTest(a=a, b=b):
                arbiter = led.LedArbiter()
                for node, mode in (("A", a), ("B", b)):
                    state = {} if mode == "off" else {"object": {"mode": mode}}
                    if mode == "blink":
                        state["object"]["until"] = 109.0
                    arbiter.update({"node": node, "states": state}, now=100.0)
                self.assertEqual(arbiter.mode(now=100.0), result)

    def test_button_off_does_not_clear_object_or_other_table(self):
        arbiter = led.LedArbiter()
        arbiter.update({"node": "A", "states": {"object": {"mode": "on"}}}, now=100.0)
        arbiter.update({"node": "B", "states": {
            "object": {"mode": "on"},
            "button": {"mode": "blink", "until": 109.0},
        }}, now=100.0)
        self.assertEqual(arbiter.mode(now=100.0), "blink")
        self.assertEqual(arbiter.mode(now=109.0), "off")  # Both node heartbeats have expired.
        arbiter.update({"node": "A", "states": {"object": {"mode": "on"}}}, now=108.0)
        arbiter.update({"node": "B", "states": {"object": {"mode": "on"}}}, now=108.0)
        self.assertEqual(arbiter.mode(now=109.0), "on")
        arbiter.update({"node": "B", "states": {}}, now=109.0)
        self.assertEqual(arbiter.mode(now=109.0), "on")

    def test_expired_source_and_invalid_message(self):
        arbiter = led.LedArbiter()
        arbiter.update({"node": "A", "states": {"object": {"mode": "on"}}}, now=100.0)
        self.assertEqual(arbiter.mode(now=103.1), "off")
        with self.assertRaises(ValueError):
            arbiter.update({"node": "A", "states": {"button": {"mode": "on"}}}, now=100.0)
        with self.assertRaises(ValueError):
            arbiter.update({"node": "B", "states": {"object": {"mode": "blink", "until": "later"}}}, now=100.0)


class LedConfigurationTests(unittest.TestCase):
    def test_led_socket_is_optional_but_must_be_absolute(self):
        import yaml
        import configuration

        config_path = Path(__file__).resolve().parent.parent / "config.yaml"
        values = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        values.pop("LED_SOCKET_PATH")
        configuration._validate_configuration(values, config_path)
        values["LED_SOCKET_PATH"] = "relative/led.sock"
        with self.assertRaisesRegex(ValueError, "LED_SOCKET_PATH must be absolute"):
            configuration._validate_configuration(values, config_path)
        values["LED_SOCKET_PATH"] = "/tmp/tracevision-led.sock"
        values["THERE_IS_LED"] = True
        values["THERE_IS_BUTTON"] = True
        values["BUTTON_PIN"] = values["LED_PIN"]
        with self.assertRaisesRegex(ValueError, "LED_PIN and BUTTON_PIN must be different"):
            configuration._validate_configuration(values, config_path)

    def test_shared_nodes_do_not_open_led_gpio(self):
        import configuration
        import main

        values = {
            "IS_ZED": True,
            "ZED_ENV_HAS_DEPTH": True,
            "THERE_IS_LED": True,
            "THERE_IS_BUTTON": False,
            "LED_SOCKET_PATH": "/tmp/tracevision-led.sock",
            "RUNTIME_ROOT": "/tmp/tracevision-a",
            "CAMERA_SERIAL_NUMBER": "123456",
            "ONLINE_SENDER": False,
        }
        with patch.dict(configuration.__dict__, values), patch.object(main, "_require_module") as require:
            main.validate_runtime_dependencies("A")
            self.assertNotIn("gpiod", [call.args[0] for call in require.call_args_list])
            main.validate_runtime_dependencies("LED")
            require.assert_any_call("gpiod")


class LedSocketTests(unittest.TestCase):
    def test_clients_share_one_gpio_and_recover_after_blink(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            socket_path = str(Path(temp_dir) / "led.sock")
            fake_light = FakeLight()
            with patch.object(led, "Light", return_value=fake_light) as light_factory, \
                 patch.object(led, "BLINK_DURATION", 0.8), \
                 patch.object(led, "BLINK_HALF_PERIOD", 0.15), \
                 patch.object(led, "REFRESH_INTERVAL", 0.05), \
                 patch.object(led, "SOURCE_TIMEOUT", 0.3):
                controller = led.LedController(socket_path)
                controller.start()
                stops = [threading.Event(), threading.Event()]
                queues = [Queue(), Queue()]
                clients = [
                    threading.Thread(target=led.SharedLedHandler, args=(stops[index], queues[index], node, socket_path))
                    for index, node in enumerate(("A", "B"))
                ]
                for client in clients:
                    client.start()
                try:
                    queues[0].put(("object", "on"))
                    wait_until(lambda: fake_light.is_on)
                    fake_light.history.clear()
                    queues[1].put(("button", "blink"))
                    wait_until(lambda: "off" in fake_light.history)
                    wait_until(lambda: controller.arbiter.mode() == "on", timeout=2.0)
                    wait_until(lambda: fake_light.is_on)
                    queues[1].put(("button", "off"))
                    queues[0].put(("object", "off"))
                    wait_until(lambda: controller.arbiter.mode() == "off")
                    wait_until(lambda: not fake_light.is_on)
                finally:
                    for stop in stops:
                        stop.set()
                    for client in clients:
                        client.join(timeout=2.0)
                        self.assertFalse(client.is_alive())
                    controller.stop()
                self.assertFalse(Path(socket_path).exists())
                self.assertFalse(fake_light.is_on)
                light_factory.assert_called_once()

    def test_second_owner_is_rejected_and_bad_message_is_ignored(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            socket_path = str(Path(temp_dir) / "led.sock")
            with patch.object(led, "Light", return_value=FakeLight()):
                controller = led.LedController(socket_path)
                controller.start()
                try:
                    with self.assertRaises(RuntimeError):
                        led.LedController(socket_path).start()
                    with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as client:
                        client.sendto(b"not json", socket_path)
                        client.sendto(json.dumps({"node": "A", "states": {"object": {"mode": "on"}}}).encode(), socket_path)
                    wait_until(lambda: controller.arbiter.mode() == "on")
                    self.assertTrue(Path(socket_path).exists())
                finally:
                    controller.stop()


class LocalLedTests(unittest.TestCase):
    def test_button_release_preserves_object_request(self):
        from engine.handler import LightHandler

        fake_light = FakeLight()
        commands = Queue()
        stop = threading.Event()
        with patch("engine.handler.Light", return_value=fake_light), \
             patch.object(led, "BLINK_DURATION", 0.7), \
             patch.object(led, "BLINK_HALF_PERIOD", 0.15):
            worker = threading.Thread(target=LightHandler, args=(stop, commands))
            worker.start()
            try:
                commands.put(("object", "on"))
                wait_until(lambda: fake_light.is_on)
                commands.put(("button", "on"))
                commands.put(("button", "off"))
                time.sleep(0.15)
                self.assertTrue(fake_light.is_on)
                commands.put(("button", "blink"))
                wait_until(lambda: "off" in fake_light.history)
                time.sleep(0.8)
                self.assertTrue(fake_light.is_on)
                commands.put(("object", "off"))
                wait_until(lambda: not fake_light.is_on, timeout=2.0)
            finally:
                stop.set()
                worker.join(timeout=2.0)
                self.assertFalse(worker.is_alive())


if __name__ == "__main__":
    unittest.main()
