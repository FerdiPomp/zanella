import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

import configuration as CONFIG

if not CONFIG._loaded:
    CONFIG.load(Path(__file__).resolve().parent.parent / "config.yaml")

from hardware.camera import ObjCamera
from hardware.realsense_diagnostics import RealSenseFrameDiagnostics, device_details


def frame(number, timestamp, data):
    return SimpleNamespace(
        get_frame_number=lambda: number,
        get_timestamp=lambda: timestamp,
        get_frame_timestamp_domain=lambda: "hardware_clock",
        get_data=lambda: data,
    )


def trace_rows(root):
    paths = list((Path(root) / ".runtime").glob("realsense_frames_*.jsonl"))
    if len(paths) != 1:
        raise AssertionError(f"Expected one trace file, got {paths}")
    return [json.loads(line) for line in paths[0].read_text(encoding="utf-8").splitlines()]


class RealSenseFrameDiagnosticsTests(unittest.TestCase):
    def test_repeated_depth_with_advancing_color_is_recorded_per_frame(self):
        with tempfile.TemporaryDirectory() as root, patch.dict(CONFIG.__dict__, {"RUNTIME_ROOT": root}):
            diagnostics = RealSenseFrameDiagnostics("B", {"serial_number": "123", "firmware_version": "1"})
            depth_data = np.array([[1, 2], [3, 4]], dtype=np.uint16)
            color_a = np.zeros((2, 2, 3), dtype=np.uint8)
            color_b = np.ones((2, 2, 3), dtype=np.uint8)
            depth_frame = frame(10, 1000.0, depth_data)
            diagnostics.record_frame(depth_frame, frame(20, 1000.0, color_a), depth_data, color_a, 25.0)
            diagnostics.record_frame(depth_frame, frame(21, 1200.0, color_b), depth_data, color_b, 20.0)
            self.assertEqual(len(trace_rows(root)), 3)  # Records are readable before shutdown.
            diagnostics.close()

            rows = trace_rows(root)
            self.assertEqual([row["event"] for row in rows], ["camera_open", "frame", "frame"])
            self.assertEqual(rows[0]["device"]["serial_number"], "123")
            self.assertEqual(rows[1]["depth"]["number"], 10)
            self.assertEqual(rows[2]["depth"]["timestamp_ms"], 1000.0)
            self.assertEqual(rows[2]["color"]["number"], 21)
            self.assertEqual(rows[2]["depth"]["timestamp_domain"], "hardware_clock")
            self.assertEqual(rows[2]["depth"]["digest_sha256"], rows[1]["depth"]["digest_sha256"])
            self.assertNotEqual(rows[2]["color"]["digest_sha256"], rows[1]["color"]["digest_sha256"])
            self.assertTrue(rows[2]["same_depth_frame"])
            self.assertTrue(rows[2]["same_depth_payload"])
            self.assertTrue(rows[2]["color_advanced"])
            self.assertEqual(rows[2]["depth_repeat_count"], 1)
            self.assertIn("+", rows[2]["host_time"])
            self.assertGreater(rows[2]["host_monotonic_ns"], 0)

    def test_new_depth_number_with_identical_payload_is_distinguished(self):
        with tempfile.TemporaryDirectory() as root, patch.dict(CONFIG.__dict__, {"RUNTIME_ROOT": root}):
            diagnostics = RealSenseFrameDiagnostics("B", {})
            depth_data = np.zeros((2, 2), dtype=np.uint16)
            diagnostics.record_frame(frame(10, 1000.0, depth_data), None, depth_data, None, 1.0)
            diagnostics.record_frame(frame(11, 1200.0, depth_data), None, depth_data, None, 1.0)
            diagnostics.close()
            first, second = [row for row in trace_rows(root) if row["event"] == "frame"]
            self.assertFalse(second["same_depth_frame"])
            self.assertTrue(second["same_depth_payload"])
            self.assertIsNone(second["color"])
            self.assertEqual(second["depth_repeat_count"], 0)
            self.assertEqual(first["depth"]["digest_sha256"], second["depth"]["digest_sha256"])

    def test_pipeline_restart_resets_frame_comparison(self):
        with tempfile.TemporaryDirectory() as root, patch.dict(CONFIG.__dict__, {"RUNTIME_ROOT": root}):
            diagnostics = RealSenseFrameDiagnostics("B", {})
            depth_data = np.zeros((2, 2), dtype=np.uint16)
            depth_frame = frame(10, 1000.0, depth_data)
            diagnostics.record_frame(depth_frame, None, depth_data, None, 1.0)
            diagnostics.record_frame(depth_frame, None, depth_data, None, 1.0)
            diagnostics.record_event("pipeline_restarted")
            diagnostics.reset_frame_history()
            diagnostics.record_frame(depth_frame, None, depth_data, None, 1.0)
            diagnostics.close()
            frames = [row for row in trace_rows(root) if row["event"] == "frame"]
            self.assertEqual([row["depth_repeat_count"] for row in frames], [0, 1, 0])

    def test_stall_warning_is_emitted_once_and_recovery_is_reported(self):
        with tempfile.TemporaryDirectory() as root, patch.dict(CONFIG.__dict__, {"RUNTIME_ROOT": root}):
            depth_data = np.zeros((2, 2), dtype=np.uint16)
            with patch("hardware.realsense_diagnostics.print_log") as log:
                diagnostics = RealSenseFrameDiagnostics("B", {})
                for _ in range(8):
                    diagnostics.record_frame(frame(10, 1000.0, depth_data), None, depth_data, None, 1.0)
                diagnostics.record_frame(frame(11, 1200.0, depth_data), None, depth_data, None, 1.0)
                diagnostics.close()
            self.assertEqual(log.call_count, 2)
            self.assertIn("repeated", log.call_args_list[0].args[0])
            self.assertIn("resumed", log.call_args_list[1].args[0])

    def test_diagnostic_stdout_failure_does_not_interrupt_frames(self):
        with tempfile.TemporaryDirectory() as root, patch.dict(CONFIG.__dict__, {"RUNTIME_ROOT": root}):
            depth_data = np.zeros((2, 2), dtype=np.uint16)
            with patch("hardware.realsense_diagnostics.print_log", side_effect=BrokenPipeError("stdout closed")):
                diagnostics = RealSenseFrameDiagnostics("B", {})
                for _ in range(7):
                    diagnostics.record_frame(frame(10, 1000.0, depth_data), None, depth_data, None, 1.0)
                diagnostics.close()
            self.assertEqual(len([row for row in trace_rows(root) if row["event"] == "frame"]), 7)

    def test_missing_metadata_and_unavailable_trace_do_not_interrupt_detection(self):
        with tempfile.TemporaryDirectory() as root, patch.dict(CONFIG.__dict__, {"RUNTIME_ROOT": root}):
            diagnostics = RealSenseFrameDiagnostics("B", {})
            diagnostics.record_frame(SimpleNamespace(get_data=lambda: np.zeros((1, 1))), None, None, None, 1.0)
            diagnostics.close()
            row = [item for item in trace_rows(root) if item["event"] == "frame"][0]
            self.assertIsNone(row["depth"]["number"])
            self.assertIn("number_error", row["depth"])
            self.assertIsNone(row["color"])

        with tempfile.TemporaryDirectory() as root, patch.dict(CONFIG.__dict__, {"RUNTIME_ROOT": root}):
            (Path(root) / ".runtime").write_text("not a directory", encoding="utf-8")
            with patch("hardware.realsense_diagnostics.print_log") as warning:
                diagnostics = RealSenseFrameDiagnostics("B", {})
                diagnostics.record_frame(None, None, None, None, 1.0)
                diagnostics.close()
            warning.assert_called_once()

    def test_device_details_skips_unsupported_values(self):
        info = SimpleNamespace(serial_number=1, firmware_version=2, usb_type_descriptor=3)
        device = SimpleNamespace(supports=lambda key: key != 3, get_info=lambda key: {1: "123", 2: "5.0"}[key])
        self.assertEqual(device_details(SimpleNamespace(camera_info=info), device), {
            "serial_number": "123", "firmware_version": "5.0"
        })

    def test_object_camera_trace_links_saved_artifacts_without_changing_result(self):
        with tempfile.TemporaryDirectory() as root, patch.dict(CONFIG.__dict__, {"RUNTIME_ROOT": root, "DEBUGGING": True}):
            depth_data = np.array([[1, 2], [3, 4]], dtype=np.uint16)
            color_data = np.zeros((2, 2, 3), dtype=np.uint8)
            depth_frame = frame(10, 1000.0, depth_data)
            color_frame = frame(20, 1000.0, color_data)
            frames = SimpleNamespace(get_depth_frame=lambda: depth_frame, get_color_frame=lambda: color_frame)
            camera = ObjCamera.__new__(ObjCamera)
            camera.pipeline = SimpleNamespace(stop=lambda: None)
            camera._wait_for_frames = Mock(return_value=frames)
            camera._build_height_map_from_depth_frame = lambda _: np.zeros((2, 2))
            camera._update_object_state = lambda _: (True, True)
            camera.frame_diagnostics = RealSenseFrameDiagnostics("B", {})
            result = camera.find_object()
            camera.close()

            self.assertTrue(result[0])
            np.testing.assert_array_equal(result[1], depth_data)
            self.assertEqual(result[2].shape, (2, 2))
            rows = trace_rows(root)
            frame_row = [row for row in rows if row["event"] == "frame"][0]
            artifacts = [row for row in rows if row["event"] == "artifacts_saved"][0]
            self.assertEqual(artifacts["frame_index"], frame_row["frame_index"])
            self.assertIn("+", frame_row["acquired_host_time"])
            self.assertGreater(frame_row["acquired_monotonic_ns"], 0)
            self.assertTrue(Path(artifacts["depth_path"]).is_file())
            self.assertTrue(Path(artifacts["color_path"]).is_file())
            self.assertEqual(rows[-1]["event"], "camera_close")

    def test_timeout_and_restart_are_traced(self):
        with tempfile.TemporaryDirectory() as root, patch.dict(CONFIG.__dict__, {"RUNTIME_ROOT": root, "DEBUGGING": False}):
            depth_data = np.zeros((1, 1), dtype=np.uint16)
            depth_frame = frame(1, 1000.0, depth_data)
            frames = SimpleNamespace(get_depth_frame=lambda: depth_frame)
            camera = ObjCamera.__new__(ObjCamera)
            camera.pipeline = SimpleNamespace(stop=lambda: None, start=lambda config: None)
            camera.config = object()
            camera._wait_for_frames = Mock(side_effect=[RuntimeError("timeout"), frames])
            camera._build_height_map_from_depth_frame = lambda _: np.zeros((1, 1))
            camera._update_object_state = lambda _: (False, False)
            camera.frame_diagnostics = RealSenseFrameDiagnostics("B", {})
            with patch("hardware.camera.time.sleep"):
                self.assertFalse(camera.find_object()[0])
            camera.close()
            self.assertEqual([row["event"] for row in trace_rows(root)], [
                "camera_open", "wait_error", "pipeline_restarted", "frame", "camera_close"
            ])

    def test_retry_timeout_is_traced_and_still_propagates(self):
        with tempfile.TemporaryDirectory() as root, patch.dict(CONFIG.__dict__, {"RUNTIME_ROOT": root, "DEBUGGING": False}):
            camera = ObjCamera.__new__(ObjCamera)
            camera.pipeline = SimpleNamespace(stop=lambda: None, start=lambda config: None)
            camera.config = object()
            camera._wait_for_frames = Mock(side_effect=[RuntimeError("first timeout"), RuntimeError("second timeout")])
            camera.frame_diagnostics = RealSenseFrameDiagnostics("B", {})
            with patch("hardware.camera.time.sleep"):
                with self.assertRaisesRegex(RuntimeError, "second timeout"):
                    camera.find_object()
            camera.close()
            errors = [row for row in trace_rows(root) if row["event"] == "wait_error"]
            self.assertEqual([row["phase"] for row in errors], ["initial", "after_restart"])

    def test_restart_failure_is_traced_and_still_propagates(self):
        with tempfile.TemporaryDirectory() as root, patch.dict(CONFIG.__dict__, {"RUNTIME_ROOT": root, "DEBUGGING": False}):
            camera = ObjCamera.__new__(ObjCamera)
            camera.pipeline = SimpleNamespace(
                stop=lambda: None,
                start=Mock(side_effect=RuntimeError("USB unavailable")),
            )
            camera.config = object()
            camera._wait_for_frames = Mock(side_effect=RuntimeError("timeout"))
            camera.frame_diagnostics = RealSenseFrameDiagnostics("B", {})
            with patch("hardware.camera.time.sleep"):
                with self.assertRaisesRegex(RuntimeError, "USB unavailable"):
                    camera.find_object()
            camera.close()
            rows = trace_rows(root)
            self.assertEqual([row["event"] for row in rows], [
                "camera_open", "wait_error", "pipeline_restart_error", "camera_close"
            ])

    def test_missing_depth_keeps_existing_none_result(self):
        with tempfile.TemporaryDirectory() as root, patch.dict(CONFIG.__dict__, {"RUNTIME_ROOT": root, "DEBUGGING": True}):
            camera = ObjCamera.__new__(ObjCamera)
            camera.pipeline = SimpleNamespace(stop=lambda: None)
            camera._wait_for_frames = Mock(return_value=SimpleNamespace(get_depth_frame=lambda: None))
            camera.frame_diagnostics = RealSenseFrameDiagnostics("B", {})
            self.assertIsNone(camera.find_object())
            camera.close()
            row = [item for item in trace_rows(root) if item["event"] == "frame"][0]
            self.assertIsNone(row["depth"])
            self.assertIsNone(row["color"])

    def test_color_getter_failure_with_missing_depth_keeps_none_result(self):
        with tempfile.TemporaryDirectory() as root, patch.dict(CONFIG.__dict__, {"RUNTIME_ROOT": root, "DEBUGGING": True}):
            frames = SimpleNamespace(
                get_depth_frame=lambda: None,
                get_color_frame=Mock(side_effect=RuntimeError("RGB stream failed")),
            )
            camera = ObjCamera.__new__(ObjCamera)
            camera.pipeline = SimpleNamespace(stop=lambda: None)
            camera._wait_for_frames = Mock(return_value=frames)
            camera.frame_diagnostics = RealSenseFrameDiagnostics("B", {})
            self.assertIsNone(camera.find_object())
            camera.close()
            row = [item for item in trace_rows(root) if item["event"] == "frame"][0]
            self.assertIsNone(row["depth"])
            self.assertEqual(row["color_frame_error"], "RGB stream failed")

    def test_color_payload_failure_is_logged_without_losing_depth_result(self):
        with tempfile.TemporaryDirectory() as root, patch.dict(CONFIG.__dict__, {"RUNTIME_ROOT": root, "DEBUGGING": True}):
            depth_data = np.zeros((1, 1), dtype=np.uint16)
            depth_frame = frame(1, 1000.0, depth_data)
            color_frame = frame(2, 1000.0, None)
            color_frame.get_data = Mock(side_effect=RuntimeError("RGB unavailable"))
            frames = SimpleNamespace(get_depth_frame=lambda: depth_frame, get_color_frame=lambda: color_frame)
            camera = ObjCamera.__new__(ObjCamera)
            camera.pipeline = SimpleNamespace(stop=lambda: None)
            camera._wait_for_frames = Mock(return_value=frames)
            camera._build_height_map_from_depth_frame = lambda _: np.zeros((1, 1))
            camera._update_object_state = lambda _: (False, False)
            camera.frame_diagnostics = RealSenseFrameDiagnostics("B", {})
            self.assertFalse(camera.find_object()[0])
            camera.close()
            row = [item for item in trace_rows(root) if item["event"] == "frame"][0]
            self.assertEqual(row["color_data_error"], "RGB unavailable")
            self.assertEqual(row["color"]["number"], 2)


if __name__ == "__main__":
    unittest.main()
