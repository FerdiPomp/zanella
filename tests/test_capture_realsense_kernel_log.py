import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from capture_realsense_kernel_log import capture_kernel_log


class CaptureKernelLogTests(unittest.TestCase):
    def test_saves_selected_boot_and_time_window(self):
        with tempfile.TemporaryDirectory() as root:
            output = Path(root) / ".runtime" / "kernel_usb.log"
            with patch("capture_realsense_kernel_log.subprocess.run", return_value=SimpleNamespace(
                returncode=0, stdout="kernel: usb disconnect\n", stderr=""
            )) as run:
                self.assertEqual(capture_kernel_log("2026-11-11 11:16:20", "2026-11-11 11:17:00", -1, output), output)
            self.assertEqual(run.call_args.args[0], [
                "journalctl", "-k", "-b", "-1", "--since", "2026-11-11 11:16:20",
                "--until", "2026-11-11 11:17:00", "--no-pager",
            ])
            contents = output.read_text(encoding="utf-8")
            self.assertIn("Boot: -1", contents)
            self.assertIn("kernel: usb disconnect", contents)

    def test_journal_failure_does_not_create_misleading_output(self):
        with tempfile.TemporaryDirectory() as root:
            output = Path(root) / "kernel_usb.log"
            with patch("capture_realsense_kernel_log.subprocess.run", return_value=SimpleNamespace(
                returncode=1, stdout="", stderr="journal unavailable"
            )):
                with self.assertRaisesRegex(RuntimeError, "journal unavailable"):
                    capture_kernel_log("yesterday", "today", 0, output)
            self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
