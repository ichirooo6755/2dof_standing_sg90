"""A macOS tcdrain failure must be saved as a transport failure."""

import contextlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
from unittest import mock

try:
    import termios
except ImportError:
    raise SystemExit("SKIP: termios is unavailable")

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location("standing_monitor", ROOT / "tools/standing_monitor.py")
monitor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(monitor)


class LostSerial:
    def __init__(self, *args, **kwargs):
        pass

    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False

    def write(self, data):
        return len(data)

    def flush(self):
        raise termios.error(6, "Device not configured")


with tempfile.TemporaryDirectory() as directory:
    args = ["standing_monitor.py", "--port", "mock", "--seconds", "1", "--out", directory]
    with mock.patch.object(sys, "argv", args), mock.patch.object(monitor.serial, "Serial", LostSerial):
        with contextlib.redirect_stdout(io.StringIO()):
            try:
                monitor.main()
            except SystemExit as error:
                assert error.code == 1
            else:
                raise AssertionError("transport failure must return nonzero")
    result = json.loads((Path(directory) / "summary.json").read_text())
    assert "Device not configured" in result["recording_error"]
    assert result["frames_received"] == 0
    assert result["motion_commands_sent"] is False
print("PASS: USB tcdrain disconnect is recorded and exits nonzero")
