from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from protocol import Command, Mode


@unittest.skipUnless(shutil.which("g++"), "Optional host C++ compiler not installed")
class FirmwareInteropTests(unittest.TestCase):
    def test_all_modes_and_settings_match_actual_firmware_codec(self):
        folder = Path(__file__).resolve().parent
        firmware = folder.parents[2] / "lib" / "commProtocol" / "src"
        with tempfile.TemporaryDirectory() as temporary:
            binary = Path(temporary) / "protocol_reference"
            subprocess.run(["g++", "-std=c++11", "-I", str(firmware),
                            str(folder / "protocol_reference.cpp"),
                            str(firmware / "commProtocol.cpp"), "-o", str(binary)],
                           check=True, capture_output=True)
            output = subprocess.run([str(binary)], check=True, capture_output=True, text=True).stdout
        lines = output.splitlines()
        self.assertEqual(len(lines), 168)
        for line in lines:
            mode, setting, angle, expected = line.split()
            with self.subTest(mode=mode, setting=setting, angle=angle):
                self.assertEqual(Command(Mode(int(mode)), int(setting), int(angle)).encode().hex(), expected)
