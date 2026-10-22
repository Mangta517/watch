"""Check the host sender handshake without opening a serial port."""
import importlib.util
from pathlib import Path
import struct
import unittest
from unittest.mock import patch
import zlib

module_path = Path(__file__).resolve().parents[2] / "tools/send_bootloader.py"
spec = importlib.util.spec_from_file_location("send_bootloader", module_path)
sender = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sender)


class Connection:
    def __init__(self, image, error=None):
        self.image = image
        self.error = error
        self.writes = []
        self.ready = False
        self.read_count = 0

    def write(self, data):
        if self.writes:
            assert self.ready, "BIN sent before ready"
        self.writes.append(data)
        return len(data)

    def readline(self):
        self.read_count += 1
        if self.error:
            return self.error
        if self.read_count == 1:
            return b"Received start command\r\n"
        if not self.ready:
            assert len(self.writes) == 1
            self.ready = True
            return b"Flash erased, ready to receive data...\r\n"
        assert b"".join(self.writes[1:]) == self.image
        return f"ok,len:{len(self.image)}\n".encode("ascii")


class SenderTest(unittest.TestCase):
    def setUp(self):
        self.image = struct.pack("<II", 0x20020000, 0x08008009) + bytes(range(256)) * 9 + b"x"

    def test_crc_ready_and_exact_payload(self):
        connection = Connection(self.image)
        sender.send_image(connection, self.image)
        self.assertEqual(connection.writes[0], f"start:{len(self.image)}:{zlib.crc32(self.image):08x}\n".encode("ascii"))
        self.assertEqual(b"".join(connection.writes[1:]), self.image)

    def test_invalid_file_is_rejected_before_start(self):
        for image in [b"", b"x" * 8, b"x" * (0x78000 + 1), struct.pack("<II", 0x20020000, 0x08000339) + b"x" * 20]:
            connection = Connection(image)
            with self.assertRaises(ValueError):
                sender.send_image(connection, image)
            self.assertEqual(connection.writes, [])

    def test_device_error_stops_before_payload(self):
        connection = Connection(self.image, b"app rec error,system reset\n")
        with self.assertRaises(RuntimeError):
            sender.send_image(connection, self.image)
        self.assertEqual(len(connection.writes), 1)

    def test_ready_timeout(self):
        connection = Connection(self.image)
        with patch.object(sender.time, "monotonic", side_effect=[0, 61]):
            with self.assertRaises(TimeoutError):
                sender.send_image(connection, self.image)
        self.assertEqual(len(connection.writes), 1)


if __name__ == "__main__":
    unittest.main()
