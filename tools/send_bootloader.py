"""Send an upgrade APP BIN over classic Bluetooth SPP with CRC-32."""
import argparse
from pathlib import Path
import struct
import time
import zlib


def build_command(image):
    if not 8 <= len(image) <= 0x50000:
        raise ValueError("BIN must contain 8..327680 bytes")
    stack, reset = struct.unpack_from("<II", image)
    if not (0x20000000 < stack <= 0x20020000 and stack & 7 == 0 and
            reset & 1 and 0x08010008 <= (reset & ~1) < 0x08010000 + len(image)):
        raise ValueError("BIN vectors do not match the upgrade APP at 0x08010000")
    return f"start:{len(image)}:{zlib.crc32(image):08x}\n".encode("ascii")


def wait_reply(connection, expected):
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline:
        line = connection.readline().strip()
        if line.startswith((b"app rec error", b"Invalid")):
            raise RuntimeError(line.decode("ascii", errors="replace"))
        if expected.startswith(b"OTA,LAYOUT,") and line.startswith(b"OTA,LAYOUT,") and line != expected:
            raise RuntimeError("Flash layout mismatch; install the selector and independent recovery first")
        if line == expected:
            return
    raise TimeoutError(f"Timed out waiting for {expected.decode('ascii')}")


def send_image(connection, image):
    command = build_command(image)
    query = b"OTA,INFO\n"
    if connection.write(query) != len(query):
        raise OSError("Incomplete layout query write")
    wait_reply(connection, b"OTA,LAYOUT,08010000,00050000")
    if connection.write(command) != len(command):
        raise OSError("Incomplete start command write")
    wait_reply(connection, b"Flash erased, ready to receive data...")
    for offset in range(0, len(image), 1024):
        chunk = image[offset:offset + 1024]
        if connection.write(chunk) != len(chunk):
            raise OSError("Incomplete BIN write; reconnect and restart the transfer")
    wait_reply(connection, f"ok,len:{len(image)}".encode("ascii"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="SPP serial port, e.g. COM5")
    parser.add_argument("bin", type=Path, help="BIN linked at 0x08010000")
    args = parser.parse_args()
    import serial

    try:
        image = args.bin.read_bytes()
        command = build_command(image)
        print(f"{command.decode('ascii').strip()}; waiting for ready, then sending BIN", flush=True)
        with serial.Serial(args.port, 115200, timeout=1, write_timeout=5) as connection:
            send_image(connection, image)
    except (OSError, ValueError, RuntimeError, serial.SerialException) as error:
        parser.exit(1, f"Transfer failed: {error}\n")
    print(f"ok,len:{len(image)}")


if __name__ == "__main__":
    main()
