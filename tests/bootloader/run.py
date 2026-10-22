"""Execute real bootloader, application state machine and KT6368A callbacks with simulated Flash/DMA."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET
import struct
import zlib
import sys

root = Path(__file__).resolve().parents[2]
gcc = shutil.which("gcc")
if not gcc:
    raise SystemExit("Desktop GCC is required.")

for project_name, target_name, start, size in [
    ("f411_bootloader", "f411_bootloader", 0x08000000, 0x8000),
    ("f411ceu6", "f411ceu6", 0x08000000, 0x80000),
    ("f411ceu6_upgrade_app", "f411ceu6_upgrade_app", 0x08008000, 0x78000),
]:
    project = ET.parse(root / "MDK-ARM" / (project_name + ".uvprojx"))
    target = project.find("./Targets/Target[TargetName='" + target_name + "']")
    assert target is not None
    linker = target.find(".//LDads")
    assert linker.findtext("umfTarg") == "1" and linker.findtext("useFile") == "0", "Expected target-generated scatter"
    region = target.find(".//OCR_RVCT4")
    assert int(region.findtext("StartAddress"), 16) == start
    assert int(region.findtext("Size"), 16) == size
    defines = target.findtext(".//Cads/VariousControls/Define")
    if project_name != "f411_bootloader":
        assert "USER_VECT_TAB_ADDRESS" in defines
        assert "VECT_TAB_OFFSET=" + hex(start - 0x08000000) in defines
    else:
        assert "F411_BOOTLOADER" in defines

with tempfile.TemporaryDirectory(prefix="watch-bootloader-") as output:
    output = Path(output)
    header = (root / "tests/ble/host/main.h").read_text(encoding="utf-8")
    header = header.replace("    UART_InitTypeDef Init;", "    UART_InitTypeDef Init;\n    void *hdmarx;")
    header += '\n#include "' + (root / "tests/bootloader/host/flash.h").as_posix() + '"\n'
    (output / "main.h").write_text(header, encoding="utf-8")
    shutil.copyfile(root / "tests/ble/host/usart.h", output / "usart.h")
    crc_defines = []
    for size in [101, 512, 10001, 491520]:
        image = struct.pack("<II", 0x20020000, 0x08008009) + bytes((i * 37 + 11) & 255 for i in range(8, size))
        crc_defines.append(f"#define TEST_CRC_{size} 0x{zlib.crc32(image):08X}U\n")
    (output / "crc_vectors.h").write_text("".join(crc_defines), encoding="ascii")
    executable = output / "bootloader.exe"
    subprocess.run([
        gcc, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
        "-Wno-int-to-pointer-cast", "-Wno-missing-field-initializers", "-DF411_BOOTLOADER",
        "-I" + str(output), "-Iinterface", "-Iapplication",
        "tests/bootloader/bootloader_harness.c", "interface/kt6368a.c",
        "interface/int_bootloader.c", "application/app_bootloader.c", "-o", str(executable),
    ], cwd=root, check=True)
    subprocess.run([str(executable)], cwd=root, check=True)

subprocess.run([sys.executable, "-B", str(root / "tests/bootloader/sender_test.py")], cwd=root, check=True)
