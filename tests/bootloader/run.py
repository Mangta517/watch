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
    ("f411_recovery", "f411_recovery", 0x08008000, 0x8000),
    ("f411ceu6", "f411ceu6", 0x08000000, 0x60000),
    ("f411ceu6_upgrade_app", "f411ceu6_upgrade_app", 0x08010000, 0x50000),
]:
    project = ET.parse(root / "MDK-ARM" / (project_name + ".uvprojx"))
    target = project.find("./Targets/Target[TargetName='" + target_name + "']")
    assert target is not None
    linker = target.find(".//LDads")
    if project_name in ["f411_bootloader", "f411_recovery"]:
        assert linker.findtext("umfTarg") == "1" and linker.findtext("useFile") == "0"
    else:
        assert linker.findtext("umfTarg") == "0" and linker.findtext("useFile") == "1"
        scatter = (root / "MDK-ARM" / (project_name + "_flash_safe.sct")).read_text(encoding="utf-8")
        assert "LR_IROM1 0x%08X 0x%08X" % (start, size) in scatter
        assert "RW_IRAM1 0x20000000 0x00020000" in scatter
        assert "int_battery_log.o (battery_log_ram)" in scatter
    region = target.find(".//OCR_RVCT4")
    assert int(region.findtext("StartAddress"), 16) == start
    assert int(region.findtext("Size"), 16) == size
    ram = target.find(".//OCR_RVCT9")
    assert int(ram.findtext("StartAddress"), 16) == 0x20000000
    assert int(ram.findtext("Size"), 16) == 0x20000
    defines = target.findtext(".//Cads/VariousControls/Define")
    if project_name != "f411_bootloader":
        assert "USER_VECT_TAB_ADDRESS" in defines
        assert "VECT_TAB_OFFSET=" + hex(start - 0x08000000) in defines
    else:
        assert "F411_BOOTLOADER" in defines
    if project_name == "f411_bootloader":
        assert target.find(".//File[FileName='app_boot_select.c']") is not None
        for excluded in ["kt6368a.c", "int_bootloader.c", "app_bootloader.c"]:
            assert target.find(".//File[FileName='" + excluded + "']") is None
    if project_name == "f411_recovery":
        assert target.find(".//File[FileName='recovery_main.c']") is not None
        assert target.find(".//File[FileName='int_bootloader.c']") is not None

with tempfile.TemporaryDirectory(prefix="watch-bootloader-") as output:
    output = Path(output)
    header = (root / "tests/ble/host/main.h").read_text(encoding="utf-8")
    header = header.replace("    UART_InitTypeDef Init;", "    UART_InitTypeDef Init;\n    void *hdmarx;")
    header += '\n#include "' + (root / "tests/bootloader/host/flash.h").as_posix() + '"\n'
    (output / "main.h").write_text(header, encoding="utf-8")
    shutil.copyfile(root / "tests/ble/host/usart.h", output / "usart.h")
    crc_defines = []
    for size in [101, 512, 10001, 327680]:
        image = struct.pack("<II", 0x20020000, 0x08010009) + bytes((i * 37 + 11) & 255 for i in range(8, size))
        crc_defines.append(f"#define TEST_CRC_{size} 0x{zlib.crc32(image):08X}U\n")
    (output / "crc_vectors.h").write_text("".join(crc_defines), encoding="ascii")
    executable = output / "bootloader.exe"
    subprocess.run([
        gcc, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
        "-Wno-int-to-pointer-cast", "-Wno-missing-field-initializers", "-DF411_BOOTLOADER",
        "-I" + str(output), "-Iinterface", "-Iapplication", "-Icommon",
        "tests/bootloader/bootloader_harness.c", "interface/kt6368a.c",
        "interface/int_bootloader.c", "application/app_bootloader.c", "application/app_boot_select.c", "interface/bl24c02.c", "common/platform_delay.c", "-o", str(executable),
    ], cwd=root, check=True)
    subprocess.run([str(executable)], cwd=root, check=True)

    entry_executable = output / "boot_entry.exe"
    subprocess.run([
        gcc, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror", "-DF411_BOOTLOADER",
        "-Itests/bootloader/entry_host", "-Ibootloader", "-Iapplication", "-Icommon",
        "tests/bootloader/boot_entry_harness.c", "-o", str(entry_executable),
    ], cwd=root, check=True)
    subprocess.run([str(entry_executable)], cwd=root, check=True)

subprocess.run([sys.executable, "-B", str(root / "tests/bootloader/sender_test.py")], cwd=root, check=True)
