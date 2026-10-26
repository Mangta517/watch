"""Execute real Flash logger and BLE protocol against a bounded Flash/UART model; no flashing."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[2]
gcc = shutil.which("gcc")
if not gcc:
    raise SystemExit("Desktop GCC is required.")
for name, size in [("f411ceu6", 0x60000), ("f411ceu6_upgrade_app", 0x58000)]:
    target = ET.parse(root / "MDK-ARM" / (name + ".uvprojx")).find(".//Target")
    assert int(target.findtext(".//OCR_RVCT4/Size"), 16) == size
    for source in ["app_battery_log.c", "app_battery_log_ble.c", "int_battery_log.c"]:
        assert target.find(".//File[FileName='" + source + "']") is not None
with tempfile.TemporaryDirectory(prefix="watch-battery-log-") as output:
    exe = str(Path(output) / "battery-log.exe")
    subprocess.run([gcc, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror", "-Wno-int-to-pointer-cast",
                    "-Itests/battery_log/host", "-Itests/sensors/host", "-Iapplication", "-Iinterface",
                    "tests/battery_log/battery_log_harness.c", "application/app_battery_log.c",
                    "application/app_battery_log_ble.c", "application/app_reset_ble.c",
                    "interface/int_battery_log.c", "-o", exe], cwd=root, check=True)
    for mode in ["record", "full", "faults", "unknown", "allocation", "ble", "clear", "erase_error",
                 "reset", "reset_torn", "reset_full", "reset_history"]:
        subprocess.run([exe, mode], cwd=root, check=True)
