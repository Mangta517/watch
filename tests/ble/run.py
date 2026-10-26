"""Compile the real KT6368A driver with simulated UART/GPIO; no hardware flashing."""
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
gcc = shutil.which("gcc")
if not gcc:
    raise SystemExit("Desktop GCC is required.")
with tempfile.TemporaryDirectory(prefix="watch-ble-") as out:
    exe = str(Path(out) / "ble.exe")
    subprocess.run([
        gcc, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
        "-Itests/ble/host", "-Iinterface", "tests/ble/ble_harness.c",
        "interface/kt6368a.c", "-o", exe,
    ], cwd=root, check=True)
    subprocess.run([exe], cwd=root, check=True)
    sleep_exe = str(Path(out) / "rx-sleep.exe")
    subprocess.run([
        gcc, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
        "-flto", "-ffunction-sections", "-fdata-sections", "-Wl,--gc-sections",
        "-Itests/ble/host", "-Itests/startup/host", "-Iapplication", "-Iinterface", "-Icommon",
        "-IMiddlewares/LVGL/lvgl/examples/porting",
        "tests/ble/rx_sleep_harness.c", "-o", sleep_exe,
    ], cwd=root, check=True)
    subprocess.run([sleep_exe], cwd=root, check=True)
