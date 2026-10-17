"""Compile the real TPS3823 driver with HAL/RTOS stubs; does not flash hardware."""
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
gcc = shutil.which("gcc")
if not gcc:
    raise SystemExit("Desktop GCC is required.")

with tempfile.TemporaryDirectory(prefix="watch-watchdog-") as out:
    exe = str(Path(out) / "watchdog.exe")
    subprocess.run([
        gcc, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
        "-include", "tests/sensors/host/i2c_platform.h",
        "-Itests/sensors/host", "-ICore/Inc", "-Iinterface",
        "tests/watchdog/watchdog_harness.c", "interface/tps3823.c", "-o", exe,
    ], cwd=root, check=True)
    for mode in ["running", "before_scheduler", "suspended"]:
        subprocess.run([exe, mode], cwd=root, check=True)
