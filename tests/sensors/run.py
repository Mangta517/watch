"""Compile real sensor drivers, service, and fusion; replace only I2C/RTOS/HAL.
Run: python tests/sensors/run.py. Does not flash hardware.
"""
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
gcc = shutil.which("gcc")
if not gcc:
    raise SystemExit("Desktop GCC is required.")
sources = ["tests/sensors/sensor_harness.c", "interface/lsm303dlhc.c",
           "interface/mpu6050.c", "interface/spl006.c",
           "interface/bl24c02.c", "application/app_mag_store.c",
           "common/common_imu.c", "application/app_imu.c"]
with tempfile.TemporaryDirectory(prefix="watch-sensors-") as out:
    exe = str(Path(out) / "sensors.exe")
    args = [gcc, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
            "-include", "tests/sensors/host/platform.h"]
    args += ["-I" + p for p in ["tests/sensors/host", "Core/Inc", "application", "interface", "common"]]
    subprocess.run(args + sources + ["-lm", "-o", exe], cwd=root, check=True)
    subprocess.run([exe], cwd=root, check=True)
    subprocess.run([exe, "restored"], cwd=root, check=True)
    subprocess.run([exe, "load_failed"], cwd=root, check=True)
    subprocess.run([exe, "save_failed"], cwd=root, check=True)
