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
           "interface/bl24c02.c", "application/app_mag_store.c", "application/app_imu_store.c", "application/app_uptime.c",
           "common/common_imu.c", "common/common_activity.c", "application/app_imu.c", "application/app_baro.c"]
with tempfile.TemporaryDirectory(prefix="watch-sensors-") as out:
    battery_exe = str(Path(out) / "battery.exe")
    subprocess.run([gcc, "tests/diagnostics/diag_stub.c", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-Itests/sensors/host", "-ICore/Inc", "-Iapplication",
                    "tests/sensors/battery_harness.c", "application/app_battery.c",
                    "-o", battery_exe], cwd=root, check=True)
    subprocess.run([battery_exe], cwd=root, check=True)
    subprocess.run([battery_exe, "percentage"], cwd=root, check=True)
    subprocess.run([battery_exe, "allocation_failed"], cwd=root, check=True)
    i2c_exe = str(Path(out) / "i2c.exe")
    subprocess.run([gcc, "tests/diagnostics/diag_stub.c", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-include", "tests/sensors/host/i2c_platform.h",
                    "-Itests/sensors/host", "-ICore/Inc", "-Icommon",
                    "tests/sensors/i2c_harness.c", "common/my_i2c.c",
                    "-o", i2c_exe], cwd=root, check=True)
    subprocess.run([i2c_exe, "stop"], cwd=root, check=True)
    subprocess.run([i2c_exe, "suspend"], cwd=root, check=True)
    subprocess.run([i2c_exe], cwd=root, check=True)
    bus_exe = str(Path(out) / "bus-recovery.exe")
    subprocess.run([gcc, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-include", "tests/sensors/host/i2c_platform.h",
                    "-Itests/sensors/host", "-ICore/Inc", "-Icommon",
                    "tests/sensors/bus_recovery_harness.c", "common/my_i2c.c",
                    "-o", bus_exe], cwd=root, check=True)
    for mode in ["normal", "sda_release", "sda_stuck", "scl_stuck", "eeprom_nack", "eeprom_suspend", "tp_cache"]:
        subprocess.run([bus_exe, mode], cwd=root, check=True)
    aht21_exe = str(Path(out) / "aht21.exe")
    subprocess.run([gcc, "tests/diagnostics/diag_stub.c", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-Itests/sensors/host", "-ICore/Inc", "-Iinterface", "-Iapplication", "-Icommon",
                    "tests/sensors/aht21_harness.c", "interface/aht21.c", "application/app_aht21.c",
                    "-lm", "-o", aht21_exe], cwd=root, check=True)
    subprocess.run([aht21_exe], cwd=root, check=True)
    exe = str(Path(out) / "sensors.exe")
    args = [gcc, "tests/diagnostics/diag_stub.c", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
            "-include", "tests/sensors/host/platform.h"]
    args += ["-I" + p for p in ["tests/sensors/host", "Core/Inc", "application", "interface", "common"]]
    subprocess.run(args + sources + ["-lm", "-o", exe], cwd=root, check=True)
    subprocess.run([exe, "activity"], cwd=root, check=True)
    subprocess.run([exe, "background_mag_sleep_retry"], cwd=root, check=True)
    subprocess.run([exe, "uptime"], cwd=root, check=True)
    subprocess.run([exe, "async_save"], cwd=root, check=True)
    subprocess.run([exe, "polling"], cwd=root, check=True)
    for mode in ["mag_schedule", "mag_schedule_ppg", "mag_schedule_wrap"]:
        subprocess.run([exe, mode], cwd=root, check=True)
    subprocess.run([exe, "power_profile"], cwd=root, check=True)
    subprocess.run([exe, "mpu_stale"], cwd=root, check=True)
    subprocess.run([exe, "mpu_sleep_recovery"], cwd=root, check=True)
    subprocess.run([exe, "mag_sleep_recovery"], cwd=root, check=True)
    subprocess.run([exe, "recovery"], cwd=root, check=True)
    subprocess.run([exe], cwd=root, check=True)
    subprocess.run([exe, "restored"], cwd=root, check=True)
    subprocess.run([exe, "load_failed"], cwd=root, check=True)
    subprocess.run([exe, "save_failed"], cwd=root, check=True)
    driver_exe = str(Path(out) / "drivers.exe")
    subprocess.run([gcc, "tests/diagnostics/diag_stub.c", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-include", "tests/sensors/host/i2c_platform.h",
                    "-Itests/sensors/host", "-ICore/Inc", "-Iinterface", "-Icommon",
                    "tests/sensors/driver_harness.c", "interface/lcd.c", "interface/touch.c", "common/my_i2c.c",
                    "-o", driver_exe], cwd=root, check=True)
    subprocess.run([driver_exe], cwd=root, check=True)
    heart_exe = str(Path(out) / "em7028.exe")
    subprocess.run([gcc, "tests/diagnostics/diag_stub.c", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-Itests/sensors/host", "-ICore/Inc", "-Iinterface", "-Icommon",
                    "tests/sensors/em7028_harness.c", "interface/em7028.c",
                    "-o", heart_exe], cwd=root, check=True)
    subprocess.run([heart_exe], cwd=root, check=True)
    service_exe = str(Path(out) / "heart-service.exe")
    subprocess.run([gcc, "tests/diagnostics/diag_stub.c", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-Itests/sensors/host", "-ICore/Inc", "-Iinterface", "-Iapplication",
                    "tests/sensors/heart_harness.c", "application/app_heart.c",
                    "-lm", "-o", service_exe], cwd=root, check=True)
    subprocess.run([service_exe], cwd=root, check=True)
    motion_exe = str(Path(out) / "heart-motion.exe")
    subprocess.run([gcc, "tests/diagnostics/diag_stub.c", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-Itests/sensors/host", "-ICore/Inc", "-Iinterface", "-Iapplication",
                    "tests/sensors/heart_motion_harness.c", "application/app_heart.c",
                    "-lm", "-o", motion_exe], cwd=root, check=True)
    subprocess.run([motion_exe], cwd=root, check=True)
