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
sources = ["common/platform_delay.c", "tests/sensors/sensor_harness.c", "interface/lsm303dlhc.c",
           "interface/mpu6050.c", "interface/spl006.c",
           "interface/bl24c02.c", "application/app_mag_store.c", "application/app_imu_store.c", "application/app_uptime.c",
           "common/common_imu.c", "common/common_activity.c", "application/app_imu.c", "application/app_baro.c"]
with tempfile.TemporaryDirectory(prefix="watch-sensors-") as out:
    battery_exe = str(Path(out) / "battery.exe")
    subprocess.run([gcc, "tests/diagnostics/diag_stub.c", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-Itests/sensors/host", "-ICore/Inc", "-Iapplication", "-Iinterface",
                    "tests/sensors/battery_harness.c", "application/app_battery.c", "interface/battery_adc.c",
                    "-o", battery_exe], cwd=root, check=True)
    subprocess.run([battery_exe], cwd=root, check=True)
    subprocess.run([battery_exe, "percentage"], cwd=root, check=True)
    subprocess.run([battery_exe, "calibration"], cwd=root, check=True)
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
    subprocess.run([gcc, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror", "-DF411_BOOTLOADER",
                    "-include", "tests/sensors/host/i2c_platform.h",
                    "-Itests/sensors/host", "-ICore/Inc", "-Icommon",
                    "tests/sensors/i2c_harness.c", "common/my_i2c.c",
                    "-o", i2c_exe], cwd=root, check=True)
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
                    "tests/sensors/aht21_harness.c", "interface/aht21.c", "common/platform_delay.c", "application/app_aht21.c",
                    "-lm", "-o", aht21_exe], cwd=root, check=True)
    subprocess.run([aht21_exe], cwd=root, check=True)
    exe = str(Path(out) / "sensors.exe")
    args = [gcc, "tests/diagnostics/diag_stub.c", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
            "-include", "tests/sensors/host/platform.h"]
    args += ["-I" + p for p in ["tests/sensors/host", "Core/Inc", "application", "interface", "common"]]
    subprocess.run(args + sources + ["-lm", "-o", exe], cwd=root, check=True)
    for mode in ["compass", "accel_warmup", "independent", "day_cold", "day_retained", "mag_calibration", "mag_save_failed", "mag_calibration_reference",
                 "activity", "activity_high_rate", "uptime", "async_save", "mpu_stale", "mpu_sleep_recovery", "mag_sleep_recovery", "recovery"]:
        subprocess.run([exe, mode], cwd=root, check=True)
    subprocess.run([exe], cwd=root, check=True)
    driver_exe = str(Path(out) / "drivers.exe")
    subprocess.run([gcc, "tests/diagnostics/diag_stub.c", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-include", "tests/sensors/host/i2c_platform.h",
                    "-Itests/sensors/host", "-ICore/Inc", "-Iinterface", "-Icommon",
                    "tests/sensors/driver_harness.c", "interface/lcd.c", "common/platform_delay.c", "interface/touch.c", "common/my_i2c.c",
                    "-o", driver_exe], cwd=root, check=True)
    subprocess.run([driver_exe], cwd=root, check=True)
    heart_exe = str(Path(out) / "em7028.exe")
    subprocess.run([gcc, "tests/diagnostics/diag_stub.c", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-Itests/sensors/host", "-ICore/Inc", "-Iinterface", "-Icommon",
                    "tests/sensors/em7028_harness.c", "interface/em7028.c",
                    "-o", heart_exe], cwd=root, check=True)
    subprocess.run([heart_exe], cwd=root, check=True)
    service_exe = str(Path(out) / "heart-service.exe")
    heart_args = [gcc, "tests/diagnostics/diag_stub.c", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                  "-Itests/sensors/host", "-ICore/Inc", "-Iinterface", "-Iapplication", "-Icommon",
                  "tests/sensors/heart_harness.c", "application/app_heart.c", "-lm", "-o", service_exe]
    subprocess.run(heart_args, cwd=root, check=True)
    subprocess.run([service_exe, "telemetry_off"], cwd=root, check=True)
    subprocess.run(heart_args + ["-DAPP_DEBUG_TELEMETRY=1"], cwd=root, check=True)
    subprocess.run([service_exe], cwd=root, check=True)
    subprocess.run([service_exe, "store"], cwd=root, check=True)
    subprocess.run([service_exe, "allocation_failed"], cwd=root, check=True)
    motion_exe = str(Path(out) / "heart-motion.exe")
    subprocess.run([gcc, "tests/diagnostics/diag_stub.c", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-Itests/sensors/host", "-ICore/Inc", "-Iinterface", "-Iapplication", "-Icommon",
                    "tests/sensors/heart_motion_harness.c", "application/app_heart.c",
                    "-lm", "-o", motion_exe], cwd=root, check=True)
    subprocess.run([motion_exe], cwd=root, check=True)
