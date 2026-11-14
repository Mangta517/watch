"""Execute real app_task.c with HAL/module/RTOS stubs; does not flash hardware."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(root / "tests"))
from source_extract import extract_functions
gcc = shutil.which("gcc")
if not gcc:
    raise SystemExit("Desktop GCC is required.")
with tempfile.TemporaryDirectory(prefix="watch-startup-") as out:
    lifecycle = "\n".join(extract_functions(root / "application" / (module + ".c"), names) for module, names in [
        ("app_imu", ["app_imu_init_hw", "app_imu_init_mag_hw", "app_imu_recover_step", "app_imu_mag_recover_step",
                     "app_imu_calibration_begin", "app_imu_calibration_progress", "app_imu_restore_offset", "app_imu_get_offset"]),
        ("app_baro", ["app_baro_init_hw", "app_baro_set_sleep", "app_baro_recover_step"]),
        ("app_heart", ["app_heart_init_hw"]),
    ])
    (Path(out) / "sensor_lifecycle.inc").write_text(lifecycle, encoding="utf-8")
    # Execute the actual main startup prefix, without duplicating its GPIO setup.
    main_source = (root / "Core/Src/main.c").read_text(encoding="utf-8")
    entry = main_source.index("int main(void)")
    prefix = main_source[main_source.index("{", entry) + 1:main_source.index("  MX_DMA_Init();", entry)]
    (Path(out) / "main_prefix.inc").write_text("static void boot_entry(void)\n{\n" + prefix + "}\n", encoding="utf-8")
    power_start = main_source.index("/* USER CODE BEGIN 2 */", entry) + len("/* USER CODE BEGIN 2 */")
    power_config = main_source[power_start:main_source.index("    App_Task_Create();", power_start)]
    (Path(out) / "main_power.inc").write_text("static void boot_power_config(void)\n{\n" + power_config + "}\n", encoding="utf-8")
    power_exe = str(Path(out) / "power.exe")
    subprocess.run([
        gcc, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
        "-Itests/sensors/host", "-ICore/Inc", "-I" + out,
        "tests/startup/power_harness.c", "-o", power_exe,
    ], cwd=root, check=True)
    subprocess.run([power_exe], cwd=root, check=True)
    exe = str(Path(out) / "startup.exe")
    subprocess.run([
        gcc, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
        "-Itests/startup/host", "-Itests/sensors/host", "-ICore/Inc",
        "-Iapplication", "-Iinterface", "-Icommon", "-I" + out,
        "-IMiddlewares/LVGL/lvgl/examples/porting",
        "tests/startup/startup_harness.c", "application/app_uptime.c", "application/app_weather.c",
        "application/app_weather_store.c", "tests/diagnostics/diag_stub.c", "-o", exe,
    ], cwd=root, check=True)
    for mode in ["ok", "devices_failed", "display_alloc_failed", "sensor_alloc_failed", "restored", "load_error",
                 "cal_success", "cal_failed", "save_error", "manual_cal", "watchdog_progress", "watchdog_wrap",
                 "watchdog_startup", "watchdog_startup_wrap", "watchdog_stop", "watchdog_stop_wrap",
                 "first_frame_stall", "first_frame_slow",
                 "first_frame_deadline", "first_frame_sparse", "runtime_slow", "runtime_sparse",
                 "runtime_sensor_stalled", "runtime_deadline", "baro_cadence", "baro_calibration", "baro_calibration_boundary", "baro_calibration_boundary_wrap",
                 "environment_policy", "environment_policy_wrap", "environment_policy_sleep_retry",
                 "reference_recovery", "environment_policy_early", "touch_alloc_failed", "touch_overrun",
                 "power_background", "power_idle", "power_motion", "power_heart", "power_calibration", "power_heart_calibration",
                 "sleep_cycle", "sleep_wrap", "sleep_calibration", "sleep_heart_early", "sleep_heart_offset",
                 "sleep_stationary", "sleep_stationary_wrap", "sleep_stationary_recovery", "sleep_stationary_wake", "sleep_stationary_heart",
                 "sleep_recovery", "sleep_recovery_wrap",
                 "bluetooth_resume", "sport_cycle", "sport_save", "sport_uptime", "sport_weather", "touch_sleep", "touch_sleep_early",
                 "background_cycle", "background_wrap", "background_calibration", "lvgl_wake",
                 "sensor_timing", "sensor_timing_overload", "sensor_timing_wrap", "sensor_timing_exact"]:
        subprocess.run([exe, mode], cwd=root, check=True)
