"""Execute real app_task.c with HAL/module/RTOS stubs; does not flash hardware."""
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
gcc = shutil.which("gcc")
if not gcc:
    raise SystemExit("Desktop GCC is required.")
with tempfile.TemporaryDirectory(prefix="watch-startup-") as out:
    # Execute the actual main startup prefix, without duplicating its GPIO setup.
    main_source = (root / "Core/Src/main.c").read_text(encoding="utf-8")
    entry = main_source.index("int main(void)")
    prefix = main_source[main_source.index("{", entry) + 1:main_source.index("  MX_DMA_Init();", entry)]
    (Path(out) / "main_prefix.inc").write_text("static void boot_entry(void)\n{\n" + prefix + "}\n", encoding="utf-8")
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
        "-Iapplication", "-Iinterface", "-Icommon",
        "-IMiddlewares/LVGL/lvgl/examples/porting",
        "tests/startup/startup_harness.c", "tests/diagnostics/diag_stub.c", "-o", exe,
    ], cwd=root, check=True)
    for mode in ["ok", "devices_failed", "display_alloc_failed", "sensor_alloc_failed", "restored", "load_error",
                 "cal_success", "cal_failed", "save_error", "manual_cal", "watchdog_progress", "watchdog_wrap",
                 "watchdog_startup", "watchdog_startup_wrap", "first_frame_stall", "first_frame_slow",
                 "first_frame_deadline", "first_frame_sparse", "runtime_slow", "runtime_sparse",
                 "runtime_sensor_stalled", "runtime_deadline", "baro_cadence", "touch_alloc_failed", "touch_overrun",
                 "power_background", "power_idle", "power_motion", "power_heart", "power_calibration"]:
        subprocess.run([exe, mode], cwd=root, check=True)
