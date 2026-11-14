"""Link the real BLE coordinator without LVGL; each boot scenario gets a process."""
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
gcc = shutil.which("gcc")
if not gcc:
    raise SystemExit("Desktop GCC is required.")

with tempfile.TemporaryDirectory(prefix="watch-ble-service-") as out:
    for debug in [0, 1]:
        exe = str(Path(out) / f"ble_service_{debug}.exe")
        subprocess.run([
            gcc, "-std=c99", "-O1", "-g", "-Wall", "-Wextra", "-Werror",
            "-include", "tests/ui/host/platform.h", "-Itests/ui/host", "-ICore/Inc",
            "-Iapplication", "-Iinterface", "-Icommon", f"-DAPP_DEBUG_TELEMETRY={debug}",
            "tests/ble_service/ble_service_harness.c", "application/app_ble_service.c",
            "application/app_clock.c", "application/app_weather.c", "-o", exe,
        ], cwd=root, check=True)
        for mode in ["receive_power", "uptime", "uptime_error", "uptime_timeout", "ownership", "uptime_heart"]:
            subprocess.run([exe, mode], cwd=root, check=True)
