"""Test the real RTC wrapper and clock parser with a simulated hardware calendar."""
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
gcc = shutil.which("gcc")
if not gcc:
    raise SystemExit("Desktop GCC is required.")
with tempfile.TemporaryDirectory(prefix="watch-clock-") as out:
    exe = str(Path(out) / "clock.exe")
    subprocess.run([gcc, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-Itests/clock/host", "-Iinterface", "-Iapplication",
                    "tests/clock/clock_harness.c", "interface/watch_rtc.c",
                    "application/app_clock.c", "application/app_weather.c", "-o", exe], cwd=root, check=True)
    subprocess.run([exe], cwd=root, check=True)
    phone_fixture = Path(r"C:\Users\27795\Downloads\bluetooth-wave-work\build\watch-sync-frames.txt")
    if phone_fixture.exists():
        subprocess.run([exe, str(phone_fixture)], cwd=root, check=True)
    weather_exe = str(Path(out) / "weather.exe")
    subprocess.run([gcc, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror", "-Iapplication",
                    "tests/clock/weather_harness.c", "application/app_weather.c", "-o", weather_exe], cwd=root, check=True)
    subprocess.run([weather_exe], cwd=root, check=True)
