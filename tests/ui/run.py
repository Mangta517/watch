"""Build and exercise the real LVGL UI on a desktop with GCC and Pillow.

Run from any directory: python tests/ui/run.py
Hardware GPIO, backlight and sensor snapshots are stubbed. Nothing is flashed.
"""
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "tests/ui/output"
OUT.mkdir(parents=True, exist_ok=True)
gcc = shutil.which("gcc")
if not gcc:
    raise SystemExit("Desktop GCC is required (for example MinGW on Windows).")
includes = ["tests/ui/host", "Core/Inc", "application", "interface", "Middlewares/LVGL", "Middlewares/LVGL/lvgl"]
sources = sorted((ROOT / "Middlewares/LVGL/lvgl/src").rglob("*.c"))
args = ["-std=c99", "-O1", "-g", "-include", "tests/ui/host/platform.h"]
args += ["-I" + path for path in includes]
exe = OUT / ("ui_harness.exe" if sys.platform == "win32" else "ui_harness")
library = OUT / "lvgl.o"
response = OUT / "build.rsp"
dependencies = sources + list((ROOT / "Middlewares/LVGL").rglob("*.h")) + list((ROOT / "tests/ui/host").glob("*.h"))
if not library.exists() or max(p.stat().st_mtime for p in dependencies) > library.stat().st_mtime:
    library_args = args + [p.relative_to(ROOT).as_posix() for p in sources]
    library_args += ["-r", "-o", library.relative_to(ROOT).as_posix()]
    response.write_text("\n".join(library_args), encoding="utf-8")
    print("Building the project's LVGL renderer...", flush=True)
    subprocess.run([gcc, "@" + str(response)], cwd=ROOT, check=True)
print("Building and testing the watch UI...", flush=True)
dma_exe = OUT / "dma_harness.exe"
subprocess.run([gcc, "-Itests/sensors/host"] + args + ["-Iinterface", "-Wall", "-Wextra", "-Werror",
               "tests/ui/dma_harness.c", str(library), "-lm", "-o", str(dma_exe)], cwd=ROOT, check=True)
subprocess.run([str(dma_exe)], cwd=ROOT, check=True)
subprocess.run([gcc, "tests/diagnostics/diag_stub.c"] + args + ["-DAPP_DEBUG_TELEMETRY=1", "-Wall", "-Wextra", "-Werror", "tests/ui/ui_harness.c", "application/ui_sydney_font.c", "application/app_clock.c", "application/app_weather.c", str(library), "-lm", "-o", str(exe)], cwd=ROOT, check=True)
subprocess.run([str(exe)], cwd=ROOT, check=True)
quiet_exe = OUT / "ui_quiet.exe"
subprocess.run([gcc, "tests/diagnostics/diag_stub.c"] + args + ["-DAPP_DEBUG_TELEMETRY=0", "-Wall", "-Wextra", "-Werror",
               "tests/ui/ui_harness.c", "application/ui_sydney_font.c", "application/app_clock.c", "application/app_weather.c",
               str(library), "-lm", "-o", str(quiet_exe)], cwd=ROOT, check=True)
subprocess.run([str(quiet_exe)], cwd=ROOT, check=True)
from PIL import Image, ImageDraw, ImageFont

try:
    font = ImageFont.truetype("arial.ttf", 15)
    small = ImageFont.truetype("arial.ttf", 13)
except OSError:
    font = small = ImageFont.load_default()
def make_preview(names, titles, columns, filename):
    rows = (len(names) + columns - 1) // columns
    montage = Image.new("RGB", (30 + columns * 266, 66 + rows * 340), "#041426")
    draw = ImageDraw.Draw(montage)
    draw.text((28, 18), "ANALOG WATCH / 240 x 280 / ACTUAL LVGL RENDER", fill="#f4faff", font=font)
    for index, (name, title) in enumerate(zip(names, titles)):
        frame = Image.open(OUT / (name + ".ppm"))
        frame.save(OUT / (name + ".png"))
        x = 28 + (index % columns) * 266
        y = 61 + (index // columns) * 340
        draw.rounded_rectangle((x - 4, y - 4, x + 243, y + 283), radius=18, outline="#173b54", width=2)
        montage.paste(frame, (x, y))
        draw.text((x, y + 294), title, font=small, fill="#7eabc1")
    draw.text((28, montage.height - 20), "Preview uses simulated sensor data.", fill="#7eabc1", font=small)
    montage.save(OUT / filename)


make_preview(["watch", "settings", "settings_scrolled"], ["WATCH / HOME", "APPS / TOP", "APPS / SCROLLED"],
             3, "watch-ui-preview.png")
make_preview(["heart", "motion", "environment", "aht21", "phone_weather", "bluetooth", "display"],
             ["HEART RATE", "MOTION", "BAROMETER", "CLIMATE", "PHONE WEATHER", "BLUETOOTH", "DISPLAY"],
             4, "watch-ui-details.png")
make_preview([f"weather_icon_{index}" for index in range(9)],
             ["CLEAR", "MAINLY CLEAR", "PARTLY CLOUDY", "FOG", "DRIZZLE", "RAIN", "SNOW", "THUNDERSTORM", "UNKNOWN"],
             3, "weather-icons-preview.png")
for name in ["watch", "motion_magnetic", "motion_calibrating", "motion_calibration_status", "motion_restored", "motion_save_failed",
             "bluetooth", "bluetooth_empty", "bluetooth_tx", "bluetooth_time",
             "watch_synced", "watch_midnight", "watch_unsynced", "watch_seconds",
             "heart", "heart_measuring", "heart_no_signal", "heart_unavailable",
             "aht21", "aht21_measuring", "aht21_crc_error", "aht21_unavailable", "battery_waiting", "battery_error", "phone_weather", "watch_weather"]:
    Image.open(OUT / (name + ".ppm")).save(OUT / (name + ".png"))
print("Preview:", OUT / "watch-ui-preview.png")
