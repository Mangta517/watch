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
subprocess.run([gcc] + args + ["-Wall", "-Wextra", "-Werror", "tests/ui/ui_harness.c", str(library), "-lm", "-o", str(exe)], cwd=ROOT, check=True)
subprocess.run([str(exe)], cwd=ROOT, check=True)
from PIL import Image, ImageDraw, ImageFont

names = ["watch", "sensors", "settings", "motion", "environment"]
titles = ["WATCH", "SENSORS", "SETTINGS", "MOTION", "ENVIRONMENT"]
montage = Image.new("RGB", (1360, 406), "#080c12")
draw = ImageDraw.Draw(montage)
try:
    font = ImageFont.truetype("arial.ttf", 15)
    small = ImageFont.truetype("arial.ttf", 13)
except OSError:
    font = small = ImageFont.load_default()
draw.text((28, 18), "DARK WATCH / 240 x 280 / ACTUAL LVGL RENDER", fill="#c5cfdd", font=font)
for index, (name, title) in enumerate(zip(names, titles)):
    frame = Image.open(OUT / (name + ".ppm"))
    frame.save(OUT / (name + ".png"))
    x = 28 + index * 266
    draw.rounded_rectangle((x - 4, 57, x + 243, 344), radius=18, outline="#253348", width=2)
    montage.paste(frame, (x, 61))
    draw.text((x, 355), title, font=small, fill="#7b8ba2")
draw.text((28, 386), "Preview uses simulated sensor data. Firmware reads the existing sensor snapshot services.", fill="#7b8ba2", font=small)
montage.save(OUT / "watch-ui-preview.png")
for name in ["motion_magnetic", "motion_calibrating", "motion_calibration_status", "motion_restored", "motion_save_failed"]:
    Image.open(OUT / (name + ".ppm")).save(OUT / (name + ".png"))
print("Preview:", OUT / "watch-ui-preview.png")
