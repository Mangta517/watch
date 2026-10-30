"""Exercise real weather parsing/storage with an EEPROM model; no hardware flashing."""
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
gcc = shutil.which("gcc")
if not gcc:
    raise SystemExit("Desktop GCC is required.")
with tempfile.TemporaryDirectory(prefix="watch-weather-store-") as out:
    exe = str(Path(out) / "weather-store.exe")
    subprocess.run([
        gcc, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
        "-Itests/startup/host", "-Iapplication", "-Iinterface",
        "tests/weather_store/weather_store_harness.c", "-o", exe,
    ], cwd=root, check=True)
    subprocess.run([exe], cwd=root, check=True)
