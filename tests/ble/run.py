"""Compile the real KT6368A driver with simulated UART/GPIO; no hardware flashing."""
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
gcc = shutil.which("gcc")
if not gcc:
    raise SystemExit("Desktop GCC is required.")
with tempfile.TemporaryDirectory(prefix="watch-ble-") as out:
    exe = str(Path(out) / "ble.exe")
    subprocess.run([
        gcc, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
        "-Itests/ble/host", "-Iinterface", "tests/ble/ble_harness.c",
        "interface/kt6368a.c", "-o", exe,
    ], cwd=root, check=True)
    subprocess.run([exe], cwd=root, check=True)
