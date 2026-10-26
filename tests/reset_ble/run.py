"""Check explicit reset queries and asynchronous UART ownership without flashing."""
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
gcc = shutil.which("gcc")
if not gcc:
    raise SystemExit("Desktop GCC is required.")
with tempfile.TemporaryDirectory(prefix="watch-reset-ble-") as output:
    exe = str(Path(output) / "reset-ble.exe")
    subprocess.run([
        gcc, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
        "-Itests/reset_ble/host", "-Iapplication", "-Iinterface",
        "tests/reset_ble/reset_ble_harness.c", "application/app_reset_ble.c", "-o", exe,
    ], cwd=root, check=True)
    subprocess.run([exe], cwd=root, check=True)
