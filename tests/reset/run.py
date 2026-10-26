"""Exercise the real reset recorder with retained backup registers and a bounded SRAM mapping."""
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
gcc = shutil.which("gcc")
if not gcc:
    raise SystemExit("Desktop GCC is required.")

with tempfile.TemporaryDirectory(prefix="watch-reset-") as output:
    exe = str(Path(output) / "reset.exe")
    subprocess.run([
        gcc, "-std=c99", "-D_DEFAULT_SOURCE", "-O2", "-Wall", "-Wextra", "-Werror",
        "-Itests/reset/host", "-Iapplication", "-IMiddlewares/LVGL/lvgl",
        "tests/reset/reset_harness.c", "application/app_reset.c", "-o", exe,
    ], cwd=root, check=True)
    subprocess.run([exe], cwd=root, check=True)
