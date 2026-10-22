"""Exercise the real touch event bridge without flashing hardware."""
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
gcc = shutil.which("gcc")
if not gcc:
    raise SystemExit("Desktop GCC is required.")
with tempfile.TemporaryDirectory(prefix="watch-touch-") as out:
    exe = str(Path(out) / "touch.exe")
    subprocess.run([
        gcc, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
        "-include", "tests/ui/host/platform.h", "-Itests/ui/host",
        "-Iapplication", "-Iinterface", "-ICore/Inc",
        "tests/touch/touch_harness.c", "-o", exe,
    ], cwd=root, check=True)
    subprocess.run([exe], cwd=root, check=True)
