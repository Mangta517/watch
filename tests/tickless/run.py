"""Exercise the real Tickless adapter with simulated RTC/SysTick and wake interrupts."""
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
gcc = shutil.which("gcc")
if not gcc:
    raise SystemExit("Desktop GCC is required.")
with tempfile.TemporaryDirectory(prefix="watch-tickless-") as out:
    exe = str(Path(out) / "tickless.exe")
    subprocess.run([gcc, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-Itests/tickless/host", "-Iapplication", "-Iinterface",
                    "tests/tickless/tickless_harness.c", "application/app_tickless.c", "-o", exe], cwd=root, check=True)
    subprocess.run([exe], cwd=root, check=True)
