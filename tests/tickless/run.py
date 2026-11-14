"""Exercise the real Tickless adapter with simulated RTC/SysTick and wake interrupts."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(root / "tests"))
from source_extract import extract_functions
gcc = shutil.which("gcc")
if not gcc:
    raise SystemExit("Desktop GCC is required.")
with tempfile.TemporaryDirectory(prefix="watch-tickless-") as out:
    queries = "\n".join(extract_functions(root / "interface" / (module + ".c"), [module + "_is_stop_idle"])
                        for module in ["lcd", "kt6368a", "battery_adc"])
    (Path(out) / "stop_queries.inc").write_text(queries, encoding="utf-8")
    exe = str(Path(out) / "tickless.exe")
    for stop_enabled in (0, 1):
        subprocess.run([gcc, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                        "-DAPP_TICKLESS_ENABLE_STOP=" + str(stop_enabled),
                        "-Itests/tickless/host", "-Iapplication", "-Iinterface", "-I" + out,
                        "tests/tickless/tickless_harness.c", "application/app_tickless.c", "-o", exe], cwd=root, check=True)
        subprocess.run([exe], cwd=root, check=True)
