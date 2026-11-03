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
    source = (root / "application/app_reset.c").read_text(encoding="utf-8")
    checksum_bit = "        for (bit = 0; bit < 32; ++bit)\n        {\n"
    if source.count(checksum_bit) != 1:
        raise SystemExit("Expected one reset CRC bit loop for interrupt injection.")
    instrumented = Path(output) / "app_reset.c"
    instrumented.write_text(source.replace(checksum_bit, checksum_bit + "            host_reset_checksum_bit();\n"), encoding="utf-8")
    subprocess.run([
        gcc, "-std=c99", "-D_DEFAULT_SOURCE", "-O2", "-Wall", "-Wextra", "-Werror",
        "-Itests/reset/host", "-Iapplication", "-IMiddlewares/LVGL/lvgl",
        "tests/reset/reset_harness.c", str(instrumented), "-o", exe,
    ], cwd=root, check=True)
    subprocess.run([exe], cwd=root, check=True)
