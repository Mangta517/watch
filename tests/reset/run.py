"""Exercise the real reset recorder with retained backup registers and a bounded SRAM mapping."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from source_extract import extract_functions

root = Path(__file__).resolve().parents[2]
gcc = shutil.which("gcc")
if not gcc:
    raise SystemExit("Desktop GCC is required.")

with tempfile.TemporaryDirectory(prefix="watch-reset-") as output:
    entries = "\n".join(extract_functions(root / module, names) for module, names in [
        ("Core/Src/main.c", ["Error_Handler"]),
        ("Core/Src/stm32f4xx_it.c", ["app_fault_stop", "NMI_Handler"]),
        ("application/app_task.c", ["vApplicationStackOverflowHook", "App_Task_Start"]),
    ])
    (Path(output) / "fatal_entries.inc").write_text(entries, encoding="utf-8")
    exe = str(Path(output) / "reset.exe")
    source = (root / "application/app_reset.c").read_text(encoding="utf-8")
    checksum_bit = "        for (bit = 0; bit < 32; ++bit)\n        {\n"
    if source.count(checksum_bit) != 1:
        raise SystemExit("Expected one reset CRC bit loop for interrupt injection.")
    instrumented = Path(output) / "app_reset.c"
    instrumented.write_text(source.replace(checksum_bit, checksum_bit + "            host_reset_checksum_bit();\n"), encoding="utf-8")
    subprocess.run([
        gcc, "-std=c99", "-D_DEFAULT_SOURCE", "-O2", "-Wall", "-Wextra", "-Werror",
        "-Itests/reset/host", "-Iapplication", "-IMiddlewares/LVGL/lvgl", "-I" + output,
        "tests/reset/reset_harness.c", str(instrumented), "-o", exe,
    ], cwd=root, check=True)
    subprocess.run([exe], cwd=root, check=True)
