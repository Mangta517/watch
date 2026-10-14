from pathlib import Path
import shutil
import subprocess
import tempfile
import sys

root = Path(__file__).resolve().parents[2]
gcc = shutil.which("gcc")
if not gcc:
    raise SystemExit("Desktop GCC is required")
fixture = Path(sys.argv[1]).resolve()
fixture.parent.mkdir(parents=True, exist_ok=True)
with tempfile.TemporaryDirectory(prefix="watch-diag-") as temporary:
    executable = str(Path(temporary) / "diagnostics.exe")
    for enabled in (1, 0):
        subprocess.run([gcc, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                        "-DAPP_DEBUG_TELEMETRY=" + str(enabled),
                        "-Itests/diagnostics/host", "-Iapplication", "-Iinterface",
                        "tests/diagnostics/diag_harness.c", "-o", executable], cwd=root, check=True)
        target = fixture
        if enabled == 0:
            target = Path(temporary) / "quiet.bin"
        subprocess.run([executable, str(target)], cwd=root, check=True)
