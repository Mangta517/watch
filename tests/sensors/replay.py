"""Replay exported raw samples with their board timestamps through the real heart service.

Usage: python tests/sensors/replay.py input.csv output.csv
Does not alter firmware settings or flash hardware; original CSV stays unchanged.
"""
from pathlib import Path
import csv
import shutil
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[2]
if len(sys.argv) != 3:
    raise SystemExit(__doc__)
source = Path(sys.argv[1]).resolve()
output = Path(sys.argv[2]).resolve()
if source == output:
    raise SystemExit("Input and output paths must be different.")
gcc = shutil.which("gcc")
if not gcc:
    raise SystemExit("Desktop GCC is required.")
with source.open(encoding="utf-8") as stream:
    reader = csv.DictReader(stream)
    required = ["sample_index", "received_elapsed_ms"] + [f"CH{i}" for i in range(1, 9)]
    if reader.fieldnames is None or reader.fieldnames[:10] != required:
        raise SystemExit("Expected F411_HEART_DIAG_V1 CSV column order.")
    records = list(reader)
if not records:
    raise SystemExit("Input CSV has no samples.")
if float(records[0]["CH7"]) != 0:
    raise SystemExit("Replay needs a complete capture starting at device sequence 0.")
with tempfile.TemporaryDirectory(prefix="watch-heart-replay-") as temporary:
    executable = str(Path(temporary) / "heart-replay.exe")
    subprocess.run([gcc, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-Itests/sensors/host", "-ICore/Inc", "-Iinterface", "-Iapplication",
                    "tests/sensors/heart_replay.c", "-lm", "-o", executable], cwd=root, check=True)
    subprocess.run([executable, str(source), str(output)], cwd=root, check=True)
with output.open(encoding="utf-8") as stream:
    rows = list(csv.DictReader(stream))
assert len(rows) == len(records)
for label in ["old", "new"]:
    valid = [row for row in rows if int(row[f"{label}_state"]) == 2]
    jumps = sum(int(left[f"{label}_state"]) == 2 and int(right[f"{label}_state"]) == 2
                and abs(float(right[f"{label}_bpm"]) - float(left[f"{label}_bpm"])) >= 20
                for left, right in zip(rows, rows[1:]))
    maximum = max(abs(float(row[f"{label}_filtered"])) for row in rows)
    print(f"{label}: valid={len(valid)}/{len(rows)}, adjacent >=20 BPM jumps={jumps}, max |filtered|={maximum:.2f}")
print("Saved comparison:", output)
