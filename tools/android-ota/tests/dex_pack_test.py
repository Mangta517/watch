"""验证从旧OTA APK重建时替换classes3.dex，其他ZIP条目保留。"""
from pathlib import Path
import sys
import tempfile
import zipfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import build

with tempfile.TemporaryDirectory(prefix="ota-dex-") as folder:
    root = Path(folder)
    dex = root / "classes.dex"
    dex.write_bytes(b"new OTA bytecode")
    for existing in (False, True):
        apk = root / "unsigned.apk"
        with zipfile.ZipFile(apk, "w") as archive:
            archive.writestr("classes.dex", b"original application")
            archive.writestr("classes2.dex", b"original weather sync")
            archive.writestr("assets/data.bin", b"asset bytes")
            if existing:
                archive.writestr("classes3.dex", b"old OTA bytecode")
        build.replace_ota_dex(apk, dex)
        build.replace_ota_dex(apk, dex)
        with zipfile.ZipFile(apk) as archive:
            names = archive.namelist()
            assert names.count("classes3.dex") == 1
            assert len(names) == len(set(names))
            assert archive.read("classes3.dex") == dex.read_bytes()
            assert archive.read("classes.dex") == b"original application"
            assert archive.read("classes2.dex") == b"original weather sync"
            assert archive.read("assets/data.bin") == b"asset bytes"
print("PASS: fresh and existing OTA APKs replace classes3.dex once, preserve other entries and support repeat packing.")
