"""检查OTA清单权限：可重复执行、保留其他权限、位置权限覆盖全部SDK。"""
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import build


FINE = "android.permission.ACCESS_FINE_LOCATION"
SCAN = "android.permission.BLUETOOTH_SCAN"


def tag_for(manifest, name):
    pattern = re.compile(r'<uses-permission(?=\s)[^>]*android:name="' + re.escape(name) + r'"[^>]*?/>')
    match = pattern.search(manifest)
    assert match is not None, name + " missing"
    return match.group(0)


def check_cached_manifest():
    """旧缓存清单里的maxSdkVersion=30必须被清掉，且不能动其他权限。"""
    manifest = ('<manifest>\n'
                '    <uses-permission android:name="android.permission.CAMERA"/>\n'
                '    <uses-permission android:name="' + FINE + '" android:maxSdkVersion="30"/>\n'
                '    <application/>\n'
                '</manifest>')
    first = build.ensure_ble_permissions(manifest)
    fine = tag_for(first, FINE)
    assert 'android:name="' + FINE + '"' in fine, fine
    assert "maxSdkVersion" not in fine, fine
    assert 'android:usesPermissionFlags="neverForLocation"' in tag_for(first, SCAN)
    assert 'android:name="android.permission.BLUETOOTH_CONNECT"' in first
    assert '<uses-permission android:name="android.permission.CAMERA"/>' in first
    assert first.count(FINE) == 1
    second = build.ensure_ble_permissions(first)
    assert first == second, "permissions are not idempotent"


def check_missing_manifest():
    """基础清单缺少权限时必须补齐，位置权限不带任何SDK上限。"""
    manifest = '<manifest>\n    <application/>\n</manifest>'
    first = build.ensure_ble_permissions(manifest)
    assert "maxSdkVersion" not in tag_for(first, FINE)
    assert "neverForLocation" in tag_for(first, SCAN)
    assert first.index(FINE) < first.index("<application")
    second = build.ensure_ble_permissions(first)
    assert first == second, "added permissions are not idempotent"


if __name__ == "__main__":
    check_cached_manifest()
    check_missing_manifest()
    print("PASS: manifest permissions are idempotent, keep other tags, and cover FINE location on all SDK levels")
