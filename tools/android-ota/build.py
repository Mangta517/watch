"""Add the OTA page to the user's existing APK, preserving the original file."""
import argparse
from pathlib import Path
import re
import subprocess
import zipfile


# 原BLE DUAL(3)在Android缓存后仍返回3；SPP升级结束必须回到BLE(2)，不能误选经典蓝牙(1)。
TRANSPORT_METHOD_OLD = """.method public static transport_type(I)I
    .locals 2

    .line 12
    const/4 v0, 0x2

    if-ne p0, v0, :cond_0

    .line 14
    return v0

    .line 16
    :cond_0
    const/4 v0, 0x1

    if-eq p0, v0, :cond_2

    const/4 v1, 0x3

    if-ne p0, v1, :cond_1

    goto :goto_0

    .line 20
    :cond_1
    const/4 p0, 0x0

    return p0

    .line 18
    :cond_2
    :goto_0
    return v0
.end method"""

TRANSPORT_METHOD_NEW = """.method public static transport_type(I)I
    .locals 2

    .line 12
    const/4 v0, 0x2

    if-ne p0, v0, :cond_0

    .line 14
    return v0

    .line 16
    :cond_0
    const/4 v1, 0x3

    if-ne p0, v1, :cond_1

    .line 18
    return v0

    .line 20
    :cond_1
    const/4 v0, 0x1

    if-ne p0, v0, :cond_2

    .line 22
    return v0

    .line 24
    :cond_2
    const/4 v0, 0x0

    return v0
.end method"""


def run(command, cwd):
    subprocess.run([str(item) for item in command], cwd=cwd, check=True)


def patch_ble_mode(decoded):
    """把DUAL(3)路由改回BLE(2)：0->0、1->1、2->2、3->2；重复执行不再改动。"""
    path = decoded / "smali_classes2/dev/mangta/wave/BleMode.smali"
    if not path.is_file():
        raise RuntimeError("The decoded APK has no BleMode.smali for DUAL routing")
    text = path.read_bytes().decode("utf-8")
    if "\r\n" in text:
        newline = "\r\n"
    else:
        newline = "\n"
    normalized = text.replace("\r\n", "\n")
    old_count = normalized.count(TRANSPORT_METHOD_OLD)
    new_count = normalized.count(TRANSPORT_METHOD_NEW)
    if old_count == 0 and new_count == 1:
        return False
    if old_count != 1 or new_count != 0:
        raise RuntimeError("BleMode transport_type must match the expected old method exactly once")
    patched = normalized.replace(TRANSPORT_METHOD_OLD, TRANSPORT_METHOD_NEW)
    path.write_bytes(patched.replace("\n", newline).encode("utf-8"))
    return True


def ensure_permission(manifest, name, attributes, remove=()):
    """Add or update one uses-permission tag without touching other permissions."""
    pattern = re.compile(r'<uses-permission(?=\s)[^>]*android:name="' + re.escape(name) + r'"[^>]*?/>')
    match = pattern.search(manifest)
    if match is None:
        tag = '<uses-permission android:name="' + name + '"'
        for key, value in attributes:
            tag += ' ' + key + '="' + value + '"'
        tag += '/>'
        return manifest.replace("    <application", '    ' + tag + '\n    <application', 1)
    tag = match.group(0)
    for key in remove:
        tag = re.sub(r'\s+' + re.escape(key) + r'="[^"]*"', '', tag)
    for key, value in attributes:
        attribute = re.compile(r'\b' + re.escape(key) + r'="[^"]*"')
        if attribute.search(tag):
            tag = attribute.sub(key + '="' + value + '"', tag, count=1)
        else:
            tag = tag[:-2].rstrip() + ' ' + key + '="' + value + '"/>'
    return manifest[:match.start()] + tag + manifest[match.end():]


def ensure_ble_permissions(manifest):
    """Android 12+扫描/连接权限，并保留全SDK范围的位置权限。"""
    manifest = ensure_permission(manifest, "android.permission.BLUETOOTH_CONNECT", [])
    manifest = ensure_permission(manifest, "android.permission.BLUETOOTH_SCAN",
                                 [("android:usesPermissionFlags", "neverForLocation")])
    # 旧版WatchSync天气在Android 12+仍读位置；maxSdkVersion=30会静默拒绝，必须连同缓存清单一起清除。
    manifest = ensure_permission(manifest, "android.permission.ACCESS_FINE_LOCATION", [],
                                 ["android:maxSdkVersion"])
    return manifest


def patch_weather_pressure(decoded):
    """沿用原定位、网络和会话校验，追加pressure_msl，并在连接期间每15分钟刷新。"""
    folder = decoded / "smali_classes2/dev/mangta/wave"
    humidity = '    const-string v7, "relative_humidity_2m"'
    pressure = ('    const-string v7, "pressure_msl"\n\n'
                '    invoke-virtual {v0, v7}, Lorg/json/JSONObject;->optDouble(Ljava/lang/String;)D\n\n'
                '    move-result-wide v8\n\n' + humidity)
    frame = ('    invoke-static {v4, v2, v3, v0}, Ldev/mangta/wave/WatchSyncProtocol;->weather_frame(IDI)Ljava/lang/String;\n\n'
             '    move-result-object v0')
    appended_frame = (frame + '\n\n'
                      '    invoke-static {v0, v8, v9}, Ldev/mangta/wave/WeatherPressure;->append_reference(Ljava/lang/String;D)Ljava/lang/String;\n\n'
                      '    move-result-object v0')
    session = ('    invoke-virtual {v2, v0, v1}, Ldev/mangta/wave/WatchSyncSession;->update(Ljava/lang/Object;Z)I\n\n'
               '    move-result v2')
    refresh = (session + '\n\n'
               '    invoke-static {}, Landroid/os/SystemClock;->elapsedRealtime()J\n\n'
               '    move-result-wide v3\n\n'
               '    invoke-static {v0, v1, v2, v3, v4}, Ldev/mangta/wave/WeatherPressure;->refresh_token(Ljava/lang/Object;ZIJ)I\n\n'
               '    move-result v2')
    patches = [
        (folder / 'WatchSync$8.smali', [
            ('&current=temperature_2m,relative_humidity_2m,weather_code"',
             '&current=temperature_2m,relative_humidity_2m,weather_code,pressure_msl"'),
            (humidity, pressure), (frame, appended_frame)]),
        (folder / 'WatchSync$3.smali', [
            ('.method public run()V\n    .locals 4', '.method public run()V\n    .locals 6'),
            (session, refresh)]),
    ]
    for path, replacements in patches:
        original = path.read_text(encoding='utf-8')
        text = original
        for old, new in replacements:
            if new in text:
                continue
            if text.count(old) != 1:
                raise RuntimeError('Weather sync patch does not match ' + str(path))
            text = text.replace(old, new, 1)
        if text != original:
            path.write_text(text, encoding='utf-8')


def replace_ota_dex(apk, dex):
    """替换OTA字节码，兼容基础APK已经包含classes3.dex的重建。"""
    repacked = apk.with_name("repacked.apk")
    with zipfile.ZipFile(apk) as original, zipfile.ZipFile(repacked, "w", zipfile.ZIP_DEFLATED) as archive:
        for item in original.infolist():
            if item.filename != "classes3.dex":
                archive.writestr(item, original.read(item))
        archive.write(dex, "classes3.dex")
    repacked.replace(apk)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-apk", type=Path, required=True)
    parser.add_argument("--work-dir", type=Path, required=True)
    parser.add_argument("--apktool", type=Path, required=True)
    parser.add_argument("--jdk", type=Path, required=True)
    parser.add_argument("--android-tools", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    source = Path(__file__).resolve().parent / "src"
    work = args.work_dir.resolve()
    work.mkdir(parents=True, exist_ok=True)
    java = args.jdk.resolve() / "bin/java.exe"
    javac = args.jdk.resolve() / "bin/javac.exe"
    keytool = args.jdk.resolve() / "bin/keytool.exe"
    sdk = args.android_tools.resolve() / "android-15"
    android = args.android_tools.resolve() / "android-35/android.jar"
    decoded = work / "decoded"
    if not decoded.exists():
        run([java, "-jar", args.apktool.resolve(), "d", args.base_apk.resolve(), "-o", decoded], work)

    # 原签名密钥不在所给目录内；使用独立包名，旧App及其数据可继续保留。
    manifest_path = decoded / "AndroidManifest.xml"
    manifest = manifest_path.read_text(encoding="utf-8")
    manifest = manifest.replace('package="xnj.lazydog.btcontroller.wave"', 'package="xnj.lazydog.btcontroller.wave.ota"')
    manifest = manifest.replace('android:label="@string/app_name"', 'android:label="蓝牙调试器·波形 OTA"')
    manifest = manifest.replace("xnj.lazydog.btcontroller.wave.FileProvider", "xnj.lazydog.btcontroller.wave.ota.FileProvider")
    if 'android:name="dev.mangta.wave.OtaActivity"' not in manifest:
        manifest = manifest.replace("</application>", '<activity android:name="dev.mangta.wave.OtaActivity" '
            'android:exported="false" android:screenOrientation="portrait" android:theme="@android:style/Theme.Material.Light"/>\n'
            '    </application>')
    manifest = ensure_ble_permissions(manifest)
    if 'android:name="dev.mangta.wave.OtaLaunchActivity"' not in manifest:
        # 旧Splash在权限完成前创建蓝牙线程，新入口取得授权后再进入旧启动流程。
        launcher = ('            <intent-filter>\n'
                    '                <action android:name="android.intent.action.MAIN"/>\n'
                    '                <category android:name="android.intent.category.LAUNCHER"/>\n'
                    '            </intent-filter>')
        if manifest.count(launcher) != 1:
            raise RuntimeError("The source launcher does not match the expected APK")
        manifest = manifest.replace(launcher, "")
        manifest = manifest.replace("</application>", '<activity android:name="dev.mangta.wave.OtaLaunchActivity" '
            'android:exported="true" android:theme="@android:style/Theme.Material.Light">\n'
            + launcher + '\n        </activity>\n    </application>')
    manifest_path.write_text(manifest, encoding="utf-8")
    yaml_path = decoded / "apktool.yml"
    yaml = yaml_path.read_text(encoding="utf-8")
    yaml = re.sub(r"(?m)^  versionCode:.*$", "  versionCode: 25", yaml)
    yaml = re.sub(r"(?m)^  versionName:.*$", "  versionName: 1.95-wave.20-baro-sync", yaml)
    yaml_path.write_text(yaml, encoding="utf-8")
    patch_ble_mode(decoded)
    patch_weather_pressure(decoded)
    hook = decoded / "smali_classes2/dev/mangta/wave/WaveBridge.smali"
    text = hook.read_text(encoding="utf-8")
    anchor = "    invoke-static {p0, v0}, Ldev/mangta/wave/WatchSync;->attach(Landroid/app/Activity;Landroid/widget/LinearLayout;)V"
    if "Ldev/mangta/wave/OtaBridge;->install" not in text:
        if text.count(anchor) != 1:
            raise RuntimeError("The source APK does not match the existing WaveBridge hook")
        text = text.replace(anchor, anchor + "\n\n    invoke-static {p0, v0}, Ldev/mangta/wave/OtaBridge;->install(Landroid/app/Activity;Landroid/widget/LinearLayout;)V")
        hook.write_text(text, encoding="utf-8")
    for folder in ["smali", "smali_classes2"]:
        for path in (decoded / folder).rglob("*.smali"):
            text = path.read_text(encoding="utf-8")
            if "xnj.lazydog.btcontroller.wave.FileProvider" in text:
                path.write_text(text.replace("xnj.lazydog.btcontroller.wave.FileProvider",
                    "xnj.lazydog.btcontroller.wave.ota.FileProvider"), encoding="utf-8")

    classes = work / "classes"
    dex = work / "dex"
    classes.mkdir(exist_ok=True)
    dex.mkdir(exist_ok=True)
    run([javac, "-encoding", "UTF-8", "--release", "8", "-classpath", android, "-d", classes, *sorted(source.rglob("*.java"))], work)
    class_jar = work / "ota.jar"
    with zipfile.ZipFile(class_jar, "w", zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(classes.rglob("*.class")):
            archive.write(path, path.relative_to(classes).as_posix())
    run([java, "-cp", sdk / "lib/d8.jar", "com.android.tools.r8.D8", "--lib", android,
         "--min-api", "21", "--output", dex, class_jar], work)
    unsigned = work / "unsigned.apk"
    run([java, "-jar", args.apktool.resolve(), "b", decoded, "-o", unsigned], work)
    replace_ota_dex(unsigned, dex / "classes.dex")
    aligned = work / "aligned.apk"
    run([sdk / "zipalign.exe", "-f", "-p", "4", unsigned, aligned], work)
    keystore = work / "ota-signing.p12"
    if not keystore.exists():
        run([keytool, "-genkeypair", "-keystore", keystore, "-storetype", "PKCS12", "-alias", "ota",
             "-keyalg", "RSA", "-keysize", "2048", "-validity", "10000", "-storepass", "android",
             "-keypass", "android", "-dname", "CN=F411 Watch OTA,O=Mangta,C=CN"], work)
    output = args.output.resolve()
    if output == args.base_apk.resolve():
        raise ValueError("Output must not overwrite the original APK")
    output.parent.mkdir(parents=True, exist_ok=True)
    run([java, "-jar", sdk / "lib/apksigner.jar", "sign", "--ks", keystore, "--ks-key-alias", "ota",
         "--ks-pass", "pass:android", "--out", output, aligned], work)
    run([java, "-jar", sdk / "lib/apksigner.jar", "verify", "--verbose", output], work)
    run([sdk / "zipalign.exe", "-c", "-p", "4", output], work)
    print("APK:", output)


if __name__ == "__main__":
    main()
