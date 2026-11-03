"""Add the OTA page to the user's existing APK, preserving the original file."""
import argparse
from pathlib import Path
import re
import subprocess
import zipfile


def run(command, cwd):
    subprocess.run([str(item) for item in command], cwd=cwd, check=True)


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
    if "android.permission.BLUETOOTH_CONNECT" not in manifest:
        manifest = manifest.replace("    <application", '    <uses-permission android:name="android.permission.BLUETOOTH_CONNECT"/>\n    <application')
    if "android.permission.BLUETOOTH_SCAN" not in manifest:
        manifest = manifest.replace("    <application", '    <uses-permission android:name="android.permission.BLUETOOTH_SCAN"/>\n    <application')
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
    yaml = re.sub(r"(?m)^  versionCode:.*$", "  versionCode: 21", yaml)
    yaml = re.sub(r"(?m)^  versionName:.*$", "  versionName: 1.95-wave.16-recovery", yaml)
    yaml_path.write_text(yaml, encoding="utf-8")
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
    with zipfile.ZipFile(unsigned, "a", zipfile.ZIP_DEFLATED) as archive:
        archive.write(dex / "classes.dex", "classes3.dex")
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
