# 旧蓝牙波形App的手机OTA扩充

使用安卓手机选BIN和SPP发送，保留旧App的波形、诊断、导出、时间与天气功能。源APK为`E:\Downloads\base.apk`，输出为`E:\Downloads\Bluetooth-Wave-OTA.apk`，1.95-wave.16-recovery/code21。

仓库包含扩充模块与脚本（`src/`、`tests/`、`build.py`等），不含原APK、`base.apk`、原Java工程和原签名密钥；原Java工程和原签名均未提供，也不是本重建路径的输入。构建需本地`base.apk`、Android SDK/JDK、Apktool，以及扩充版签名（首次由脚本生成，覆盖安装后续沿用同一签名）。构建时通过Apktool恢复旧工程，再把本目录Java模块编译到`classes3.dex`并接入原主界面。新包名为`xnj.lazydog.btcontroller.wave.ota`，新签名与旧版不同，安装后与旧版并存；旧App和`base.apk`保留。原设置不会自动迁移，需要在扩充版重新选择FFF0/FFF1。

完整部署和操作见[手表OTA说明](../../docs/bootloader.md)。首次必须通过SWD部署Bootloader；手机不能给链接到0x08000000的普通APP凭空增加Bootloader。

当前启用三个独立程序：主引导`0x08000000`、保底`0x08008000`、APP`0x08010000`，APP最大320KiB。首次需同时部署主引导和保底；可用`F411-OTA-Factory.hex`一次下载。手机发送`OTA,INFO`查询，只有收到`OTA,LAYOUT,08010000,00050000`后才发送start；旧布局和旧APP向量会在擦除前被拒绝。上一版0x08008000/352KiB布局已停用。

## 构建

复用本机Android SDK/JDK，无Gradle或新增运行库。Apktool来自[官方发布](https://github.com/iBotPeaches/Apktool/releases/tag/v2.12.1)。解包、编译、签名及模拟器产物留在E盘工作目录，原APK不会覆盖。

```powershell
python tools/android-ota/build.py --base-apk E:\Downloads\base.apk --work-dir E:\Downloads\bluetooth-wave-ota-work --apktool E:\Downloads\bluetooth-wave-ota-work\tools\apktool.jar --jdk D:\STM32CubeMX\jre --android-tools E:\Codex\2026-10-25\LocalLedger\work\android-tools --output E:\Downloads\Bluetooth-Wave-OTA.apk
```

保留工作目录中的`ota-signing.p12`，后续扩充版更新需继续使用此签名。它是本地开发签名，密码为`android`，不用于商店发布。原App的签名无法从APK恢复。

协议回归不需要手机：

```powershell
& D:\STM32CubeMX\jre\bin\javac.exe -encoding UTF-8 --release 8 -d E:\Downloads\bluetooth-wave-ota-work\protocol-tests tools/android-ota/src/dev/mangta/wave/OtaProtocol.java tools/android-ota/tests/OtaProtocolTest.java
& D:\STM32CubeMX\jre\bin\java.exe -cp E:\Downloads\bluetooth-wave-ota-work\protocol-tests dev.mangta.wave.OtaProtocolTest
python -X utf8 tests/ota/run.py
```

文件选择使用Android[系统文档选择器](https://developer.android.com/training/data-storage/shared/documents-files)，SPP权限按[Android蓝牙权限说明](https://developer.android.com/develop/connectivity/bluetooth/bt-permissions)处理。升级页面独占自己的SPP socket，原BLE收发不读取该连接。新页面不扫描经典蓝牙，配对在系统设置完成。

页面前台时才维持升级；进入后台会停止本次连接。只在设备返回严格匹配的`ok,len:N`后报成功。CRC不提供来源认证；沿用原Bootloader的单APP升级及重新完整传输恢复方式。
