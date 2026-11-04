# 蓝牙波形App的手机SPP OTA扩充

在旧波形App上增加“一键升级”：日常仍用BLE连接手表，OTA时APK自动经BLE请求进入保底，再关闭BLE、改用同一模块的SPP地址发送APP BIN。保留原波形、诊断、导出、时间与天气功能。

本版产物为`work/spp-oneclick-20261104/delivery/Bluetooth-Wave-SPP-OTA-OneClick.apk`，版本`1.95-wave.19-spp-oneclick` / code24，包名`xnj.lazydog.btcontroller.wave.ota`，签名与上一版`1.95-wave.18-oneclick` / code23相同，可直接覆盖安装。原波形App的签名不同，两者仍并存。

仓库包含扩充模块与脚本（`src/`、`tests/`、`build.py`等），不含原始波形App、其Java工程和原签名密钥。构建通过Apktool复用已解码工程，把本目录Java模块编译到`classes3.dex`并接入原主界面。

## 操作步骤

1. 首次部署用SWD写入主引导`0x08000000`、保底`0x08008000`和APP`0x08010000`；已经刷过仅BLE保底的设备，只需用SWD把本版`F411-OTA-Recovery.bin`写到`0x08008000`一次。主引导本版未要求变化；保底仍兼容SPP OTA的旧设备不强制SWD。
2. 在原主界面连接手表BLE，打开“手表固件升级 · OTA”，选择链接到`0x08010000`的APP BIN（不超过320KiB）。
3. 点一次“开始升级”，等待自动完成：BLE探测或`OTA,START` → 关闭BLE → 按SPP地址连接 → 严格查询布局 → 单次发送BIN → `ok,len:N` → 关闭SPP → 等待APP启动 → 恢复原BLE。
4. 首次安全RFCOMM连接可能弹出Android系统配对确认或PIN，允许一次即可；之后不需要每次手动配对、进入升级模式、重新扫描或改设置。
5. 停止、切后台、断连或失败会结束本次传输，不续传、不自动整轮重试；重新点“开始升级”会从头发送完整BIN。收到`ok,len:N`后恢复原BLE失败只影响连接，固件仍算成功。

手机只发送APP BIN，不能写主引导或保底，也不能替代SWD恢复。完整部署、迁移和协议细节见[手表OTA说明](../../docs/bootloader.md)。

## 构建

复用本机Android SDK/JDK，无Gradle或新增运行库。Apktool来自[官方发布](https://github.com/iBotPeaches/Apktool/releases/tag/v2.12.1)。

本版沿用本机已准备的本地缓存解码构建：`work/spp-oneclick-20261104/android-build/decoded/`目录已存在，不要重新解包；签名密钥`ota-signing.p12`在它的同级`work/spp-oneclick-20261104/android-build/`下，不在`decoded/`里面。`--base-apk`只在`decoded/`不存在时供Apktool解包，并用于防止输出覆盖输入；不要删除`decoded/`后用旧APK重新解包，否则会把旧`classes3.dex`带回。`build.py`已把版本号写成code24 / `1.95-wave.19-spp-oneclick`，并在打包前把BleMode的`DUAL`路由固定到BLE（纯经典设备仍按经典处理）。

```powershell
python tools/android-ota/build.py --base-apk work/ota-oneclick-20261104/delivery/Bluetooth-Wave-BLE-OTA-OneClick.apk --work-dir work/spp-oneclick-20261104/android-build --apktool E:/Downloads/bluetooth-wave-ota-work/tools/apktool.jar --jdk D:/STM32CubeMX/jre --android-tools E:/Codex/2026-10-25/LocalLedger/work/android-tools --output work/spp-oneclick-20261104/delivery/Bluetooth-Wave-SPP-OTA-OneClick.apk
```

脚本复用`android-build/ota-signing.p12`（PKCS12，别名`ota`，密码`android`）签名，该密钥与code23相同，是本地开发签名，不用于商店发布。构建产物和中间文件留在`work/spp-oneclick-20261104/android-build`，原APK不会被覆盖。

## 主机测试

协议、适配器和地址换算回归不需要手机，使用本机JDK。普通测试输出到`work/spp-oneclick-20261104/tests`，编译六个核心源文件（新增`Kt6368Address.java`、`OtaSppTransport.java`）、`OtaBridge.java`、`tests/stubs/**`（其中的`dev/mangta/wave/OtaActivity.java`只是编译`OtaBridge`用的占位）和六个普通测试。`OtaUpgrade.Actions`新增`createRecovery()`，`OtaUpgradeTest`已相应更新，覆盖BLE控制后关闭、再走SPP正文。

```powershell
$jdk = 'D:\STM32CubeMX\jre\bin'
$src = 'tools/android-ota/src/dev/mangta/wave'
$tests = 'tools/android-ota/tests'
$out = 'work/spp-oneclick-20261104/tests'
New-Item -ItemType Directory -Force -Path $out | Out-Null
$core = @(
    "$src/OtaProtocol.java",
    "$src/OtaUpgrade.java",
    "$src/OtaDiagnosticFilter.java",
    "$src/OtaBleTransport.java",
    "$src/OtaSppTransport.java",
    "$src/Kt6368Address.java"
)
$stubs = @(
    Get-ChildItem -LiteralPath "$tests/stubs" -Recurse -Filter *.java |
    Select-Object -ExpandProperty FullName
)
$sources = $core + @("$src/OtaBridge.java") + $stubs + @(
    "$tests/OtaProtocolTest.java",
    "$tests/OtaUpgradeTest.java",
    "$tests/OtaBleTransportTest.java",
    "$tests/OtaBridgeTest.java",
    "$tests/OtaSppTransportTest.java",
    "$tests/Kt6368AddressTest.java"
)
& "$jdk\javac.exe" -encoding UTF-8 --release 8 -d $out @sources
if ($LASTEXITCODE -ne 0)
{
    exit $LASTEXITCODE
}
foreach ($test in 'OtaProtocolTest', 'OtaUpgradeTest', 'OtaBleTransportTest', 'OtaBridgeTest', 'OtaSppTransportTest', 'Kt6368AddressTest')
{
    & "$jdk\java.exe" -cp $out "dev.mangta.wave.$test"
    if ($LASTEXITCODE -ne 0)
    {
        exit $LASTEXITCODE
    }
}
```

诊断过滤测试必须单独编译到`work/spp-oneclick-20261104/tests-diagnostic`：同样的六个核心源文件、`tests/stubs/**`中的Android桩（排除`dev/mangta/wave/OtaActivity.java`占位，更不含实际页面）、额外的`tests/diagnostic-stubs/**`，再加`OtaDiagnosticFilterTest.java`和`OtaBleDiagnosticTest.java`。诊断替身的默认响应是`null`，混入普通测试会改变过滤结果，因此两类测试不能共用输出目录或classpath。

```powershell
$jdk = 'D:\STM32CubeMX\jre\bin'
$src = 'tools/android-ota/src/dev/mangta/wave'
$tests = 'tools/android-ota/tests'
$out = 'work/spp-oneclick-20261104/tests-diagnostic'
New-Item -ItemType Directory -Force -Path $out | Out-Null
$core = @(
    "$src/OtaProtocol.java",
    "$src/OtaUpgrade.java",
    "$src/OtaDiagnosticFilter.java",
    "$src/OtaBleTransport.java",
    "$src/OtaSppTransport.java",
    "$src/Kt6368Address.java"
)
$stubs = @(
    Get-ChildItem -LiteralPath "$tests/stubs" -Recurse -Filter *.java |
    Where-Object Name -ne 'OtaActivity.java' |
    Select-Object -ExpandProperty FullName
)
$diagnosticStubs = @(
    Get-ChildItem -LiteralPath "$tests/diagnostic-stubs" -Recurse -Filter *.java |
    Select-Object -ExpandProperty FullName
)
$sources = $core + $stubs + $diagnosticStubs + @(
    "$tests/OtaDiagnosticFilterTest.java",
    "$tests/OtaBleDiagnosticTest.java"
)
& "$jdk\javac.exe" -encoding UTF-8 --release 8 -d $out @sources
if ($LASTEXITCODE -ne 0)
{
    exit $LASTEXITCODE
}
foreach ($test in 'OtaDiagnosticFilterTest', 'OtaBleDiagnosticTest')
{
    & "$jdk\java.exe" -cp $out "dev.mangta.wave.$test"
    if ($LASTEXITCODE -ne 0)
    {
        exit $LASTEXITCODE
    }
}
```

构建脚本与清单检查：

```powershell
python -X utf8 tools/android-ota/tests/build_routing_test.py
if ($LASTEXITCODE -ne 0)
{
    exit $LASTEXITCODE
}
python -X utf8 tools/android-ota/tests/manifest_test.py
if ($LASTEXITCODE -ne 0)
{
    exit $LASTEXITCODE
}
```

`build_routing_test.py`检查构建脚本的BleMode路由补丁：只把双模设备`DUAL`映射到BLE、保留纯经典设备和BLE原映射，并且重复执行不再改动；`manifest_test.py`检查BLE权限可重复补齐、保留其他权限且位置权限覆盖全部SDK。最终编译和测试结果以实际运行记录为准，本README不声明已通过。

## 说明

- 地址换算依据KT6368A手册V2.0第9-10页：SPP地址是BLE地址最高字节加1，AT串字节序与Android显示相反。Android BLE `2C:A2:C8:B1:0C:22`对应SPP `2D:A2:C8:B1:0C:22`；APK自动换算首字节，其余五字节不变，用户不需要另选SPP名称或地址。BLE与SPP不是同一个地址。
- 原主界面的连接路由把双模设备固定按BLE处理，纯经典设备仍按经典处理；OTA页显式建立RFCOMM，绕过这条日常路由。
- 首次安全RFCOMM连接可能触发系统配对确认/PIN，允许一次即可，不是每次升级都要重新配对。
- 扫描使用`BluetoothLeScanner`：Android 12+需要“附近设备”的扫描和连接权限；Android 6–11还需要位置权限并打开系统位置服务。离开页面或点停止会停止扫描并关闭本次连接。
- 页面前台时才维持扫描和升级；进入后台会停止本次扫描/连接。只有收到固件确认的`ok,len:N`后才自动恢复原主连接；升级失败或在确认前停止/断连时不恢复，只提示重新连接。
- 上述六个普通测试与两个诊断测试是主机适配器/契约回归，不是真实无线、系统配对对话框或整机验证。文件选择使用Android[系统文档选择器](https://developer.android.com/training/data-storage/shared/documents-files)，BLE权限按[Android蓝牙权限说明](https://developer.android.com/develop/connectivity/bluetooth/bt-permissions)处理。CRC不提供来源认证；升级中断后按完整BIN重新发送，不支持偏移续传。
