# STM32F411 智能手表

基于 STM32F411CEU6、FreeRTOS 和 LVGL 的手表工程，包含 RTC 表盘、蓝牙校时与天气同步、运动与环境传感器、心率采集（手动START/STOP与AUTO每10分钟定时）和电池显示。

本页为 2026-11-03 累计发布版本，当前分支 `codex/release-20261103`，汇总三程序 OTA（主引导、独立保底与升级 APP）、默认 ECO 低功耗、电量与复位日志及运行诊断等已完成改动。普通 APP 和升级 APP 共用业务源码，包含低功耗调度、计步与辅助抬腕、日常 BLE 通信、复位和电压日志，以及主引导／保底的 BLE OTA 升级。2026-11-04 起 OTA 传输由 SPP 改为 BLE（保底同步改造，只认 SPP 的旧保底须先用 SWD 更新），本页的当前协议与部署描述已按 BLE 更新，实板烧录与手机实测仍待完成。本次涉及的文件、具体改动、优化效果及验证结果见[更新记录](#更新记录)。后面的历史条目保留各原始提交的交付记录，部分提交来自其他职责分支；2026-11-02的蓝牙默认ECO、心率AUTO每10分钟定时、稳定熄屏400ms慢待机、AHT采样窗口收尾及触摸休眠成功后才等待通知，以及2026-11-03所有实际STOP均临时屏蔽MPU DATA_RDY/PB12 EXTI并原样恢复、保留PA4/RTC及pending为最新当前行为，实际电流与佩戴效果仍待实测，当前行为以本页首节及当前源码为准。

## 当前低功耗策略

| 项目 | 当前行为 |
| --- | --- |
| 主频与电压档 | APP 固定64MHz / Scale3，Flash 2等待周期；APB1=32MHz、APB2=64MHz、SPI1=32MHz，背光PWM仍约3.32kHz |
| CPU Sleep / STOP | 默认ECO熄屏关闭BLE、外设空闲时进入低功耗稳压器STOP（慢待机RTC最长400ms）；选择保连时BLE常开使用Sleep，其单次Sleep受SysTick 24位容量限制为最多262ms；启用Flash掉电及稳压器低电压模式，醒后恢复64MHz |
| GPIO与外设停钟 | 空闲PA0/PA2/PA6/PA7/PB10/PC13为模拟无上下拉；PA3和屏幕控制/心率LED推挽输出移除内部下拉；熄屏后停止TIM3并关闭TIM3/SPI1/DMA2时钟，ADC在Sleep期间停钟 |
| 手机性能诊断 | 现有App诊断页发送DIAG,1加换行开启、DIAG,0关闭；采集lvgl/sensor/touch/IDLE及传感器、内存和UART指标，默认静默，心率波形仍关闭 |
| 无触摸待机 | 连续 10 秒无操作后，与 PA4 手动熄屏共用处理：背光 0%、默认ECO关闭蓝牙供电/UART（选择保连时保留）、暂停绘图，LCD/触摸芯片休眠且仅在休眠成功后阻塞等待PA4，失败时首次立即、其后按500ms退避；PA4 或辅助抬腕亮屏，恢复原亮度和当前 RTC 时间 |
| 显示与触摸轮询 | 活跃约 5ms；熄屏无按键时等待 PA4/UART/sensor 通知或200/400ms期限，按键消抖/按住期间保持 20ms，触摸休眠后阻塞 |
| MPU6050 | 心率测量及独立运动模式 100Hz，姿态页与校准 50Hz；无姿态需求的亮屏页面与普通熄屏使用 MPU 自主 5Hz 硬件低功耗循环（5Hz为硬件CYCLE，非400ms ODR），关闭陀螺仪和温度，持续计步及辅助抬腕；启用AUTO心率窗口保留 100Hz 运动参考；稳定熄屏约10秒后CPU采样放宽至400ms，需实板复测计步/抬腕精度与时延 |
| 磁力计 | 心率与SPORT使用加速度参考时关闭磁力计，5Hz后台同样休眠；仅姿态页/校准按完整IMU档位恢复（活动/校准30Hz），不再随运动/心率场景开启 |
| 心率读取 | 测量期间sensor目标10ms，PPG读取下限30ms；MPU配置100Hz、校准50Hz；心率页独立`AUTO / 10 MIN`默认OFF、仅RAM，启用后熄屏每10分钟测最多10秒、有效BPM即停，与手动START/STOP及SPORT强制连续互不干扰；实际循环率受串行读取和抢占影响，以手机诊断为准 |
| 温湿度采样 | 所有亮屏页面每1秒采样，熄屏每10分钟采样；AHT测量窗口在采样及恢复完成后即收尾（保留恢复与读回超时），普通待机和SPORT遵守同一屏幕策略，切页不重置周期 |
| 零偏校准 | 保留 100 帧静止确认和 100 帧平均，50Hz 下约 4 秒；两个阶段超时分别为 6 秒、4 秒 |
| 电池估算 | 4.00V为100%、3.50V及以下为0%；2026-10-31依据同电池未充电记录重标：3.60V约24%、3.55V约14%。启动均值、滤波及本次开机百分比只降不升；该下限不触发关机 |

Tickless保留1ms逻辑节拍，单次最长400ms，使用RTC测量并补齐HAL / FreeRTOS时间；RTC未就绪时退回1ms Sleep。`application/app_tickless.h`中的`APP_TICKLESS_ENABLE_STOP`默认1。默认ECO在熄屏、当前命令和TX完成且接收队列清空后留1秒安静期，再关闭蓝牙供电与UART；外设空闲时进入STOP，常规200ms、静止慢待机最长400ms。选择保连时BLE常开，BLE/USART时钟保留，只使用Sleep，单次受SysTick 24位容量限制最多262ms，超过时分多次完成。STOP额外启用Flash掉电及稳压器低电压模式，醒后恢复64MHz，再补齐时间并开放中断；所有实际STOP（含常规200ms与慢待机400ms）均临时屏蔽MPU DATA_RDY/PB12 EXTI，保留PA4/RTC及已置位pending，退出时按保存值原样恢复IMR。实际功耗和采样裕量需上板测量。自动待机与 PA4 手动熄屏行为一致；ECO熄屏后关闭PA8供电及PA9/PA10 UART，保连保留，支持 PA4 或放下后抬腕亮屏。LCD 先等在途 DMA 完成或超时安全收尾再进入休眠，亮屏发 Sleep Out 后至少等待 120ms。触摸恢复由独立任务复位并等待约 310ms，不阻塞 sensor。亮屏时电池约1秒采样一次、温湿度每1秒触发一次、气压每5分钟一次；熄屏电池稳定后约10秒一次，温湿度每10分钟测量，无姿态需求时 MPU 自主进行 5Hz 加速度采样，sensor 常规每 200ms 读取新帧，稳定熄屏可放宽至 400ms。步数按本次开机累计，不写 EEPROM；首页脚印图标旁显示数值，向上图标提示辅助抬腕。活动、运动和心率所需采样频率保持原配置。

心率测量已恢复约 30ms 读取，40～200 BPM 合成回归在稳定段逐帧检查有效输出，包含 200 BPM；范围外的估算仍不钳位。实板心率精度、总线时序及功耗尚需测量。具体实现见 [界面与低功耗说明](docs/watch-ui.md)。

## 蓝牙与 OTA

普通 APP、升级 APP 和独立保底始终仅开启 BLE（`T4+01`、`T5+00`，用 `AT+B500` 关闭 SPP），用于校时、天气、现有接收协议以及 OTA 升级；全程不启用 SPP，也不需要经典蓝牙配对。正常启动直接跳转 APP，升级成功回到 APP 后 BLE 连接随复位断开，需要手机重新连接。

复位期间连续按住KEY_1（PA5，低电平）满2000ms即进入独立保底；APP向量无效、EEPROM读取失败，或EEPROM记录以BL前缀标记升级中/畸形时，主引导也进入保底。记录为空白或非BL遗留标志且APP向量有效时，主引导正常启动APP。保底经 BLE 的 `FFF1` 接收手机发来的升级 APP BIN，按`start:长度:CRC32`升级，ready 后才接收正文，连续 30 秒无进展才复位后重新等待（不是整轮限时 30 秒）。手机先查询并确认 APP 布局，再发送 start；每次 BLE 写不超过 20 字节并等写回调返回后再续，片间不小于 20ms，BIN 仍按 256 字节外层块组织，块间保留 40ms 暂停，最后一块之后不再补暂停。命令不带换行时按 1000ms 空闲判结束，因此 22 字节的 start 命令拆成 20＋2 两片仍是一条完整命令；Write Without Response 的回调只代表 Android 已接受该次写，只有设备回传匹配长度的 `ok,len:N` 才算固件校验通过。断连或整机复位后一律完整重传，不续传、不回滚。OTA 只擦写 sector 4..6，主引导、保底和日志保留。完整部署见[Bootloader说明](docs/bootloader.md)。

2026-11-02启用主引导＋独立保底＋手表APP三程序结构。升级APP的日常BLE入口为`OTA,START`加实际换行，sensor保存并读回EEPROM标志后复位进入保底，回执为`OTA,REBOOTING,BLE`。手机APK基于旧波形App扩充，BLE OTA版为`1.95-wave.17-ble-ota` / code22，目标产物`work/ble-ota-20261104/delivery/Bluetooth-Wave-BLE-OTA.apk`；源码和重建脚本见[tools/android-ota](tools/android-ota/README.md)。原签名密钥缺失，扩充版与旧波形版并存。首次使用需通过SWD部署主引导和独立保底，旧`0x08008000`的APP BIN不可沿用；只支持SPP接收的旧保底也不能被手机OTA更新，迁移到BLE OTA前必须先SWD写入`F411-OTA-Recovery.bin`。

## 工程与文档

使用 Keil ARM Compiler 5，按部署方式选择对应工程：

| 工程 / Target | Flash 起始地址 | 可用空间 | 用途 |
| --- | --- | --- | --- |
| [f411ceu6](MDK-ARM/f411ceu6.uvprojx) | `0x08000000` | 384KiB | 独立 APP，直接通过调试器下载 |
| [f411_bootloader](MDK-ARM/f411_bootloader.uvprojx) | `0x08000000` | 32KiB | 主引导，选择手表APP或独立保底 |
| [f411_recovery](MDK-ARM/f411_recovery.uvprojx) | `0x08008000` | 32KiB | 独立保底程序，负责蓝牙接收和APP擦写，向量偏移`0x8000` |
| [f411ceu6_upgrade_app](MDK-ARM/f411ceu6_upgrade_app.uvprojx) | `0x08010000` | 320KiB | 三程序结构的手表APP，向量表偏移`0x10000` |

最后128KiB（sector 7）保留给电压历史；普通APP/升级APP及Bootloader均已限制边界，旧Bootloader和只支持SPP接收的旧保底均须通过SWD同步更新，手机OTA不能替换这两个程序。下载时使用按扇区擦除，整片擦除会清掉日志。

直接运行手表可选择独立 APP。部署 OTA 时，通过 SWD 分别下载 `F411-OTA-Bootloader.bin` 到 `0x08000000`、`F411-OTA-Recovery.bin` 到 `0x08008000`、`F411-OTA-Upgrade-App.bin` 到 `0x08010000` 三个 BIN，普通复位即运行 SWD 安装的升级 APP，首次部署不要求先完成一次手机升级；之后手机只经 BLE 下发升级 APP 的 BIN，不下发主引导与保底。下载升级 APP 时保留主引导和保底所在扇区，并使用按扇区擦除。独立 APP 与主引导都占用 `0x08000000`，两者不能共存，应按上表选择部署组合。

主引导按 EEPROM 中的升级记录决定去向：记录为主引导写入且标志有效时启动 APP；标志为升级中或记录损坏时进入独立保底；记录缺失或为旧版、空白时，只有在 APP 向量表有效且 EEPROM 读取成功的情况下才启动 APP，APP 向量无效或 EEPROM 读取失败则进入保底。手动进入保底需按住返回键 KEY_1（PA5，低电平）并持续 2000ms 跨过复位，短按或空闲都正常启动，KEY_1 为高电平时不设无条件延时。升级 APP 的手机入口 `OTA,START` 仍先写入并读回持久的升级中标志再复位，因此升级中断后复位会回到保底等待完整重传，不会凭看似有效的向量表启动未完成镜像。分区保持主引导 `0x08000000`、独立保底 `0x08008000`、升级 APP `0x08010000` 不变。

应用代码在 `application/`，硬件接口在 `interface/`，公共算法与软件 I2C 在 `common/`。

| 内容 | 文档 |
| --- | --- |
| 界面、低功耗、电池条和电量估算 | [docs/watch-ui.md](docs/watch-ui.md) |
| 电压历史记录、蓝牙导出与清空 | [docs/battery-log.md](docs/battery-log.md) |
| 蓝牙、校时和天气协议 | [docs/bluetooth.md](docs/bluetooth.md) |
| 启动流程与等待优化 | [docs/startup-optimization.md](docs/startup-optimization.md) |
| 运行诊断 | [docs/diagnostics.md](docs/diagnostics.md) |
| 复位现场与 RTC / 天气状态查询 | [docs/reset-diagnostics.md](docs/reset-diagnostics.md) |
| BLE OTA 升级与部署 | [docs/bootloader.md](docs/bootloader.md) |
| 看门狗与任务进度检查 | [docs/watchdog.md](docs/watchdog.md) |
| MPU 零偏及 EEPROM 保存 | [docs/imu-eeprom.md](docs/imu-eeprom.md) |
| 磁航向与校准 | [docs/magnetometer.md](docs/magnetometer.md) |
| 心率与故障诊断 | [docs/em7028.md](docs/em7028.md) |
| 温湿度 | [docs/aht21.md](docs/aht21.md) |

常用主机检查在项目根目录运行：

```powershell
python -X utf8 tests/clock/run.py
python -X utf8 tests/sensors/run.py
python -X utf8 tests/startup/run.py
python -X utf8 tests/ui/run.py
python -X utf8 tests/battery_log/run.py
```

需要 Python、桌面 GCC；UI 渲染还需要 Pillow。主机测试、Keil 编译和实板测试是三种不同证据，不能相互代替。

## 更新记录

### 2026-11-03 · [47790a7](https://github.com/Mangta517/watch/commit/47790a76d4641694bebb2a5604bdf6bca50aeea7) · 三程序OTA与ECO低功耗整合发布

- **问题**：OTA、低功耗、传感器调度与维护日志的累计改动尚未统一发布；现有说明仍含旧分区容量及采样周期，weather_store主机测试遗漏公共头文件路径。
- **改动**：该提交位于`codex/release-20261103`，相对`main`汇总197个文件。三程序布局为主引导0x08000000、独立保底0x08008000、320KiB升级APP 0x08010000；正常启动运行有效APP，PA5复位连续按2秒进保底，BLE`OTA,START`进保底，SPP支持查询布局/长度/CRC/回读及中断完整重传，sector7日志保留。默认64MHz/Scale3与ECO，后台采样200/400ms；所有实际STOP屏蔽MPU EXTI并精确恢复，以减少不必要唤醒。AHT亮屏1秒、熄屏10分钟，心率独立`AUTO`10分钟默认OFF，电量3.50V=0%、4.00V=100%，并整合电压/复位BLE日志、天气持久化、看门狗与触摸恢复。本次发布整理仅为测试脚本补`weather_store`的`-Icommon`，保留累计固件行为。
- **验证结果**：battery_log、ble、bootloader、clock、diagnostics、ota、reset、reset_ble、sensors、startup、tickless、touch、ui、watchdog、weather_store共15套主机回归最终通过（weather_store首次漏路径失败、补后重跑通过；UI/diagnostics开关及tickless STOP开关均覆盖；clock可选外部手机帧夹具缺失未运行）。4个Keil目标f411_bootloader/f411_recovery/f411ceu6/f411ceu6_upgrade_app ARMCC5.06增量构建均0错误0警告；Android OtaProtocolTest通过，javac有3条Java8选项过时警告。
- **剩余问题**：未烧录，未测实际功耗收益、传感器精度、真实SPP与掉电恢复；完整APK未重新构建，重建仍需外部base.apk、本地工具链与扩充版签名。

### 2026-11-02 · 当前工作区 · 低功耗默认ECO、心率自动定时与慢待机

- **改动**：蓝牙默认改为ECO省电，重启恢复ECO且模式仅存RAM、不写EEPROM；心率页新增独立`AUTO / 10 MIN`开关（默认OFF、仅RAM），启用后熄屏每10分钟测最多10秒、有效BPM即停，从启用/熄屏起算而非立即触发，与手动START/STOP及SPORT强制连续各自独立，关闭AUTO时sensor停止在途测量并拉低PB15；稳定熄屏无其它业务且加速度约10秒稳定后进入慢待机，CPU采样/基础服务由200ms放宽至400ms（RTC STOP最长400ms，慢STOP临时屏蔽MPU DATA_RDY EXTI并保留PA4/RTC，双任务进度与100ms最小喂狗及0.9s硬件超时不变）；AHT测量窗口改为采样/恢复完成即收尾；触摸仅在休眠成功后阻塞等待PA4、失败首次立即其后500ms退避、亮屏通知可打断。
- **验证结果**：sensors、startup、UI（诊断开/关）、BLE、clock、tickless、touch、watchdog、diagnostics 共9套主机回归通过。普通APP与升级APP的Keil ARMCC构建均0错误、0警告，已生成HEX；未烧录，电流、计步/抬腕精度及唤醒时序尚未实板验证。Sleep单次按SysTick容量限制为最多262ms，RTC STOP最长400ms。

### 2026-10-28 · 当前工作区 · 64MHz Scale3与初始化省电配置

- **改动位置**：`Core/Src/main.c`、`tim.c`、`adc.c`、`gpio.c`、`FreeRTOSConfig.h`及`.ioc`同步64MHz/Scale3/Flash2、PWM分频、ADC Sleep停钟和空闲GPIO；`app_tickless.*`恢复默认STOP；`interface/lcd.c`在面板休眠后停止TIM3并关闭TIM3/SPI1/DMA2时钟，亮屏先恢复时钟再发命令。软件I2C/触摸按64MHz缩放位延时，Bootloader保留原时钟和总线延时。
- **行为与作用**：移除PA3及LCD_CS/LCD_DC/HEART_LED推挽输出的内部下拉，空闲引脚配置模拟无上下拉，不开启未使用的GPIOH总线时钟。STOP启用Flash掉电与低功耗稳压器低电压模式；普通模式继续保持BLE实时接收，SPORT熄屏满足原资格时进入STOP。TIM2诊断计时、1ms逻辑节拍、传感器档位、喂狗与GPIO电源保持顺序继续使用原流程。
- **验证结果**：tickless、startup、sensors、UI（诊断开/关）、clock、BLE、bootloader、diagnostics、watchdog、touch、reset、reset_ble、battery_log共13套主机回归通过。新增检查覆盖显示外设停钟前DMA空闲、亮屏命令前恢复时钟、背光保持0至恢复亮度，以及真实GPIO初始化只将选定空闲引脚置为模拟模式。三个Keil目标增量构建均0 errors、0 warnings；两套APP的MAP压缩后ROM均220316 B、RW+ZI均104216 B，HEX已生成。
- **实板边界**：未烧录；实际整板电流、64MHz下采样裕量、软件I2C时序、连续BLE与STOP唤醒延迟仍待测量。此前84MHz/Sleep条目保留为历史性能测试记录。

除注明“当前工作区”的条目外，以下日期取自 Git 提交日期。记录按新到旧排列，旧条目中的 4.2V 电量曲线等限制为当时状态，已由上述低功耗版本更新；各条目的验证结果仅适用于对应版本。

### 2026-10-31 · 当前工作区 · 恢复10分钟熄屏温湿度采样

- **行为与作用**：AHT21亮屏保持1秒、熄屏恢复10分钟，待机温湿度窗口同步采用10分钟周期；不改变气压5分钟、电池ADC熄屏10秒、BLE策略及自动心率周期。
- **改动位置**：`application/app_aht21.h`统一周期，sensors及startup回归同步检查10分钟边界、唤醒恢复、气压独立周期及tick回绕。
- **验证结果**：sensors与startup主机回归通过，两套APP的Keil构建均0错误、0警告，已生成HEX；未烧录或实板验证。

### 2026-10-31 · 当前工作区 · 以3.50V为电量下限

- **改动位置**：`application/app_battery.c`更新显示电量节点，`tests/sensors/battery_harness.c`同步边界、插值及回弹回归；标定依据见[3.50V下限标定](docs/battery-log.md#2026-10-31-以350v为显示下限重新标定)。
- **行为与作用**：按本次2851条同电池、未充电记录中的开机3～5重算3.50～4.00V节点；3.50V及以下显示0%，4.00V及以上显示100%，3.60V为24%、3.55V为14%。不增加低压关机，不调整ADC换算、采样、滤波及本次开机百分比只降不升的规则。
- **验证结果**：sensors及UI（含诊断开/关）主机回归通过；普通APP与升级APP的Keil增量构建均0错误、0警告，已生成HEX，未烧录或实板验证。

### 2026-10-28 · 当前工作区 · 蓝牙心率报错与显式清空日志

- **改动位置**：`application/app_heart.c`在发布快照锁前保存启动失败，在停测前保留原始故障；`app_diag.*`将详细故障写入原EM7028结果字段；`app_reset_ble.*`复用异步发送增加HEART,READ查询。`tests/diagnostics/`、`tests/sensors/heart_harness.c`、`tests/reset_ble/`和`tests/battery_log/`补充对应编码、初始化故障及查询回归。
- **行为与作用**：HEART,READ加换行返回状态、启用标志、READ/WRITE/PID/CONFIG等故障、REG与VALUE；DIAG导出的last_result低字节保留ERROR=2，其余三字节依次为故障类型、寄存器和值，成功读数恢复0。帧布局、现有App、采样档位保持，不新增任务或动态分配；避免只能看到init_result=2而无法确定失败步骤。
- **验证结果**：diagnostics、sensors、reset_ble、battery_log、UI（开/关）、startup共6套主机回归通过，覆盖启动CONFIG故障、原始READ不被stop失败覆盖、分包命令、BUSY后的固定快照及TX归属。普通/升级APP构建均0 errors、0 warnings，ROM均219332 B（增加452 B）、RW+ZI仍104216 B，两套HEX已生成；新增报错固件尚未烧录。
- **实板清除**：本轮已用电脑BLE连接现有固件执行一次BATLOG,CLEAR。清除前STATUS为2217条电压/2469占用槽，收到CLEARING_REBOOT后重连确认为0条/0槽、READY；只清sector 7的电压和复位历史，EEPROM校准与累计运行时间不变，后台记录重启后继续。

### 2026-10-28 · 当前工作区 · sensor超期后取消整周期等待

- **改动位置**：`application/app_task.c`修正sensor循环末尾的等待策略；`tests/startup/startup_harness.c`和`run.py`新增真实任务的耗时与超期回归；串行读取核查、模型结果和实板复测方法记录在`docs/diagnostics.md`。
- **行为与作用**：旧实现超期后重新计时，再完整等待一个周期，心率同轮读取约11.2ms时会额外停10ms；现在按时完成等待剩余时间，超期或恰好到期只让出一个tick，按实际唤醒时刻重排，避免连续补跑并保留低优先级任务执行机会。84MHz/Sleep、采样档位、总线延时、读取一致性检查、任务优先级和栈大小保持。
- **验证结果**：startup、sensors、tickless、diagnostics、watchdog共5套主机回归通过。新增覆盖8ms IMU/磁力计与额外3ms PPG重叠、持续15ms工作、120ms抢占、恰好到期、tick回绕和电池/气压/喂狗推进；真实任务模型282轮/3006ms（约93.8Hz），旧等待策略触发新增回归断言。普通APP与升级APP的Keil ARMCC构建均0 errors、0 warnings，ROM均218880 B（增加12 B），RW+ZI均104216 B；两套HEX已更新。
- **实板边界**：未烧录；93.8Hz为主机耗时模型结果。单轮串行读取仍可能超过10ms，是否恢复稳定100Hz及触摸、BLE、心率表现需新CSV和实板时序确认。

### 2026-10-28 · 当前工作区 · 84MHz Sleep与手机任务性能采集

- **改动位置**：`Core/Src/main.c`、`tim.c`、`FreeRTOSConfig.h`和`.ioc`恢复84MHz/Scale 2/Flash 2及PWM分频；`common/my_i2c.c`、`interface/touch.c`恢复位延时；`application/app_tickless.*`默认只用Sleep；`application/app_diag.*`和`app_task.c`开放手机按需诊断并补齐touch业务计时。时钟相关注释及`tests/diagnostics/`、`tests/tickless/`同步更新。
- **行为与作用**：默认不自动发送，现有App的DIAG,1/DIAG,0加换行可开启/关闭WDG1；心率波形仍关闭。四个任务提供CPU/栈指标，lvgl/sensor/touch提供周期、处理耗时和超预算；保留传感器、内存及UART统计。沿用原帧布局和CSV，约1.4秒一组，普通熄屏可持续采集；不增加任务和动态分配。Sleep保留PLL配置，唤醒继续84MHz，减少本轮实验中跨STOP恢复的变量；实际功耗变化待测。
- **验证结果**：diagnostics、tickless、startup、sensors、UI（诊断开/关）、BLE、clock、watchdog共8套主机回归通过，覆盖默认静默/分包开启/关闭后收取在途TX、四任务栈与CPU、touch周期及耗时、84MHz补时、默认禁止STOP及原可选STOP回归。普通APP、升级APP的Keil ARMCC构建均0 errors、0 warnings，ROM均218868 B、RW+ZI均104216 B；已生成各目标HEX。
- **实板边界**：未烧录，尚无实测CPU负载或BLE结果。逐字节RX中断及Flash日志机制保持，未消除Flash编程重叠接收的潜在丢字节风险；现场采集步骤、CPU首窗口及历史极值含义见[诊断说明](docs/diagnostics.md#当前测试方式2026-10-28)。

### 2026-10-28 · 当前工作区 · 偶发手机同步失败与复位风险修复

- **改动位置**：`interface/kt6368a.*`、`application/app_ui.c`记录并消费UART实际到达时间及错误边界；`application/app_reset.c`缩短正常存档的关中断窗口；`interface/watch_rtc.*`改为有界运行期校时；`tests/ble/`、`tests/ui/`、`tests/reset/`、`tests/clock/`补充对应故障回归。
- **行为与作用**：修复后半包已到队列、显示任务延迟超过200ms才消费时将完整时间/天气拆坏的问题；真实空闲和无换行消息仍按200ms分帧。UART错误不再清空错误后全部队列，按损坏位置恢复；错误字节不作为有效业务输入。喂狗检查点在开中断时计算CRC，异常现场仍优先完整提交。RTC时钟丢失或同步失败时校时有界退出，下一条合法消息可在时钟恢复后重试，避免显示任务进入HAL的1s等待。
- **资源与验证**：接收队列增加4096字节到达时间及256字节边界元数据，不新增任务。clock、sensors、startup、UI（诊断开/关）、BLE、bootloader、battery_log、reset、reset_ble、tickless、touch、watchdog、diagnostics共13套主机回归通过；新增覆盖250ms消费延迟、真实空闲和错误后的首帧、199/200ms入队竞态、UART错误回调顺序、RTC有界失败/恢复及1088个CRC中途异常注入点。普通/升级APP与Bootloader本次Keil增量构建均0 errors、0 warnings；两套APP的MAP压缩后ROM均219564 B、RW+ZI均104136 B，Bootloader ROM17916 B、RW+ZI15720 B。实际ARM反汇编确认正常检查点的CRC在恢复IRQ后计算，提交前重新核对异常状态；CRLF空白检查通过。
- **实板边界**：没有烧录。软件修复不能覆盖MCU硬件复位时PA3释放导致TPS_EN掉锁的电源保持缺口；关机最初触发源仍需实板日志和波形确认。当前首页仍显示板载AHT21温湿度，手机天气在独立天气页显示，布局本次不改。

### 2026-10-27 · 当前工作区 · 最新放电记录校准电量

- **改动位置**：`application/app_battery.c`更新电压—百分比节点，`tests/sensors/battery_harness.c`同步标定与回弹回归；计算依据见[电压日志说明](docs/battery-log.md#2026-10-27-最新电压记录校准)。
- **行为与作用**：依据1818条同电池、未充电且末尾仍正常运行的记录，使用1815条连续记录及末6小时下降趋势重标3.55～4.0V。3.60V由10%调整到约31%，新增3.55V/18%节点，改善旧低压段偏低的显示；保留4.00V/100%、3.50V/5%和3.30V/0%显示锚点。每秒采样、启动均值、1/8滤波、ADC失败处理及本次开机电量只降不升均不变。
- **验证结果**：sensors与UI（诊断开/关）主机回归通过，覆盖新曲线插值、启动、电压回弹及ADC恢复；两套APP本次Keil增量构建均0 errors、0 warnings，ROM均218876 B、RW+ZI均99784 B。
- **验证边界**：未烧录。3500mV到达时间来自短程趋势外推，5%与0%锚点仍待完整放电实测，不能把本次显示估算当作容量或精确剩余续航。

### 2026-10-27 · 当前工作区 · STOP混合休眠与48MHz运行

- **改动位置**：`application/app_tickless.*`、`application/app_task.*`、`interface/watch_rtc.*`、`Core/Src/stm32f4xx_it.c`及头文件负责STOP资格、RTC唤醒、时钟恢复和补时；`Core/Src/main.c`、`Core/Src/tim.c`、`FreeRTOSConfig.h`、`.ioc`同步48MHz/Scale3/Flash1及PWM；`common/my_i2c.c`、`interface/touch.c`同步软件总线延时，Bootloader保留100MHz原延时。
- **行为与作用**：BLE开启时使用Tickless Sleep，保留115200串口接收；BLE关闭且熄屏、外设空闲时使用低功耗调压器STOP，由RTC/EXTI唤醒，恢复PLL和RTC影子后补齐HAL/RTOS/TIM2时间。降低运行主频及电压档，减少合适空闲间隙的MCU功耗；传感器采样周期、按键、辅助抬腕及双任务进度喂狗策略保持原配置。当前普通待机仍保留BLE，STOP主要用于SPORT熄屏后的采样间隙。
- **验证结果**：clock、sensors、startup、UI（含诊断开/关）、BLE、bootloader、tickless、touch、watchdog、diagnostics共10套主机回归通过。两套APP全量Keil重编译均0 errors、13条既有LVGL warnings，最终增量构建均0 errors、0 warnings；ROM均218876 B，RW+ZI均99784 B。RTC/STOP测试覆盖有界等待、失败回退、提前唤醒、时钟/中断顺序、节拍及诊断补时、最近任务截止时间和外设忙时回退；CRLF空白检查通过。
- **验证边界**：未烧录或实板测量；实际整板电流、100Hz采样裕量、软件I2C时序、UI帧率、连续BLE及STOP唤醒延迟待验证。硬件依据见[ST数据手册](https://www.st.com/resource/en/datasheet/stm32f411ce.pdf)：Scale3适用于HCLK≤64MHz，STOP停止USART所需时钟。

### 2026-10-26 · [84f62b5](https://github.com/Mangta517/watch/commit/84f62b54a0650cafcabe11460740da64ae6885e4) · 后台活动识别、熄屏通信与复位 / 电压日志

- **问题**：原先 5 秒无操作只调暗背光，普通手动熄屏会关闭蓝牙；缺少后台计步、辅助抬腕及跨重启的电压记录，偶发重启也缺少可查询的异常现场。显示分块调度、拖动后的点击判断和电量插值也需要调整。
- **改动与优化效果**：

| 模块 / 主要文件 | 改了哪里 | 优化效果 |
| --- | --- | --- |
| 后台采样：`application/app_task.c`、`application/app_imu.c`、`interface/mpu6050.c`、`interface/lsm303dlhc.c` | 无姿态需求时使用 MPU 自主 5Hz 加速度低功耗循环，关闭陀螺仪和温度、休眠磁力计，sensor 每 200ms 取新帧；姿态页与校准保留 50Hz，运动和心率保留 100Hz 六轴采样 | 减少普通亮屏 / 熄屏后台的传感器与总线工作量，同时保留活跃场景所需采样频率；节电幅度尚未实测 |
| 计步与辅助抬腕：`common/common_activity.c`、`application/app_imu.c`、`application/app_ui.c` | 增加加速度峰谷计步，连续 3 个有效步态后回补；首页显示本次开机累计步数和脚印图标；放下后转向观看姿态可触发一次亮屏，并丢弃过期唤醒事件 | 低频后台也能提供日常步行计数和辅助亮屏；步数只存 RAM，重启后重新累计 |
| 待机与 BLE：`application/app_ui.c`、`application/app_task.c`、`application/app_touch.c`、`interface/kt6368a.c` | 由 5 秒调暗改为 10 秒完整熄屏；LCD / 触摸芯片休眠、触摸任务阻塞；普通熄屏保留 BLE 电源、UART 和消息解析，接收达到水位时通知任务；SPORT 启停仍控制蓝牙电源 | 减少熄屏绘图和轮询，待机时继续接收校时、天气及查询，无需因屏幕明暗重启蓝牙 |
| 显示与触摸：`interface/lcd.c`、`Middlewares/LVGL/lvgl/examples/porting/lv_port_disp_template.c`、`application/app_ui.c` | 先启动像素 DMA 再让 sensor 运行；LCD 地址参数按组发送、SPI 阻塞超时缩短为 50ms；指南针只重绘旧 / 新指针覆盖区域；睡眠前等 DMA 完成或安全收尾，唤醒后等待至少 120ms；卡片拖动后取消点击、详情右滑返回功能页并恢复滚动位置 | 让传感器执行与像素传输重叠，减少重复发送和重绘，避免面板休眠与在途传输冲突，并减少拖动误入页面 |
| 复位与状态诊断：`application/app_reset.c`、`application/app_reset_ble.c`、`Core/Src/stm32f4xx_it.c`、`interface/watch_rtc.c` | 在 RTC 备份寄存器保存复位标志和可取得的异常现场，支持 `RESET,READ`；`STATE,READ` 查询 RTC / LSE、校时和天气快照；LSE 停振时不把冻结时间当作有效读取，LVGL 断言也记录停机原因 | 为偶发重启、校时及显示异常提供可导出的证据，帮助区分异常现场、同步状态和读取失败；不代表已确定或修复物理复位原因 |
| 电压历史与升级：`application/app_battery_log*.c`、`interface/int_battery_log.c`、`interface/int_bootloader.c`、`bootloader/boot_main.c`、`MDK-ARM/*.uvprojx`、`tools/send_bootloader.py` | 每分钟追加滤波电压与开机秒数，按开机编号通过 `BATLOG` 查询 / 导出；写满停止，显式 CLEAR 才擦除并重启。sector 7 的 128KiB 留给日志，普通 / 升级 APP 限制为 384 / 352KiB；EEPROM 保存升级中 / 有效标志 | 保留跨重启的放电趋势，并使链接边界和升级长度上限避开日志区；升级未完成时留在 Bootloader，避免启动残缺镜像 |
| 电量估算与首页：`application/app_battery.c`、`application/app_ui.c` | 按未充电日志局部修正 3.6～4.0V 的插值节点，保留 4.0V 满电、启动均值、Q8 滤波及本次开机百分比只降不升；调整首页步数和脚印位置 | 让已测电压区间的估算更贴合本地记录，避免脚印贴顶裁切；3.6V 以下仍是估算节点，不能据此给出精确剩余时长 |
| 上次开机时长：`application/app_uptime.c`、`application/app_ui.c` | 将启动时读到的上次记录单独保存，在收到合法手机校时消息后每次开机只报告一次，并与日志 / 诊断发送共享 UART | 避免本次运行的新存档覆盖“上次开机”报告内容，保留消息分包、BUSY 和发送失败时的处理 |

- **验证结果**：`84f62b5` 推送前，在独立工作树中通过 clock、sensors、startup、UI（含诊断开 / 关）、BLE、bootloader（含发送脚本）、battery_log、reset、reset_ble、tickless、touch、watchdog、diagnostics 共 13 套现有主机回归；CRLF 空白检查通过，远端提交号一致且同步差异为 `0 0`。本次 README 补充只修改文档，沿用该功能提交的测试结果，没有重新运行固件测试、Keil 编译或实板验证。
- **剩余问题**：实际待机电流、唤醒延迟、屏幕帧率、连续 BLE / SPP 传输和 Flash 操作时序尚需实测；偶发重启的物理触发原因仍未确定。计步算法即使在 100Hz 六轴模式下也按至少 200ms 间隔消费，5Hz 面向日常步行，快速跑步可能混叠漏计，佩戴精度尚未验证。电池低压端尚未完成耗尽标定，充电期间百分比回升仍需重新上电估算。

### 2026-10-22 · [91036d2](https://github.com/Mangta517/watch/commit/91036d2dd975e360ba9e01752055bac17a4ec5e0) · 低功耗、4.0V 电量估算与 OTA 蓝牙策略

- **问题**：无触摸时表盘仍刷新、背光偏亮；CPU 和传感器持续以较高频率运行；日常 SPP 开启多余；旧电量曲线以 4.2V 为满电点。
- **改动**：5 秒无触摸后降至 2% 背光并暂停表盘刷新；APP 固定 84MHz/Scale 2，空闲进入 Sleep，待机轮询约 20ms。运动、心率与校准时 MPU 为 50Hz，后台 25Hz；同步调整磁力计、心率读取、滤波与校准超时。电量满电点改为 4.0V。日常仅 BLE，Bootloader 进入 OTA 时开启 SPP，并补齐两个 APP 与 Bootloader 工程及其触摸任务、看门狗、分步恢复和保存依赖。
- **验证结果**：`91036d2` 发布前，在独立 worktree 中通过 clock、sensors、startup、UI（含关闭诊断发送）、BLE、bootloader、touch、watchdog、diagnostics 主机回归；三个工程引用的源码齐全，远端同步为 `0 0`。同源本地普通 APP 与升级 APP 此前 Keil 编译均为 0 错误、0 警告；发布时没有重新执行 Keil。本次 README 补充仅做文档检查，不重跑固件测试。
- **剩余问题**：未烧录或完成实板功耗、心率准确率及完整 OTA 链路验证。40ms 心率采样在接近 200 BPM 时可能间歇回到测量状态；电量仍为软件估算，充电期间百分比回升需要重新上电估算。

### 2026-10-22 · [afeba21](https://github.com/Mangta517/watch/commit/afeba210018c5abc351a7669eb8a0d3ce010df64) · 电池条、滤波与放电估算

- **问题**：原界面只显示电压；直接显示采样值易跳动；用户观察到接电池后约 3.8V 在六七秒内升至约 3.9V，百分比跟随回升，造成电量虚增。
- **改动**：增加绿色电池形进度条、估算百分比和低电量红色提示。启动采集 8 次有效读数，以后 4 次均值初始化显示，正常约等待 7 秒；随后电压按 1/8 低通更新。电压可升降，本次开机百分比只降不升；保留原始 ADC 值及采样错误状态。
- **验证结果**：实际待推送版本在独立 worktree 中通过 clock、sensors、startup、UI 四套主机测试和空白检查。新增用例覆盖启动 3.8V→3.9V、电压回弹、电量不回涨、ADC 故障恢复，以及电池条半电/空电/满电、像素填充和切页。该独立分支版本未单独运行 Keil 或实板验证。
- **剩余问题**：电池标签标称 4.0V，但充电限制电压尚未确认；估算曲线暂按板上 TP4056 的 4.2V 上限，仍需用实际电芯校准。未接入充电状态检测，充电期间电量也不会增长，需重新上电重新估算。约 7 秒的启动采样窗口是否足够、启动电压变化的实际原因，仍需实板确认。

### 2026-10-18 · [de4da10](https://github.com/Mangta517/watch/commit/de4da10bfc9d1236f62e58692b4ef6968e00e67e) · 天气状态和图标

- **问题**：天气页需要更直观地区分晴、多云、雨雪等天气，并展示手机同步状态。
- **改动**：复用已有 WMO 天气码和手机协议，增加天气文字、LVGL 绘制的天气图标、温湿度和同步时间；覆盖未知天气码及过期状态，不引入额外位图资源。
- **验证结果**：当次推送记录中的 clock、sensors、startup、UI 四套主机测试通过，界面渲染及 CRLF 空白检查通过；测试包含天气标签、图标、过期颜色和无效帧保留。
- **剩余问题**：未在该次推送中运行 Keil、烧录或手机实测。手机定位、天气服务、真实 BLE 传输和屏幕显示仍需联调。

### 2026-10-17 · [201290f](https://github.com/Mangta517/watch/commit/201290fd4ec60690f9ba18744b8ffb0d71da3161) · UART 丢包后的解析恢复

- **问题**：UART 丢字节或接收错误后，校时、天气、诊断解析器可能继续使用残留半帧，导致错误拼接或恢复不完整。
- **改动**：为相关解析器增加接收状态重置，接收入口在丢包、错误及计数变化时清理残帧，并在换行或接收空闲后恢复解析；保留最后有效的时间和天气数据。
- **验证结果**：提交包含诊断和 UI 故障注入用例，覆盖错误、计数回绕、残留后缀丢弃和有效帧恢复。未找到该旧版本独立执行结果的完整记录，本次文档整理未重跑该版本。
- **剩余问题**：用例存在不等于实机验证通过；真实 UART 溢出、蓝牙断续发送及板上恢复时序仍需验证。

### 2026-10-14 · [fbd8928](https://github.com/Mangta517/watch/commit/fbd89280178acf4a51e80f919e7f96468b4967f0) · 启动诊断、IMU 保存与代码规范

- **问题**：RTC/蓝牙启动等待影响早期采样；MPU 零偏需要掉电保存；缺少分阶段诊断，部分自定义代码不符合项目风格。
- **改动**：分阶段推进 RTC、蓝牙与 IMU 校准，增加启动和传感器诊断；增加 MPU EEPROM 双副本、CRC、恢复及重新校准流程；减少重复 UI 更新并缓存主页面。统一自定义代码的大括号、缩进和运行时条件分支，保留接口与参数。
- **验证结果**：当时的提交文档和验证记录记载 sensors、startup、diagnostics、UI、clock、BLE 主机回归通过，Keil ARMCC 5.06 构建 0 错误、0 警告；手机端另有构建及签名检查。依据见该版本的 [启动文档](https://github.com/Mangta517/watch/blob/fbd89280178acf4a51e80f919e7f96468b4967f0/docs/startup-optimization.md) 和 [EEPROM 文档](https://github.com/Mangta517/watch/blob/fbd89280178acf4a51e80f919e7f96468b4967f0/docs/imu-eeprom.md)。
- **剩余问题**：实际启动时间、EEPROM 断电重启恢复、手机真实通信和调度表现仍需实板验证。构建与签名结果不能证明已经烧录或安装。

### 2026-10-13 · [d6ba327](https://github.com/Mangta517/watch/commit/d6ba327c612b318e759625cea150c5f20e9e96a7) · 电池电压与手机天气服务

- **问题**：原工程缺少独立电池电压服务及手机天气快照，表盘和功能入口需要统一整理。
- **改动**：增加 PA1/ADC1_IN1 电池采样、互斥快照和异常显示；增加天气消息解析与快照，更新 SYDNEY 表盘、七项功能入口及相关 RTC、任务、CubeMX 和 Keil 配置。
- **验证结果**：当次推送记录中的 clock、sensors、startup、UI 四套主机测试通过，UI 导航、天气、电池、蓝牙及压力场景通过。该次推送没有重新运行 Keil，也没有实板验证。
- **剩余问题**：该版显示电压，不显示电量百分比；参考电压按标称 3.3V，需实测校准。天气依赖手机同步；实体电路、RTC、触摸和通信表现仍需上板确认。

### 2026-10-13 · [7a54097](https://github.com/Mangta517/watch/commit/7a5409764cb3778cc2122034b4dda62e4c35b2b0) · 心率、温湿度与蓝牙波形

- **问题**：缺少完整心率与温湿度应用服务，心率基线和突变恢复影响连续输出，调试时需要原始波形与状态。
- **改动**：接入 EM7028 心率和 AHT21 温湿度服务、页面及诊断；蓝牙发送八通道 JustFloat 波形，补充实测 CSV 回放；调整心率启动基线、突变恢复、峰间隔处理及 SENSOR I2C 时序，并同步 RTC、背光等配置。
- **验证结果**：提交说明记载 UI、蓝牙、RTC、传感器主机测试通过，Keil 构建 0 错误、0 警告；两段实测数据完成离线回放。
- **剩余问题**：离线回放不是实时测量准确度证明；佩戴、运动、接触情况及参考心率对照仍需实板验证。

### 2026-10-12 · [2b03bf1](https://github.com/Mangta517/watch/commit/2b03bf1623912f293927f45e7f0e1dae3a26251f) · 蓝牙校时与驱动故障处理

- **问题**：需要通过蓝牙设置时间；SPL006 初始化、LCD 单色填充及触摸总线恢复的错误处理不完整，任务内硬件等待影响其他任务运行。
- **改动**：接入 KT6368A 收发、LSE RTC 和分段时分校时；整理表盘与设置页；检查气压系数和状态、LCD 分块写入结果，增强触摸恢复；调度器运行后使用可让出 CPU 的等待。
- **验证结果**：提交说明记载四套主机测试通过，Keil 构建 0 错误、0 警告；补充 RTC、BLE、UI 和真实驱动故障注入测试。
- **剩余问题**：当次未进行实物验证，真实 I2C 波形、外设恢复、RTC 走时、触摸及蓝牙收发仍需确认。

### 2026-10-11 · [ff238aa](https://github.com/Mangta517/watch/commit/ff238aaa42e8f3fb4c4274aa0b34f7e44d406b10) · 磁航向与 EEPROM 校准

- **问题**：需要结合磁力计获得航向，校准结果需要掉电保存；EEPROM 引脚定义需要与硬件一致。
- **改动**：加入 LSM303DLHC 磁航向融合、BL24C02F 双副本校准记录与桌面测试；修正 EEPROM 为 PA12=SCL、PA11=SDA，同步 UI、运动参数、CubeMX 和 Keil 配置。
- **验证结果**：提交说明记载 ARMCC5 编译 0 错误、0 警告，传感器、EEPROM 保存恢复及界面测试通过。版本文档也记录过 LVGL 重编译产生的第三方警告，不能将一次无警告结果推广到所有构建。
- **剩余问题**：实物上拉、I2C 波形、装配磁干扰、航向精度及运行时栈余量仍需测量；没有硬件功耗测量结果。

### 2026-09-17 · [5f21046](https://github.com/Mangta517/watch/commit/5f21046babcf79756eb5f0cadb9e72024011966d) · 初始工程

- **问题**：建立可继续开发的 STM32F411CEU6 手表工程基线。
- **改动**：初次导入 FreeRTOS、LVGL 8.2、MPU6050、SPL006 等工程内容。
- **验证结果**：初始提交没有完整的项目级验证结果记录，本次未重新检出和验证该旧版。
- **剩余问题**：该基线的完整功能覆盖及实板运行状态未记录，不能推断为已经全部验证。

## 后续记录规则

1. 每次提交的提交说明都必须写清“改动：涉及模块 / 文件及行为变化”和“优化：解决的问题或带来的作用”，不能只写“更新”或“优化”。功能或修复提交完成后，把确定的提交号、日期和四项说明补入本页，并在推送时同步提交文档。
2. “问题”写具体触发现象；“改动”写涉及模块 / 主要文件、修改后的行为、优化效果及必要代价，不能只写“优化了一下”；没有实测的数据不写成已验证的收益。
3. “验证结果”写实际执行的检查、对应代码版本及结果；没有执行写“未运行”，没有历史证据写“未记录”。
4. “剩余问题”保留未解决缺陷、待实板验证项和功能限制；后续修复用新提交链接追溯，不抹去旧版交付时的限制。
5. 纯文档维护提交不重复列为功能版本，避免为填写本条记录自身的提交号而反复提交。

新条目沿用以下格式：

```text
日期 · 提交号链接 · 版本主题
问题：具体触发现象或功能缺口。
改动：涉及模块 / 主要文件、修改后的行为、优化效果及必要代价。
验证结果：代码版本、实际执行的检查及结果。
剩余问题：尚未解决或尚待验证的内容。
```
