# STM32F411 智能手表

基于 STM32F411CEU6、FreeRTOS 和 LVGL 的手表工程，包含 RTC 表盘、蓝牙校时与天气同步、运动与环境传感器、心率和电池显示。

本页对应 `codex/low-power` 分支，当前功能版本为 [84f62b5](https://github.com/Mangta517/watch/commit/84f62b54a0650cafcabe11460740da64ae6885e4)（2026-10-26）。普通 APP 和升级 APP 共用业务源码，包含低功耗调度、计步与辅助抬腕、日常 BLE 通信、复位和电压日志，以及 Bootloader SPP 升级。本次涉及的文件、具体改动、优化效果及验证结果见[更新记录](#更新记录)。后面的历史条目保留各原始提交的交付记录，部分提交来自其他职责分支；当前行为以本页首节及当前源码为准。

## 当前低功耗策略

| 项目 | 当前行为 |
| --- | --- |
| 主频与电压档 | APP 固定 84MHz / Scale 2，Flash 2 等待周期；亮屏、测量和待机都使用此配置 |
| CPU Sleep | 所有应用任务等待时，由 FreeRTOS IDLE 进入 Sleep；亮屏、运动、心率与校准期间也可进入，中断唤醒后继续执行 |
| 无触摸待机 | 连续 10 秒无操作后，与 PA4 手动熄屏共用处理：背光 0%、保留蓝牙供电/UART、暂停绘图，LCD/触摸芯片休眠且触摸任务阻塞；PA4 或辅助抬腕亮屏，恢复原亮度和当前 RTC 时间 |
| 显示与触摸轮询 | 活跃约 5ms；熄屏无按键时等待 PA4 通知或约 100ms 期限，按键消抖/按住期间保持 20ms，触摸休眠后阻塞 |
| MPU6050 | 心率测量及独立运动模式 100Hz，姿态页与校准 50Hz；无姿态需求的亮屏页面与普通熄屏使用 5Hz 加速度低功耗循环，关闭陀螺仪和温度，持续计步及辅助抬腕；自动心率窗口保留 100Hz |
| 磁力计 | 心率测量及独立运动模式 75Hz，活动/校准 30Hz；5Hz 加速度后台模式下保持 sleep，进入姿态/校准/运动/心率场景恢复 |
| 心率读取 | 测量期间 sensor 每 10ms 服务，约 30ms 读取一次 PPG；心率测量期间 MPU 为 100Hz，校准期间保留 50Hz |
| 零偏校准 | 保留 100 帧静止确认和 100 帧平均，50Hz 下约 4 秒；两个阶段超时分别为 6 秒、4 秒 |
| 电池估算 | 以 4.0V 为满电点，2026-10-25按未充电日志局部校正 3.6～4.0V 段；低压端待完整放电标定，保留启动均值、滤波和本次开机百分比只降不升 |

Sleep 已接入 Tickless：运行时保持 1ms SysTick，空闲时合并节拍，以 RTC 亚秒差值测量休眠时间，唤醒后同时补齐 HAL 和 FreeRTOS 时间。单次最多 100ms，UART、DMA、背光 PWM 和 RTC 保持工作；RTC 未就绪时退回 1ms Sleep。实际节电幅度需上板测量。自动待机与 PA4 手动熄屏行为一致，普通模式保留 PA8 供电及 PA9/PA10 UART，支持 PA4 或放下后抬腕亮屏；LCD 先等在途 DMA 完成或超时安全收尾再进入休眠，亮屏发 Sleep Out 后至少等待 120ms。触摸恢复由独立任务复位并等待约 310ms，不阻塞 sensor。亮屏时气压、电池约 1 秒服务一次，温湿度约 2 秒触发一次；普通熄屏电池保持约 1 秒一次，环境与自动心率每 10 分钟测量，无姿态需求时 MPU 自主进行 5Hz 加速度采样，sensor 每 200ms 读取新帧。步数按本次开机累计，不写 EEPROM；首页脚印图标旁显示数值，向上图标提示辅助抬腕。活动、运动和心率所需采样频率保持原配置。

心率测量已恢复约 30ms 读取，40～200 BPM 合成回归在稳定段逐帧检查有效输出，包含 200 BPM；范围外的估算仍不钳位。实板心率精度、总线时序及功耗尚需测量。具体实现见 [界面与低功耗说明](docs/watch-ui.md)。

## 蓝牙与 OTA

普通 APP 和升级 APP 日常始终仅开启 BLE（`T4+01`、`T5+00`），用于校时、天气及现有接收协议。只有进入 Bootloader 升级模式时才开启 SPP（`T5+01`，BLE 仍保留）；正常启动直接跳转 APP，升级成功回到 APP 后重新关闭 SPP，需要手机重新连接 BLE。

按住 KEY_1（PA5，低电平）上电或复位进入升级模式；APP 无效时也会停留在升级模式。连接 SPP 串口后使用 `start:长度:CRC32` 协议发送升级 APP 的原始 BIN。设备返回 ready 后才发送正文，连续 30 秒无新字节会中止；该方案为单 APP，开始升级后会擦除旧镜像。完整部署、串口脚本及错误处理见 [Bootloader 说明](docs/bootloader.md)。

## 工程与文档

使用 Keil ARM Compiler 5，按部署方式选择对应工程：

| 工程 / Target | Flash 起始地址 | 可用空间 | 用途 |
| --- | --- | --- | --- |
| [f411ceu6](MDK-ARM/f411ceu6.uvprojx) | `0x08000000` | 384KiB | 独立 APP，直接通过调试器下载 |
| [f411_bootloader](MDK-ARM/f411_bootloader.uvprojx) | `0x08000000` | 32KiB | 蓝牙升级引导程序 |
| [f411ceu6_upgrade_app](MDK-ARM/f411ceu6_upgrade_app.uvprojx) | `0x08008000` | 352KiB | 与 Bootloader 配套的升级 APP，向量表偏移 `0x8000` |

最后128KiB（sector 7）保留给电压历史；普通APP/升级APP及Bootloader均已限制边界，旧Bootloader须通过SWD同步更新。下载时使用按扇区擦除，整片擦除会清掉日志。

直接运行手表可选择独立 APP。部署 OTA 时，先通过 SWD 下载 Bootloader，再下载升级 APP 或进入 SPP 模式发送其 BIN；下载升级 APP 时保留 Bootloader 所在扇区。独立 APP 与 Bootloader 地址重叠，应按上表选择部署组合。

新版 Bootloader 还检查 EEPROM 中的 APP 有效标志。仅通过 SWD 下载升级 APP 不会自动建立此标志；首次部署或标志无效时，需要成功完成一次 SPP 升级，Bootloader 才会将镜像标为有效并允许正常启动。升级中断时会保留在升级模式，不能只凭向量表看似有效就启动未完成镜像。

应用代码在 `application/`，硬件接口在 `interface/`，公共算法与软件 I2C 在 `common/`。

| 内容 | 文档 |
| --- | --- |
| 界面、低功耗、电池条和电量估算 | [docs/watch-ui.md](docs/watch-ui.md) |
| 电压历史记录、蓝牙导出与清空 | [docs/battery-log.md](docs/battery-log.md) |
| 蓝牙、校时和天气协议 | [docs/bluetooth.md](docs/bluetooth.md) |
| 启动流程与等待优化 | [docs/startup-optimization.md](docs/startup-optimization.md) |
| 运行诊断 | [docs/diagnostics.md](docs/diagnostics.md) |
| 复位现场与 RTC / 天气状态查询 | [docs/reset-diagnostics.md](docs/reset-diagnostics.md) |
| BLE / SPP 升级与部署 | [docs/bootloader.md](docs/bootloader.md) |
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

以下日期取自 Git 提交日期。记录按新到旧排列，旧条目中的 4.2V 电量曲线等限制为当时状态，已由上述低功耗版本更新；各条目的验证结果仅适用于对应版本。

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

1. 功能或修复提交完成后，把确定的提交号、日期和四项说明补入本页，并在推送时同步提交文档。
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
