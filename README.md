# STM32F411 智能手表

基于 STM32F411CEU6、FreeRTOS 和 LVGL 的手表工程，包含 RTC 表盘、蓝牙校时与天气同步、运动与环境传感器、心率和电池显示。

本页对应 `codex/low-power` 分支，当前版本包含64MHz / STOP低功耗调度、外设停钟、传感器错峰、按需任务与心率故障诊断、计步与辅助抬腕、日常BLE通信、复位和电压日志，以及Bootloader SPP升级。普通APP和升级APP共用业务源码。改动位置、行为、优化作用和验证结果见[更新记录](#更新记录)；历史条目保留原提交的交付记录，当前行为以本页首节及当前源码为准。

`watch` 仓库统一提供 **独立 APP、Bootloader、升级 APP** 三个可分别编译的 Keil 工程。三个工程共享本仓库中的驱动、库和业务源码，下载时请保留完整目录。工程选择、Flash 分区和部署组合见[三个 Keil 工程](#三个-keil-工程)，首次编译见[获取与编译](#获取与编译)。

## 当前低功耗策略

| 项目 | 当前行为 |
| --- | --- |
| 主频与电压档 | APP 固定 64MHz / Scale 3，Flash 2 等待周期；APB1=32MHz、APB2=64MHz、SPI1=32MHz，背光 PWM 仍约 3.32kHz |
| CPU Sleep / STOP | BLE 开启或外设仍需时钟时使用 Tickless Sleep；BLE 断电且 USART1 关闭、熄屏、PWM 为 0、SPI / DMA / ADC 空闲且预期空闲至少 5ms 时使用低功耗调压器 STOP；STOP 开关默认启用，同时开启 Flash 掉电和稳压器低电压模式 |
| GPIO 与外设停钟 | 空闲 PA0/PA2/PA6/PA7/PB10/PC13 配置为模拟无上下拉，移除电源保持及屏幕控制等推挽输出的内部下拉；熄屏停止 TIM3 并关闭 TIM3/SPI1/DMA2 时钟，ADC 在 Sleep 停钟 |
| 手机性能与心率诊断 | DIAG,1 / DIAG,0 按需开启或关闭原 WDG1 诊断，补齐 touch 业务计时；HEART,READ 查询心率状态及具体故障，默认静默 |
| 无触摸待机 | 连续 10 秒无操作后，与 PA4 手动熄屏共用处理：背光 0%、保留蓝牙供电/UART、暂停绘图，LCD/触摸芯片休眠且触摸任务阻塞；PA4 或辅助抬腕亮屏，恢复原亮度和当前 RTC 时间 |
| 显示与触摸轮询 | 活跃约 5ms；熄屏无按键时等待 PA4 通知或约 100ms 期限，按键消抖/按住期间保持 20ms，触摸休眠后阻塞 |
| MPU6050 | 心率测量及独立运动模式 100Hz，姿态页与校准 50Hz；无姿态需求的亮屏页面与普通熄屏使用 5Hz 加速度低功耗循环，关闭陀螺仪和温度，持续计步及辅助抬腕；自动心率窗口保留 100Hz |
| 磁力计 | 心率测量及独立运动模式 75Hz，活动/校准 30Hz；5Hz 加速度后台模式下保持 sleep，进入姿态/校准/运动/心率场景恢复 |
| 心率读取 | sensor 目标 10ms，PPG 读取下限 30ms；有 PPG 事务时将磁读取延后一轮，MPU 继续读取；心率测量期间 MPU 配置 100Hz，校准期间 50Hz，实际采集率以实板诊断为准 |
| 首页心率 | 显示本次开机最近一次有效结果，停测和故障后保留；详情、趋势及蓝牙继续使用实时状态与读数 |
| 零偏校准 | 保留 100 帧静止确认和 100 帧平均，50Hz 下约 4 秒；两个阶段超时分别为 6 秒、4 秒 |
| 电池估算 | 4.0V 为满电点，2026-10-27依据最新未充电记录重标：3.6V约31%、3.55V约18%；3.50V/5%和耗尽端仍待实测，保留每秒采样、启动均值、滤波及本次开机百分比只降不升 |

Sleep / STOP 共用 Tickless 补时，运行时保持 1ms SysTick，单次最多 100ms。BLE 开启时保留 Sleep，UART、DMA 和背光 PWM 持续工作；BLE 关闭且满足空闲条件时暂停 SysTick，由 RTC WUT 或 PA4 / 传感器 EXTI 唤醒 STOP，先恢复 64MHz PLL 并同步 RTC 影子寄存器，再补齐 HAL / FreeRTOS / TIM2 诊断时间，最后开放中断。当前正常待机保持 BLE，仍使用 Sleep；SPORT 熄屏时可利用采样间隙进入 STOP，100Hz 服务周期不变。RTC 未就绪时退回 1ms Sleep，WUT 配置失败退回 Tickless Sleep；STOP 后 RTC 同步失败则停用 RTC 有效状态并退回 1ms Sleep，本次停钟间隔因缺少有效 RTC 读数无法准确补齐。实际功耗和采样裕量需上板测量。自动待机与 PA4 手动熄屏行为一致，普通模式保留 PA8 供电及 PA9/PA10 UART，支持 PA4 或放下后抬腕亮屏；LCD 先等在途 DMA 完成或超时安全收尾再进入休眠，亮屏发 Sleep Out 后至少等待 120ms。触摸恢复由独立任务复位并等待约 310ms，不阻塞 sensor。亮屏时气压、电池约 1 秒服务一次，温湿度约 2 秒触发一次；普通熄屏电池保持约 1 秒一次，环境与自动心率每 10 分钟测量，无姿态需求时 MPU 自主进行 5Hz 加速度采样，sensor 每 200ms 读取新帧。步数按本次开机累计，不写 EEPROM；首页脚印图标旁显示数值，向上图标提示辅助抬腕。活动、运动和心率所需采样频率保持原配置。

心率测量已恢复约 30ms 读取，40～200 BPM 合成回归在稳定段逐帧检查有效输出，包含 200 BPM；范围外的估算仍不钳位。实板心率精度、总线时序及功耗尚需测量。具体实现见 [界面与低功耗说明](docs/watch-ui.md)。

## 蓝牙与 OTA

普通 APP 和升级 APP 日常始终仅开启 BLE（`T4+01`、`T5+00`），用于校时、天气及现有接收协议。只有进入 Bootloader 升级模式时才开启 SPP（`T5+01`，BLE 仍保留）；正常启动直接跳转 APP，升级成功回到 APP 后重新关闭 SPP，需要手机重新连接 BLE。

按住 KEY_1（PA5，低电平）上电或复位进入升级模式；APP 无效时也会停留在升级模式。连接 SPP 串口后使用 `start:长度:CRC32` 协议发送升级 APP 的原始 BIN。设备返回 ready 后才发送正文，连续 30 秒无新字节会中止；该方案为单 APP，开始升级后会擦除旧镜像。完整部署、串口脚本及错误处理见 [Bootloader 说明](docs/bootloader.md)。

## 三个 Keil 工程

使用 Keil ARM Compiler 5，按部署方式选择对应工程：

| 工程 / Target | Flash 起始地址 | 可用空间 | 用途 |
| --- | --- | --- | --- |
| [f411ceu6](MDK-ARM/f411ceu6.uvprojx) | `0x08000000` | 384KiB | 独立 APP，直接通过调试器下载 |
| [f411_bootloader](MDK-ARM/f411_bootloader.uvprojx) | `0x08000000` | 32KiB | 蓝牙升级引导程序 |
| [f411ceu6_upgrade_app](MDK-ARM/f411ceu6_upgrade_app.uvprojx) | `0x08008000` | 352KiB | 与 Bootloader 配套的升级 APP，向量表偏移 `0x8000` |

最后128KiB（sector 7）保留给电压与复位历史；普通APP/升级APP及Bootloader均已限制边界，旧Bootloader须通过SWD同步更新。下载时使用按扇区擦除，整片擦除会清掉日志。

直接运行手表可选择独立 APP。部署 OTA 时，先通过 SWD 下载 Bootloader，再下载升级 APP 或进入 SPP 模式发送其 BIN；下载升级 APP 时保留 Bootloader 所在扇区。独立 APP 与 Bootloader 地址重叠，应按上表选择部署组合。

新版 Bootloader 还检查 EEPROM 中的 APP 有效标志。仅通过 SWD 下载升级 APP 不会自动建立此标志；首次部署或标志无效时，需要成功完成一次 SPP 升级，Bootloader 才会将镜像标为有效并允许正常启动。升级中断时会保留在升级模式，不能只凭向量表看似有效就启动未完成镜像。

## 获取与编译

克隆包含本次交付的分支：

```powershell
git clone --branch codex/low-power https://github.com/Mangta517/watch.git
cd watch
```

也可以直接[下载该分支的完整源码 ZIP](https://github.com/Mangta517/watch/archive/refs/heads/codex/low-power.zip)，解压后保持工程目录结构。

1. 安装 Keil MDK，并准备 **ARM Compiler 5.06 update 5（build 528）**；三个工程均配置此编译器。
2. 在 Pack Installer 中准备工程指定的 **Keil.STM32F4xx_DFP 3.0.0**。HAL、FreeRTOS 和 LVGL 源码已随仓库提供。
3. 按上表打开对应的 `.uvprojx`，选择同名 Target，执行 Rebuild。三个工程可以分别构建，无需先编译另一个工程。
4. 根据下面的输出位置取得镜像，再按上节选择独立 APP 或 Bootloader + 升级 APP 的部署组合。

| 工程 | AXF 输出 | 默认 HEX 输出 |
| --- | --- | --- |
| 独立 APP | `MDK-ARM/f411ceu6/f411ceu6.axf` | 同目录 `f411ceu6.hex` |
| Bootloader | `MDK-ARM/bootloader/f411_bootloader.axf` | 默认未开启 Create HEX File；需要时在 Keil Output 中勾选，或使用 fromelf 导出 |
| 升级 APP | `MDK-ARM/f411ceu6_upgrade_app/f411ceu6_upgrade_app.axf` | 同目录 `f411ceu6_upgrade_app.hex` |

SPP 升级使用升级 APP 的原始 BIN；fromelf 导出及 `tools/send_bootloader.py` 用法见 [Bootloader 说明](docs/bootloader.md#发送交互)。构建产物、编译日志和 Keil 界面缓存由 `.gitignore` 排除。

## 目录与文档

应用代码在 `application/`，硬件接口在 `interface/`，公共算法与软件 I2C 在 `common/`。

| 目录 / 文件 | 内容 |
| --- | --- |
| `MDK-ARM/` | 三个 Keil 工程、启动汇编和 RTE 配置 |
| `bootloader/` | Bootloader 入口及其独立启动流程 |
| `Core/`、`Drivers/` | MCU 初始化、中断和 STM32 HAL / CMSIS |
| `Middlewares/` | FreeRTOS 和 LVGL |
| `application/`、`interface/`、`common/` | 应用服务、器件接口和公共算法 |
| `docs/` | 功能、协议、部署和历史验证说明 |
| `tests/`、`tools/` | 主机回归与 SPP 升级发送脚本 |
| `f411ceu6.ioc` | CubeMX 配置 |

| 内容 | 文档 |
| --- | --- |
| 界面、低功耗、电池条和电量估算 | [docs/watch-ui.md](docs/watch-ui.md) |
| 电压历史记录、蓝牙导出与清空 | [docs/battery-log.md](docs/battery-log.md) |
| 蓝牙、校时和天气协议 | [docs/bluetooth.md](docs/bluetooth.md) |
| 启动流程与等待优化 | [docs/startup-optimization.md](docs/startup-optimization.md) |
| 运行诊断 | [docs/diagnostics.md](docs/diagnostics.md) |
| 复位 Flash 历史与 RTC / 天气状态查询 | [docs/reset-diagnostics.md](docs/reset-diagnostics.md) |
| BLE / SPP 升级与部署 | [docs/bootloader.md](docs/bootloader.md) |
| 看门狗与任务进度检查 | [docs/watchdog.md](docs/watchdog.md) |
| MPU 零偏及 EEPROM 保存 | [docs/imu-eeprom.md](docs/imu-eeprom.md) |
| 磁航向与校准 | [docs/magnetometer.md](docs/magnetometer.md) |
| 心率与故障诊断 | [docs/em7028.md](docs/em7028.md) |
| 温湿度 | [docs/aht21.md](docs/aht21.md) |

完整主机检查在项目根目录运行：

```powershell
python -B -X utf8 tests/clock/run.py
python -B -X utf8 tests/sensors/run.py
python -B -X utf8 tests/startup/run.py
python -B -X utf8 tests/ble/run.py
python -B -X utf8 tests/bootloader/run.py
python -B -X utf8 tests/battery_log/run.py
python -B -X utf8 tests/reset/run.py
python -B -X utf8 tests/reset_ble/run.py
python -B -X utf8 tests/tickless/run.py
python -B -X utf8 tests/touch/run.py
python -B -X utf8 tests/watchdog/run.py
python -B -X utf8 tests/diagnostics/run.py "$env:TEMP/watch-diagnostics.bin"
python -B -X utf8 tests/ui/run.py
```

需要 Python、桌面 GCC；UI 渲染还需要 Pillow。主机测试、Keil 编译和实板测试是三种不同证据，不能相互代替。

## 三工程打包与验证（2026-10-27）

本节保留 `80766c0` 的历史打包记录；当前64MHz版本的资源与验证结果见下方2026-10-28更新记录。

本次功能源码沿用已发布的 `1ade1e3`，在 `watch` 的 `codex/low-power` 分支统一交付三个 Keil 工程。本次文档整理保持固件源码、工程配置和功能版本不变。

- **问题**：三个工程的选择、构建环境、镜像输出和组合部署需要集中说明，避免取得源码后选错工程或镜像。
- **改动与优化作用**：`README.md` 补充完整源码获取方式、各工程入口、编译器与器件包版本、AXF / HEX 输出位置、目录索引和 13 套主机回归命令；沿用 Flash 分区、EEPROM 有效标志及 SPP BIN 部署说明，方便从完整源码复现构建并选择对应部署方式。
- **验证结果**：在本次交付源码的隔离工作区重新执行上述 13 套主机回归，全部通过；三个工程的源码引用无缺失，README 相对文件链接及 CRLF 空白检查通过。使用 ARMCC 5.06 update 5 全量重编译，结果如下。

| 工程 | 错误 | 警告 | MAP Total ROM Size | MAP Total RW Size（含 ZI） |
| --- | --- | --- | --- | --- |
| `f411ceu6` | 0 | 30 | 319748 B | 100056 B |
| `f411_bootloader` | 0 | 0 | 17736 B | 11360 B |
| `f411ceu6_upgrade_app` | 0 | 30 | 319748 B | 100056 B |

两个 APP 的 30 条警告均来自既有 LVGL 文件。构建结果满足各自链接区容量。

- **剩余问题**：本次未烧录或进行实板验证；功耗、触摸总线波形、连续 BLE / SPP 传输、EEPROM / Flash 掉电恢复和心率准确性仍以实板测试为准。历史版本中的功能限制继续保留在下面的更新记录中。

## 更新记录

以下日期取自 Git 提交日期。记录按新到旧排列，旧条目中的 4.2V 电量曲线等限制为当时状态，已由上述低功耗版本更新；各条目的验证结果仅适用于对应版本。

### 2026-10-28 · [146af34](https://github.com/Mangta517/watch/commit/146af34beb52b2eb2a95ae6f548c81d961b8eeb8) · 64MHz 低功耗与传感器错峰调度

- **问题**：需要同步本地后续低功耗配置；sensor 串行事务重叠及超期后整周期等待会拉长采样间隔，手机诊断缺少 touch 业务耗时和心率失败位置，首页停测后也无法保留最近有效心率。
- **改动与优化效果**：

| 模块 / 主要文件 | 改动与作用 |
| --- | --- |
| 时钟与休眠：`Core/Src/main.c`、`tim.c`、`FreeRTOSConfig.h`、`.ioc`、`app_tickless.*` | 同步 64MHz / Scale3 / Flash2 和 PWM 分频，软件 I2C、触摸位延时按当前主频调整；保留 BLE Tickless Sleep 和有条件 STOP，STOP 启用 Flash 掉电及稳压器低电压模式 |
| 外设与 GPIO：`Core/Src/adc.c`、`gpio.c`、`interface/lcd.c` | ADC 在 Sleep 停钟，LCD 休眠后停止 TIM3 并关闭显示外设时钟，亮屏先恢复时钟且保持背光为零至面板就绪；空闲 GPIO 使用模拟输入，推挽输出移除内部下拉，减少空闲外设持续运行 |
| sensor 与磁采集：`app_task.c`、`app_heart.*`、`app_imu.*`、`lsm303dlhc.c` | sensor 超期后只让出一拍；先服务 PPG，有事务时延后磁读取，MPU 继续读取。磁帧要求 DRDY=1 且 LOCK=0，排空 LOCK 残留帧不计成功；保留采样档位、一致性检查及新鲜度限制 |
| 诊断与首页：`app_diag.*`、`app_reset_ble.*`、`app_heart.*`、`app_ui.c` | DIAG 命令可独立开启四任务诊断，补齐 touch 业务计时；HEART,READ 及原诊断结果字段报告故障类型、寄存器和值，启动 / 停测保留原始故障；首页保留本次开机最近有效心率，详情和蓝牙仍用实时状态 |

- **验证结果**：在发布工作区运行 clock、sensors、startup、BLE、bootloader、battery_log、reset、reset_ble、tickless、touch、watchdog、diagnostics、UI 共 13 套主机回归，全部通过；UI 含诊断开 / 关，Tickless 含 STOP 开 / 关。覆盖 PPG / 磁错峰、磁新帧 / LOCK / 年龄、上一拍运动参考、超期与 tick 回绕、心率故障快照、首页历史值及显示停钟恢复。三个 Keil 目标使用 ARMCC 5.06 update 5 全量重编译，结果如下；两个 APP 的 13 条警告均来自本次未修改的 LVGL 源码。

| 工程 | 错误 | 警告 | MAP Total ROM Size | RAM（RW + ZI） |
| --- | --- | --- | --- | --- |
| `f411ceu6` | 0 | 13 | 220316 B | 104216 B |
| `f411_bootloader` | 0 | 0 | 17916 B | 15720 B |
| `f411ceu6_upgrade_app` | 0 | 13 | 220316 B | 104216 B |

- **剩余问题**：本次未烧录。真实待机电流、64MHz 软件总线时序、BLE 连续通信及 STOP 唤醒仍需实板验证。3 秒耗时模型得到 300 轮 sensor / 99 次 PPG 读取；独立 75Hz 转换模型在错峰时取得 174/225 帧磁数据，约 58Hz，不能据此声称实板稳定 100Hz 或磁帧全部采齐。PID=FF 的物理原因及上电恢复尚未解决，详见[诊断说明](docs/diagnostics.md)和[磁力计说明](docs/magnetometer.md)。

### 2026-10-28 · [3c9e533](https://github.com/Mangta517/watch/commit/3c9e53392cbce37c6833648d046ba9967b66214d) · 偶发手机同步失败与复位风险修复

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

### 2026-10-26 · [1ade1e3](https://github.com/Mangta517/watch/commit/1ade1e3cd72d39fad42bb99fe45c8caf15b5ca4d) · 复位现场追加保存到 Flash 与蓝牙历史查询

- **问题**：原复位现场仅保存在 RTC 备份寄存器，备份供电丢失或后续启动覆盖后难以对照多次异常；需要把复位标志和可取得的故障现场留存到 Flash。
- **改动与优化效果**：`application/app_battery_log.c/.h` 在每次启动满 10 秒且获得有效电压后保存一次复位标志、故障类型、PC 和喂狗时间等现场，复用 sector 7 的 17 个连续槽和一个共用开机标记；逐槽校验并最后提交，写满停写，不自动擦除。`application/app_reset_ble.c/.h` 增加 `RESET,HISTORY`，保留原 `RESET,READ`；`app_battery_log_ble.c` 跳过复位槽，使电压样本计数与导出保持兼容。同步更新相关测试与日志文档。已完整写入的历史可跨断电保留，旧电压数据无需迁移；代价是每次启动额外占用 136 字节现场空间，`BATLOG,CLEAR` 会同时清除两类历史。
- **验证结果**：在隔离工作区运行 battery_log、reset、reset_ble、startup、UI（调试开／关）主机套件通过，覆盖 72 个半字写入中断位置、残缺记录、旧电压兼容、备份域丢失、写满、锁竞争、异步发送和真实 UI 收发入口。实现阶段另有 bootloader 回归通过；同一业务源码的普通／升级 APP ARMCC 5.06 编译均为 0 错误，全量构建的 30 条既有警告均来自未修改 LVGL 文件，修改模块增量构建为 0 警告，ROM 均为 319748 字节（312.25 KiB）。电脑 BLE 清空后实读 `BATLOG,STATUS,0,0,16384,1`，仅证明当次清空与重连状态，不代表新历史功能已完成实板故障复现验证。
- **剩余问题**：异常后到下一次成功保存前仍依赖备份供电，完全掉电或早期连续复位可能丢失该次现场；不保存日历日期。尚需实板验证故障捕获、Flash 编程时序和手机通知完整性；复位标志和历史不能直接确定物理根因。

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
