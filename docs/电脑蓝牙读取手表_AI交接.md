# 电脑通过蓝牙读取 F411 手表：操作与 AI 交接

整理日期：2026-11-16。目的：让另一个 AI 在 Windows 上通过电脑蓝牙读取手表的状态、电压历史和复位历史，并保存、核验数据。

本文依据当前工作区协议代码及本机保留的读取脚本整理。本文编写期间没有扫描、连接手表或读取新数据；当前手表烧录的固件版本与在线状态需执行时确认。

## 1. 连接参数

| 项目 | 值 |
| --- | --- |
| 设备广播名称 | `F411-Watch` |
| 连接方式 | BLE GATT |
| 服务 UUID | `0000fff0-0000-1000-8000-00805f9b34fb` |
| 收发特征 UUID | `0000fff1-0000-1000-8000-00805f9b34fb` |
| 电脑发命令 | 向 FFF1 执行 Write Without Response，Bleak 参数为 `response=False` |
| 电脑收数据 | 先订阅同一个 FFF1 的 Notify，再发命令 |
| 命令编码 | ASCII，每条命令末尾发送真正的 LF 或 CRLF |
| 回包格式 | 本文查询命令返回 ASCII 文本行，以 CRLF 结束 |

普通 APP 默认关闭经典蓝牙 SPP，因此日常读取不走 COM 串口、RFCOMM 或配对后的“串口服务”。STM32 与蓝牙模块之间的 115200/8N1 是板内 UART 参数，电脑 BLE 客户端不设置串口波特率。不要硬编码历史 MAC 地址，先扫描确认设备。

## 2. 读取前准备

1. 打开电脑蓝牙，确认电脑具备 BLE 能力。
2. 断开手机及其他调试软件与手表的连接，关闭可能自动重连手表的手机应用。
3. 唤醒手表，退出 SPORT 运动模式。SPORT 会关闭蓝牙。
4. 当前源码支持在 `Settings → Bluetooth` 向下滚动，选择 `BLE: KEEP CONNECTED`。该选项只保存在 RAM，重启后恢复 `BLE: ECO / SCREEN OFF`。旧固件如无此选项，保持亮屏读取。
5. 等蓝牙页完成初始化后再连接；`INIT OK / BLE ONLY` 表示模块配置完成，不表示电脑已经连接或已订阅通知。

当前源码默认 ECO：熄屏后在业务空闲时关闭蓝牙。未收到合法校时时安静期约 1 秒，收到后约 30 秒；有日志导出、报告或在途收发时会延后关闭。长时间采集优先选择 KEEP CONNECTED，不依靠持续校时保连接。若实际固件较旧，以设备表现为准。

## 3. 本机已有读取工具

以下路径在整理文档时均已确认存在：

| 用途 | 路径 |
| --- | --- |
| Python | `D:\Python312\python.exe` |
| 已有读取脚本 | `C:\Users\27795\.codex\visualizations\2026\11\02\01a19a75-2a61-7c03-ad13-57a7788ac527\read_watch.py` |
| 已有依赖目录 | `C:\Users\27795\.codex\visualizations\2026\10\31\01a18fd0-0614-7962-9ebe-686d0aacc875\ble-deps` |

依赖目录内保留了 `bleak-3.0.2.dist-info` 和 WinRT 相关包。使用该目录即可复用本机环境；这不是要求安装某个最新版本。历史上这套脚本曾成功导出电压与复位历史，当前连接仍需重新验证。

已有 `read_watch.py` 的行为已直接检查：扫描名称 → 连接 → 订阅 Notify → 查询状态与历史 → 保存原始数据和 CSV → 校验数量 → 断开。脚本不发送清空、校时、天气同步或 OTA 命令。

### 在本机复用脚本

在 PowerShell 7 中执行下面的命令。先复制脚本到新的工作目录，输出就会保存在新目录，避免与历史采集混用。若另一个 AI 的环境限制写入位置，把 `$captureDir` 改为其允许写入的目录即可。

```powershell
$projectDir = 'D:\3251\f411ceu6\f411ceu6'
$sourceScript = 'C:\Users\27795\.codex\visualizations\2026\11\02\01a19a75-2a61-7c03-ad13-57a7788ac527\read_watch.py'
$dependencyDir = 'C:\Users\27795\.codex\visualizations\2026\10\31\01a18fd0-0614-7962-9ebe-686d0aacc875\ble-deps'
$captureDir = Join-Path $projectDir ('work\pc-ble-read-' + (Get-Date -Format 'yyyyMMdd-HHmmss'))
New-Item -ItemType Directory -Path $captureDir -Force | Out-Null
Copy-Item -LiteralPath $sourceScript -Destination (Join-Path $captureDir 'read_watch.py')
$env:PYTHONPATH = $dependencyDir
& 'D:\Python312\python.exe' -B -u (Join-Path $captureDir 'read_watch.py')
```

此命令中的 `PYTHONPATH` 只影响当前 PowerShell 进程及它启动的子进程；用完可关闭该终端。

脚本无命令行参数，按名称扫描 15 秒、连接超时 25 秒；每次发命令前等 1 秒。正常查询逐行接收超时为 20 秒，`HEART,READ` 为 5 秒；单次导出另有约 4000 秒总时长上限。20 秒指等待下一行的时限，不是整个电压导出只能运行 20 秒。

运行时会打印 `CONNECTED`、`SEND`、回包及周期性 `PROGRESS`。接近满日志的读取可能持续几十分钟，应保持进程运行。不要同时启动第二个蓝牙客户端或另一份读取脚本。

### 旧脚本的已知限制

- 名称匹配失败后直接报错，不会自动处理同名设备选择或无限重试；有多只同名手表时，先扫描并明确目标。
- `HEART,READ` 超时可继续；其他主要查询失败通常会退出。脚本不为 `BATLOG,ERROR,...` 或 `RESET,HISTORY,ERROR,...` 提前结束等待，应由执行 AI 查看原始回包判断。
- 原始二进制和文本接收文件持续写入；`summary.json` 在全部成功后才写出。没有 summary 或 `COMPLETE` 时，不能把已有文件当作完整导出。
- 脚本按文本行解析，适合本文的 ASCII 查询。读取前保持诊断流关闭、停止心率波形发送；若已有二进制遥测混入，应先结束该流，再重新采集。

## 4. 不依赖旧脚本时的实现流程

其他 AI 若无法访问上述本地脚本，可按以下协议重新实现电脑客户端，无需修改手表固件：

1. 使用 Bleak 或等价 BLE GATT 客户端扫描 `F411-Watch`，连接扫描所得设备对象。
2. 枚举服务，确认 FFF0 下的 FFF1 支持 Notify 和 Write Without Response。
3. 先订阅 FFF1。通知回调先保存原始字节，再加入接收缓冲；按 LF 拆行并去除行尾 CR。
4. 一次只发送一条命令，等对应结束标记收齐，再等待约 1 秒发下一条。不要把一组命令连续灌入。
5. 收到通知的一次回调可能只有半行，也可能含多行；必须跨回调累积，不能将一次 Notify 当作一条业务消息。
6. 所有本文查询命令加 LF 后均不超过 20 字节，可单次写入。若扩展成较长命令，分包间隔应小于 1 秒。
7. 保存原始数据，完成数量核验后输出摘要，最后取消订阅并断开连接。

仅查询状态和导出日志无需校时。排查时钟或天气问题时，先读取原状态，避免自动校时或同步天气改变现场。文本中的 `\n` 表示一个实际 LF 字节 `0A`；不要发送反斜杠和字母 n 两个字符，也不要添加引号或自定义封包头。

## 5. 命令顺序与完成条件

表中命令均需另加实际 LF/CRLF。

| 顺序 | 命令 | 内容 | 接收完成条件 |
| --- | --- | --- | --- |
| 1 | `STATE,READ` | RTC、校时接收计数、手机天气缓存 | 收到 `STATE,END`，通常共 4 行 |
| 2 | `RESET,READ` | 当前启动捕获的复位标志及上一启动现场 | 收到 `RESET,END`，通常共 5 行 |
| 3 | `HEART,READ` | 心率状态与故障快照 | 收到一行 `HEART,STATE,...`；旧固件可能不支持 |
| 4 | `BATLOG,STATUS` | 电压样本数、占用槽数、容量、存储状态 | 收到一行 `BATLOG,STATUS,...` |
| 5 | `RESET,HISTORY` | Flash 中的完整复位历史 | 收到 `RESET,HISTORY,END,N`，或明确 ERROR |
| 6 | `BATLOG,READ` | 请求时已有的有效电压历史 | 收到 `BATLOG,END,N`，或明确 ERROR |

不要在历史导出尚未完成时发送下一条查询。需要提前结束电压导出时，可以发送 `BATLOG,STOP` 并等待 `BATLOG,STOPPED`；它不删除历史。当前文本导出一般每行完成后至少间隔约 100ms，任务调度与无线传输还可能进一步延长耗时。

### 电压状态与导出

```text
BATLOG,STATUS,100,102,16384,1
```

字段依次为有效电压点数、已占用槽数、总容量、状态。状态：`0=STARTING`、`1=READY`、`2=FULL`、`3=NEED_CLEAR`、`4=ERROR`。占用槽还包含开机标记、复位现场和坏槽，不能拿它当电压样本数。FULL 或 ERROR 时可能仍能读出已保存的有效记录；NEED_CLEAR 不授权自动清空。

```text
BATLOG,BEGIN,3,5
BATLOG,DATA,1,1,60,3980
BATLOG,DATA,2,1,120,3975
BATLOG,DATA,4,2,60,3960
BATLOG,END,3
```

- `BEGIN`：本次请求的有效电压点数、占用槽数。
- `DATA`：`slot,boot_id,uptime_s,voltage_mv`。
- `END`：本次请求的期望有效点数。它不是电脑实际接收数量的证明。

完整性必须满足：`BEGIN 点数 = END 点数 = 实收 DATA 行数 = 唯一 slot 数`。每条 DATA 应恰有四个数值字段。slot 不要求连续，复位记录和开机标记会占槽。新追加的后台数据留到下一轮读取，因此不要用导出结束后的 STATUS 强行要求样本总数相等。

`uptime_s` 是对应 `boot_id` 内的开机经过秒数，电压单位 mV；日志没有日历时间。按开机编号分组分析，不补造关机间隔或最近一条记录之后的运行时间。传输核验完整，也不代表异常电压值具备物理可信度。

### 复位历史

```text
RESET,HISTORY,BEGIN,1,18,1
RESET,HISTORY,ENTRY,SLOT,1,BOOT,1
RESET,CAUSE,VALID,0,FLAGS,0x0E000000,KIND,0
RESET,TIME,UP_MS,0,FEED_MS,0,GAP_MS,0,TASK,0
RESET,FRAME,SP,0x00000000,EXC,0x00000000,PC,0x00000000,LR,0x00000000,XPSR,0x00000000
RESET,FAULT,CFSR,0x00000000,HFSR,0x00000000,MMFAR,0x00000000,BFAR,0x00000000
RESET,HISTORY,END,1
```

此处数值仅作格式示例。BEGIN 字段为完整现场数、占用槽数、存储状态；每个 ENTRY 后应有 CAUSE/TIME/FRAME/FAULT 四行，没有每条现场独立的 `RESET,END`。核对 BEGIN 数量、END 数量、ENTRY 数量相等，且每条 ENTRY 后四行完整。空历史也应有条数为 0 的 BEGIN 与 END。

复位 FLAGS 可以同时有多个位。只有 PINRSTF 不能区分看门狗拉复位、手动复位或其他外部电路；VALID=0、故障寄存器全零也不能证明没有硬件或供电问题。需保留原始值，结合对应固件符号与硬件证据分析。

## 6. 保存结果与验收

建议每次采集单独建目录，至少保存：

| 文件 | 内容 |
| --- | --- |
| `received.bin` | 全部 Notify 原始字节 |
| `received.txt` | 拼接后的文本行 |
| `battery.csv` | 表头为 `slot,boot_id,uptime_s,voltage_mv` 的电压记录 |
| `reset-history.txt` | 复位历史原始文本 |
| `summary.json` | 地址、采集时间、查询结果、导出计数与完整性结果 |

验收时确认电压四项计数一致、复位历史组数和字段完整、正常收到各结束标记，且已断开连接。本机旧脚本全部核验成功后会设置 `verified_complete: true` 并打印 `COMPLETE`。失败或被中断的文件保留为部分采集；重新读取另建目录，不直接拼接两次导出。

## 7. 常见问题

| 现象 | 处理 |
| --- | --- |
| 找不到手表 | 先唤醒、退出 SPORT、断开手机连接，等初始化结束后重新扫描 |
| 已连接但无回包 | 确认订阅的是 FFF1 Notify，写入也使用 FFF1、无响应写；检查命令末尾真实换行 |
| 很快断连 | 检查 ECO 熄屏策略；选择 KEEP CONNECTED 或保持亮屏 |
| 某条查询超时 | 保留原始回包，检查是否收到 ERROR；确认上一条已结束，等空闲后再单独重试 |
| 只有一半文本行 | 继续缓存等后续 Notify；若最终超时则报告部分导出 |
| DATA 条数不足或 slot 重复 | 本轮核验失败，重新完整 READ；不要只凭 END 判断成功 |
| 收到二进制内容 | 检查是否开启 `DIAG,1` 的 WDG1 遥测或心率 JustFloat；结束对应流后重新读取 |

当前源码默认 `APP_DEBUG_TELEMETRY=0`，不会自动发送心率波形；`DIAG,1` 可独立开启二进制诊断。本文只处理文本查询，不负责解码这些波形。若采集前发现诊断流已开启，可用 `DIAG,0` 加换行关闭发送。

## 8. 交给其他 AI 的任务说明

> 请按本文通过 Windows 电脑 BLE 读取 F411-Watch。先检查本机保留的 read_watch.py 和 ble-deps，优先复制已有只读脚本到本次输出目录后运行。连接 FFF0/FFF1，先订阅 Notify，再依次读取 STATE、RESET、HEART、BATLOG STATUS、RESET HISTORY 和 BATLOG READ。每条命令加实际换行，等待完整结束标记再进行下一条。保存原始二进制、文本、电压 CSV、复位历史和摘要；电压 BEGIN/END/实收条数/唯一 slot 数必须一致，复位历史也要核对组数和四行现场。完成后断开连接并报告输出绝对路径、数据是否完整及关键异常。若连接受熄屏影响，提示将手表设为 BLE KEEP CONNECTED。当前任务仅查询和导出；不要运行 clear_watch.py，不要发送 BATLOG,CLEAR、IMU,CAL、校时、天气或 OTA 命令，不要修改或烧录固件。BATLOG,CLEAR 会同时删除电压和复位历史并重启，需要用户另外明确要求才执行。

## 9. 当前工作区依据

项目路径：`D:\3251\f411ceu6\f411ceu6`，对应另一入口 `D:\32 51\f411ceu6\f411ceu6`。

- `interface/kt6368a.h`：广播名称、模块接口、收发约束。
- `application/app_ble_service.c`：协议分发、ECO/SPORT 策略、心率波形发送条件。
- `application/app_battery_log_ble.c`：BATLOG 命令、字段、导出节奏与结束格式。
- `application/app_reset_ble.c`：STATE、RESET、HEART、RESET HISTORY 文本协议。
- `application/app_diag.h`：默认关闭调试遥测。
- `docs/bluetooth.md`、`docs/battery-log.md`、`docs/reset-diagnostics.md`：协议与背景说明；其中历史日期章节可能描述旧固件，当前行为应结合代码与设备实测判断。

工作区原本已有大量未提交改动；本文仅新增交接文档，不把当前源码行为等同于手表已烧录行为。
