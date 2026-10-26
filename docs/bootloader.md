# F411 蓝牙 SPP 升级

沿用 F407 的应用层状态：INIT → RUN → REC_DATA → CHECK_DATA → JUMP_TO_APP。推荐命令为 `start:十进制字节数:8位CRC32`，兼容原 `start:十进制字节数`。设备先擦除、开启 DMA、提示 ready，再接收原始 BIN；收满声明长度且处理完毕后等待 100ms 保护窗口，再核对长度、CRC 和向量并跳转。尚未收满时，连续 30s 无新字节则失败复位，包括 ready 后零字节的情况。

## 工程与分区

| 内容 | Keil 工程 / Target | Flash 地址 | 容量 |
| --- | --- | --- | --- |
| 独立 Bootloader | `MDK-ARM/f411_bootloader.uvprojx` / `f411_bootloader` | `0x08000000`～`0x08007FFF`，sector 0～1 | 32KiB |
| 正常手表 APP | `MDK-ARM/f411ceu6.uvprojx` / `f411ceu6` | `0x08000000`～`0x0805FFFF` | 384KiB |
| 供升级使用的手表 APP | `MDK-ARM/f411ceu6_upgrade_app.uvprojx` / `f411ceu6_upgrade_app` | `0x08008000`～`0x0805FFFF`，sector 2～6 | 352KiB |

sector 7（`0x08060000～0x0807FFFF`，128KiB）独立保留给[电压历史记录](battery-log.md)。2026-10-23起普通APP、升级APP链接区及Bootloader最大镜像边界均止于`0x08060000`；发送脚本上限同步为360448字节。**旧Bootloader必须先通过SWD更新**，旧版仍允许擦写sector 7。调试器下载需选择按覆盖扇区擦除；整片擦除会删除日志。

正常 APP 和升级 APP 使用上表两个独立 `.uvprojx` 工程，每个工程只保留自己的一个 Target。两者共用全部源码和业务配置，仅区分链接起始地址、可用 Flash 长度和对应向量表偏移；输出目录分别为 `MDK-ARM/f411ceu6/` 和 `MDK-ARM/f411ceu6_upgrade_app/`，避免相互覆盖。正常 APP 从 `0x08000000` 独立启动；升级 APP 的 `USER_VECT_TAB_ADDRESS,VECT_TAB_OFFSET=0x8000` 让 `Core/Src/system_stm32f4xx.c` 设置后移的向量表。

正常 APP 直接烧录使用，其地址与 Bootloader 重叠，不能和 Bootloader 同时保留。使用蓝牙升级时，单独编译烧录 Bootloader，再打开 `f411ceu6_upgrade_app.uvprojx` 编译并发送 BIN；也可以通过调试器将 Bootloader 与升级 APP 分别烧录。调试器使用按覆盖扇区擦除，不能在下载升级 APP 时整片擦除 Bootloader。Bootloader 不链接 LVGL、FreeRTOS 或传感器模块。

正常上电时，EEPROM 标志为 APP 有效且 APP 向量合法才直接进入手表；**按住 KEY_1（PA5，低电平）上电或复位**仍会停留升级模式。APP 尚未安装、标志空白/损坏/读取失败或上次升级未完成时也停留升级模式。进入后可松开 KEY_1，等待蓝牙初始化并重新连接 SPP。升级模式不运行手表 UI。

当前 Bootloader 不增加运行时 `boot` 命令，进入方式使用现有按键。普通手表的时间、天气、心率和原有蓝牙接收逻辑保持原样。

2026-10-22蓝牙策略：普通APP与升级APP始终仅BLE（`T4+01`、`T5+00`），只有上述Bootloader升级分支才初始化并开启SPP（`T5+01`，BLE仍开启）。正常启动直接跳APP，不会先开启SPP。升级成功返回APP后，APP启动配置重新关闭SPP；因此SPP连接会断开，手机需改为重新连接BLE。SPP使能属于模块记忆参数，变化后由现有驱动重新上电并验证，已匹配时不重复写入。

## EEPROM 启动标志（2026-10-24）

复用板上 BL24C02F（7位地址 `0x50`、PA12=SCL、PA11=SDA），占用 `0xC0–0xC3` 四字节，格式为 `42 4C 状态 状态反码`。校准记录 `0x00–0x9F` 和开机时长记录 `0xA0–0xBF` 保持原布局。

| 四字节内容 | 含义 | 正常上电行为 |
| --- | --- | --- |
| `42 4C 5A A5` | 升级中 | 留在 Bootloader，重新发送完整镜像 |
| `42 4C A5 5A` | APP 有效 | 同时通过 APP 向量检查且未按住 KEY_1 时跳转 |
| 空白或其他内容 | 未初始化或损坏 | 留在 Bootloader |

收到合法 start 后，先写入“升级中”并读回比较，成功才擦除 APP；EEPROM 缺失、写保护、通信失败或读回不符均中止，不能继续擦除。收到完整镜像并通过原长度、CRC（若声明）、回读和向量检查，提交 Flash 向量并锁定 Flash 后，才写入“APP 有效”并读回确认；成功才返回 `ok,len:N`。最终标志保存失败走原失败复位路径。升级期间掉电或复位后按持久化标志重新判断，不自动续传。

普通启动只读取，不反复写入 EEPROM；启动时仍核对向量，不重新计算整包 CRC。Bootloader 使用 HAL 毫秒计时等待 EEPROM 就绪（10ms 上限），不依赖 FreeRTOS；APP 继续使用原任务等待方式。每次访问结束仍释放 EEPROM 引脚。

**首次换用本版 Bootloader，原有或通过 SWD 单独下载的 APP 不会自动取得有效标志。** 需要成功执行一次本版蓝牙升级；仅下载 APP 或仅写合法向量不足以允许启动。EEPROM 故障时保持升级模式，需先恢复 EEPROM 通信。此规则不增加 APP 运行时升级命令。

## 按程序大小决定擦除范围

APP 起始地址固定，**程序大小改变的是最后一个擦除扇区和擦除数量**。最后一个镜像地址为 `APP_START_ADDR + length - 1`；不能用 `APP_START_ADDR + length`，否则恰好到边界时会多擦一个扇区。

| APP 长度（字节） | 擦除 sector | 覆盖 Flash |
| --- | --- | --- |
| 8～16384 | 2 | `0x08008000`～`0x0800BFFF` |
| 16385～32768 | 2～3 | `0x08008000`～`0x0800FFFF` |
| 32769～98304 | 2～4 | `0x08008000`～`0x0801FFFF` |
| 98305～229376 | 2～5 | `0x08008000`～`0x0803FFFF` |
| 229377～360448 | 2～6 | `0x08008000`～`0x0805FFFF` |

长度不足 8 字节、超过 360448 字节、负数、溢出或非数字均拒绝，拒绝时不擦除。只在本次接收前擦除一次覆盖范围，写入过程中不补擦。旧镜像比新镜像大时，覆盖范围以外的旧尾部保留，APP 不会从该尾部启动。

## 发送交互

1. 打开 **`MDK-ARM/f411ceu6_upgrade_app.uvprojx` 工程**编译，导出原始 BIN；读取 BIN 的实际文件字节数。不要发送 HEX 文本、AXF 或正常 APP 工程的 `0x08000000` 链接镜像。
2. 在升级模式连接模块的经典蓝牙 SPP（PC 上通常表现为蓝牙串口）。沿用模块原有 SPP 名称，BLE 名称仍为 `F411-Watch`；现有驱动已查询并配置 SPP 开启。
3. 发送 `start:文件字节数:CRC32`。CRC 使用 CRC-32/ISO-HDLC，与 Python `zlib.crc32(BIN)` 一致，字段恰好 8 位十六进制、不带 `0x`，大小写均可。兼容旧 `start:文件字节数`，但设备会提示 `Warning: legacy transfer without CRC`，该模式不能检出等长正文损坏。命令可以带 CR/LF，也兼容无换行后空闲 10ms 结束；命令阶段使用原蓝牙逐字节中断接收。
4. 设备返回 `Received start command, program length: N len`。继续等待完整的 `Flash erased, ready to receive data...`，期间不能发送固件。大镜像擦除多个扇区需要更久，主机的 ready 等待时间应覆盖这一阶段。
5. 原样发送 BIN；不能添加换行、文本编码、包头或包尾。SPP 任意分包不影响字节流写入；未收满时允许超过 2 秒的停顿，但连续 30s 没有新字节会超时。接收阶段的新 `start:` 会被当成 BIN，不能用它取消或重启本次传输。
6. 收满并处理完数据后等待 100ms，期间多收字节仍拒绝；通过长度、CRC（若声明）和向量校验后返回 `ok,len:N`，留 500ms 让模块发送回执，再进入 APP。失败返回 `app rec error,system reset; reconnect and send start again`，等待复位、重新连接后，再发 start、等 ready、发 BIN。500ms 是模块发送窗口，不表示已经得到手机收件确认。

项目附带主机脚本，使用本机 `pyserial`（缺少时执行 `python -m pip install pyserial`）：

```powershell
python tools/send_bootloader.py COM5 MDK-ARM/f411ceu6_upgrade_app/f411ceu6_upgrade_app.bin
```

将 COM5 替换为实际 SPP 串口。脚本先核对 BIN 长度和 MSP/Reset 地址，再计算 CRC、发送命令、等待 ready，最后发送原始 BIN 并等待匹配长度的 ok；不会自动重试或烧录调试器。主机侧向量检查可以在发送 start 前拒绝明显的错误镜像，设备侧仍会在最终提交前复查。脚本不自动回退到无 CRC 协议；旧 Bootloader 需先通过 SWD 更新。

例如，在项目根目录的 PowerShell 中导出本机 Keil 编译得到的 APP：

```powershell
& 'D:\32 51\51\ARM\ARMCC\bin\fromelf.exe' --bin --output 'MDK-ARM\f411ceu6_upgrade_app\f411ceu6_upgrade_app.bin' 'MDK-ARM\f411ceu6_upgrade_app\f411ceu6_upgrade_app.axf'
(Get-Item -LiteralPath 'MDK-ARM\f411ceu6_upgrade_app\f411ceu6_upgrade_app.bin').Length
```

固件侧保持 USART1 PA9/PA10、115200、8N1，没有自行提升模块 UART 波特率；SPP 的有效吞吐仍受这段 UART 限制。

## 中断、DMA 和校验

- 升级数据使用 DMA2 Stream5 / Channel4、8192 字节 circular 缓冲，前后两个 4096 字节半区交替接收。与 F407 一样使用 HT/TC 完成标记，属于 circular 的两个半区，不是硬件 DBM 模式。屏幕原有 DMA2 Stream2 不变。
- UART/DMA ISR 只更新半区结束位置、回绕计数或错误计数。主循环复制到 512 字节工作缓冲后写 Flash；不足半区的尾部在接收位置稳定 10ms 后处理。
- 收到超长数据、DMA 缓冲覆盖、UART/DMA 错误、擦除/编程失败或回读不一致时中止升级，不跳过丢失数据继续顺序写入。
- 按半字编程，支持任意分包及奇数总长度。最后一个奇数字节补 `0xFF` 写入，补齐字节不计入镜像长度。
- 先暂存最前面的 MSP/Reset 向量，镜像长度、写入回读、CRC（若声明）和向量范围检查通过后才提交；MSP 最后写。CRC覆盖完整BIN，包含前8字节和最后奇数字节，不包含补齐的FF；CRC失败时不提交向量，也不再擦除扇区。这是单 APP 方案，start 后旧版本已擦除，不提供回滚。CRC用于检错，不提供来源认证或防回滚。
- 旧 `start:len` 保持可用，并明确提示未启用整包CRC；残缺、非十六进制或位数不对的CRC字段会在擦除前拒绝，不降级到旧协议。
- 升级时屏蔽 TPS3823 的 RST 并让 WDI 保持高阻，不启动内部 IWDG；升级期间没有看门狗保护，接收超时仍由状态机按30s处理，执行停滞无法由看门狗复位。跳转保持 PA3 电源，不执行会复位全部 GPIO 的 `HAL_DeInit()`。

DMA 请求映射和 Flash 扇区布局依据 [ST STM32F411 参考手册 RM0383](https://www.st.com/resource/en/reference_manual/DM00119316-.pdf)。

## 验证

`python -X utf8 tests/bootloader/run.py` 在 Windows 上使用模拟 Flash/DMA 执行真实 `kt6368a.c`、`int_bootloader.c`、`app_bootloader.c`。工程预检确认当前使用自动生成scatter，并检查真正的链接区 `OCR_RVCT4`，不要求器件内存字段 `IROM.Size` 等于分区容量。运行用例覆盖擦除边界、完整352KiB镜像及sector 7日志保留、奇偶长度、DMA回绕/延迟TC、长度及外设错误、命令分片、旧协议、zlib参考CRC、等长内容损坏、零CRC、CRC字段拒绝、100ms完成保护、零字节/缺包30s超时、超过2s暂停、tick回绕及失败复位；并执行主机发送脚本的模拟交互测试。`tests/ble/run.py` 验证普通 APP 的原蓝牙接收路径。

同一主机测试执行真实 `bl24c02.c` 的 Bootloader 分支，模拟底层 I2C，覆盖空白/损坏/有效标志、读取失败、10ms就绪超时及tick回绕、写保护、部分写入、先持久化再擦除、最终标志保存失败与重新升级，以及其他EEPROM地址内容保留。实板需再验证：正常升级后掉电重启进入APP，升级中断电后留在Bootloader并可重新升级；记录保留及总线时序以实板结果为准。

Keil 编译和主机模拟不能证明实板 SPP 连续接收、Flash 编程期间的 DMA 时序或跳转后的手表运行；这些仍需按上述交互上板验证。未自动烧录或交付 HEX。
