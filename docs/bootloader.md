# F411 主引导、独立保底与手机蓝牙 OTA

2026-11-02启用三个独立程序：主Bootloader读取EEPROM和按键后选择跳转目标；保底程序独立驻留，负责KT6368A、BLE/SPP接收、CRC和Flash更新；手表APP运行FreeRTOS/LVGL及传感器业务。沿用F407的分层职责，F411直接写内部Flash。

2026-11-04的SPP一键版本起，手机OTA采用“BLE控制＋SPP传输”：日常APP保持`T4+01`/`T5+00`，保底保持`T4+01`/`T5+01`。手机先在原主界面连接APP的BLE，打开OTA页选择APP BIN并点一次“开始升级”；APK经BLE探测或发送`OTA,START`，随后关闭BLE，按同一模块的SPP地址完成布局查询、整包发送和`ok,len:N`确认，再关闭SPP、等待APP启动并自动恢复原BLE。三程序分区、320KiB上限、CRC32、回读、向量和EEPROM提交规则不变。更早的BLE OTA与一键BLE版本说明保留为历史记录。

KT6368A手册V2.0第9-10页说明：SPP地址是BLE地址最高字节加1，AT串字节序与Android显示相反。手册的“最高字节”指AT串中的最高字节；Android显示顺序相反，所以换算落在Android首字节。例：Android BLE `2C:A2:C8:B1:0C:22` 对应SPP `2D:A2:C8:B1:0C:22`。APK把Android地址首字节加1、其余五字节不变，用户不需要另选SPP名称或地址。BLE与SPP不是同一个地址，文档和界面不能把同一串同时写成两种地址。

一键APK为`work/spp-oneclick-20261104/delivery/Bluetooth-Wave-SPP-OTA-OneClick.apk`，版本`1.95-wave.19-spp-oneclick` / code24，包名与签名沿用上一版`1.95-wave.18-oneclick` / code23，可直接覆盖安装。本版重建保底BIN（`0x08008000`）和升级APP BIN（`0x08010000`）；主引导`0x08000000`未要求变化，可继续使用现有BIN。交付只有三个BIN，不含HEX。

## 当前分区与工程

| 程序 | Keil工程 / Target | Flash范围（含末地址） | 容量 / 扇区 |
| --- | --- | --- | --- |
| 主Bootloader | `MDK-ARM/f411_bootloader.uvprojx` / `f411_bootloader` | `0x08000000..0x08007FFF` | 32KiB / sector 0..1 |
| 独立保底程序 | `MDK-ARM/f411_recovery.uvprojx` / `f411_recovery` | `0x08008000..0x0800FFFF` | 32KiB / sector 2..3 |
| 升级手表APP | `MDK-ARM/f411ceu6_upgrade_app.uvprojx` / `f411ceu6_upgrade_app` | `0x08010000..0x0805FFFF` | 320KiB / sector 4..6 |
| 电压与复位日志 | APP管理 | `0x08060000..0x0807FFFF` | 128KiB / sector 7 |
| 独立普通APP | `MDK-ARM/f411ceu6.uvprojx` / `f411ceu6` | `0x08000000..0x0805FFFF` | 384KiB，单独使用 |

普通APP占用主引导和保底所在区域，不能与三程序部署同时使用。手机OTA只能发送升级APP的原始BIN，最大327680字节。旧版`0x08008000`链接的APP及352KiB容量配置已停用，不能继续发送到本版设备。

布局定义为`common/boot_layout.h`。Keil链接配置和手机端也使用上表边界，由回归检查一致性。升级APP的向量偏移为`0x10000`，保底程序的向量偏移为`0x8000`，两个APP继续使用原`*_flash_safe.sct`内的SRAM日志编程段。

## 启动与保底

```mermaid
flowchart TD
    A[复位：主Bootloader] --> B{KEY_1复位期间长按满2秒?}
    B -->|是| P{独立保底向量有效?}
    B -->|否| C{APP向量有效且EEPROM读成功?}
    C -->|否| P
    C -->|是| D{标志以BL 42 4C 开头?}
    D -->|是| Q{标志精确为 42 4C A5 5A?}
    D -->|否：空白或遗留首装| R[跳转0x08010000运行手表]
    Q -->|是| R
    Q -->|否：升级中/畸形/撕裂| P
    P -->|是| E[跳转0x08008000运行保底]
    P -->|否| F[保持供电并停留，等待SWD维护]
    E --> K[保底开启BLE与SPP，等待手机]
    K --> H[仅擦写sector 4..6中本镜像覆盖的范围]
    H -->|长度 CRC 回读 向量及EEPROM提交成功| R
    H -->|接收或写入失败| A
```

主引导入口为`bootloader/boot_main.c`，选择逻辑为`application/app_boot_select.c`，不链接蓝牙驱动或固件擦写模块。正常启动只读EEPROM，不写标志，也不会引入无条件开机延时。复位后PA5（KEY_1，空闲高、按下低）若为高立即进入正常判定；只有检测到按下才每10ms轮询，复位期间连续按住满2秒进入独立保底，任何提前松开都视为正常启动。APP向量有效且EEPROM读取成功时，若标志不以`42 4C`（BL）开头（空白或首次SWD下载APP的遗留记录）直接启动APP；若以BL开头则只有精确`42 4C A5 5A`才启动APP，`42 4C 5A A5`升级中、畸形或撕裂标志均进入独立保底。APP向量无效或EEPROM读取失败同样进入独立保底。

保底入口为`bootloader/recovery_main.c`，复用`application/app_bootloader.c`、`interface/int_bootloader.c`。INIT → RUN → REC_DATA → CHECK_DATA → JUMP_TO_APP状态机在保底区运行，升级失败会复位回主引导，再由主引导重新进入保底。保底没有安装或向量损坏时，主引导不会跳转未知代码，该情况需通过SWD恢复保底。

两者共用`bootloader/boot_platform.c`，最先拉高PA3保持电源，使用HSI PLL 100MHz，并在跳转时关闭SysTick、清NVIC、设置目标VTOR/MSP。交接不调用会复位GPIO的`HAL_DeInit()`。保底内的UART/DMA由保底自己停止。

## EEPROM与APP进入升级

板上BL24C02F使用PA12=SCL、PA11=SDA、7位地址`0x50`。`0xC0..0xC3`保存四字节状态：

| 内容 | 含义 |
| --- | --- |
| `42 4C 5A A5` | 升级中，主引导进入独立保底 |
| `42 4C A5 5A` | APP有效，仍须通过APP向量检查 |
| 不以`42 4C`开头（空白或首装遗留） | 主引导在APP向量有效时直接启动APP，允许SWD首装后免手机升级启动 |
| 以`42 4C`开头但非上述精确值（畸形/撕裂） | 进入独立保底 |
| EEPROM读取失败或APP向量无效 | 进入独立保底 |

校准`0x00..0x9F`、运行时间`0xA0..0xBF`、新天气记录`0xD0..0xEF`维持现有布局。旧`0xC0`天气记录只读兼容，不再写入。

升级APP从唯一BLE业务入口接收`OTA,START`加实际LF/CRLF。sensor任务等其他EEPROM保存结束后写入并读回升级中标志，LVGL回传`OTA,REBOOTING,SPP`；更早的APP回传`OTA,REBOOTING,BLE`，APK两种都接受，因此已部署的旧APP无需为这次改版更新也能被一键流程接手。回执只表示请求已受理，是否进入保底仍以随后SPP的严格布局为准。UART完成后留500ms发送窗口再软件复位；蓝牙断开不撤销已接受的完整请求；回执未完成时最多等2秒后仍复位。EEPROM失败返回`OTA,EEPROM_ERROR`，不执行软件复位。普通独立APP返回`OTA,UNAVAILABLE,INSTALL_BOOTLOADER_AND_UPGRADE_APP`。

保底在合法start命令后，先持久化升级中标志，成功才擦除APP；完整镜像、CRC和向量提交成功后，再写入并读回APP有效标志。升级中断后重新发送完整镜像，不续传、不回滚。首次通过SWD部署主引导、保底和APP后，EEPROM空白或遗留非BL记录即可直接启动有效APP，无需先完成一次手机升级；手机升级只用于后续更新及在EEPROM建立标准有效标志。

## 手机操作与首次部署

一键APK包名为`xnj.lazydog.btcontroller.wave.ota`，签名与code23相同，可直接覆盖安装并保留原波形、诊断、收发和导出入口。日常业务仍走BLE；只有OTA在保底阶段改用SPP。原来的主连接路由把双模设备固定按BLE处理，纯经典设备仍按经典处理；OTA页显式建立RFCOMM，不受这条日常路由影响。

### 日常一键升级（已完成部署）

1. 在原主界面按平时方式连接手表BLE（`F411-Watch`，FFF0/FFF1）。不需要先在安卓系统蓝牙设置里配对手表，也不需要手动让手表进入升级模式。
2. 打开“手表固件升级 · OTA”，选择升级APP BIN（链接到`0x08010000`，不超过320KiB）。APK核对MSP/Reset并计算CRC；日常只需选这一次。
3. 点一次“开始升级”，然后等待。APK自动经BLE探测手表是否已在保底；若仍在普通APP，则发送一次`OTA,START`并接受受理回执。随后APK关闭BLE，按同一模块的SPP地址建立RFCOMM，严格核对`OTA,LAYOUT,08010000,00050000`，发送`start:长度:CRC32`，收到ready后单次发送完整BIN，收到精确`ok,len:N`，关闭SPP，等待APP启动并恢复原BLE。
4. 首次安全RFCOMM连接可能由Android弹出系统配对确认或PIN，允许一次即可。后续日常升级不需要每次重新配对、手动进入升级模式、重新扫描或修改设置。
5. 进度100%只表示字节已交给协议栈；只有设备回传匹配长度的`ok,len:N`才算CRC、长度、回读、向量和EEPROM提交全部通过。收到确认后恢复原BLE失败只报告连接问题，不回滚、不重发固件，固件仍算成功。
6. 打开OTA页时若没有当前主连接，或手表已在保底（例如KEY_1进入），允许在OTA页选择目标一次；之后仍走同一自动流程。

### 首次部署与迁移（按需）

- **首次工厂部署**：用SWD写入三个BIN。手机只发送APP BIN，不能写主引导或保底。

| BIN | SWD起始地址 | 容量 | 说明 |
| --- | --- | --- | --- |
| `F411-OTA-Bootloader.bin` | `0x08000000` | 32KiB | 主引导，仅SWD可写入；本版未要求变化，可复用现有BIN |
| `F411-OTA-Recovery.bin` | `0x08008000` | 32KiB | 本版保底，BLE＋SPP OTA接收端 |
| `F411-OTA-Upgrade-App.bin` | `0x08010000` | 320KiB | 升级APP，手机OTA唯一可下发的镜像 |

- **已刷仅BLE保底（code22/code23）**：手机OTA不能写保底，必须用SWD只把本版`F411-OTA-Recovery.bin`写到`0x08008000`一次。主引导`0x08000000`本版未要求变化，可继续使用现有BIN。旧APP回传的`OTA,REBOOTING,BLE`仍被APK接受，所以保底就绪后APP可继续经手机OTA更新。
- **现有保底仍兼容SPP OTA且布局一致**：不强制SWD；覆盖安装APK后按日常流程升级APP即可。
- 不要把这理解成“每次升级都必须重刷三个BIN”；三个BIN只用于首次部署或上述保底迁移。

### 一键流程内部行为（排障用）

普通用户按上面的步骤操作即可，以下细节用于排障和协议核对：

- 点“开始升级”时APK锁定本次目标（BIN与当前手表地址），释放原主连接，改用OTA页自己的BLE控制连接。
- BLE阶段只用于探测`OTA,INFO`和发送一次`OTA,START`，不发送BIN正文。无论探测结果显示已在保底还是普通APP，打开SPP前都先关闭BLE。
- SPP阶段把Android BLE地址首字节加1、其余五字节不变，作为同一模块的SPP地址；用户不需要另选SPP名称。RFCOMM使用标准SPP UUID `00001101-0000-1000-8000-00805f9b34fb`。
- SPP连接后发送`OTA,INFO`并严格核对`OTA,LAYOUT,08010000,00050000`；布局不符或设备不识别时不发送start。核对通过后发送`start:长度:CRC32`，收到ready才单次发送BIN。
- BIN按256字节外层块组织，块间40ms，最后一块之后不再补暂停；SPP是字节流，没有BLE的20字节单包限制。
- 只有设备回传匹配长度的`ok,len:N`才算成功。成功后APK关闭SPP，等待APP启动，再恢复原BLE；恢复失败不回退固件，也不触发再次升级。
- 停止、切后台、断连或SPP写失败会结束本次传输，不续传、不自动整轮重试；重新点“开始升级”会从头发送完整BIN。未完成接收时保底按“30秒无进展”复位后重新等待，指连续30秒收不到有效数据，不是整轮升级限30秒。

## 字节流协议与擦写范围

1. 手机发送`OTA,INFO\n`。保底用同一条USART通道回`OTA,LAYOUT,...`，因此手机必须在对应连接上查询：APK先用BLE探测是否已在保底，关闭BLE进入SPP后再严格核对`OTA,LAYOUT,08010000,00050000`。布局不符或设备不识别时不发送start。
2. 手机发送`start:十进制长度:8位CRC32\n`。CRC为CRC-32/ISO-HDLC，与Python `zlib.crc32(BIN)`一致。保底继续兼容原`start:长度`调试方式，无CRC模式会警告，畸形CRC字段不会降级。SPP可一次写入整条命令，不要求拆成BLE的20字节片。
3. 等待`Flash erased, ready to receive data...`后发送原始BIN。不能添加HEX文本、换行、包头或包尾；接收阶段的新start会被当成正文。
4. 完整接收并处理后留100ms保护窗口，检查多收字节、CRC、回读和向量；确认后回传`ok,len:N`，留500ms发送窗口再跳APP。错误回传后复位。

### SPP承载与命令分帧

日常APP关闭SPP，保底同时开BLE和SPP。手机OTA的控制阶段走BLE，正文阶段走SPP；两者不能同时用于发送BIN。

- 保底把ASCII命令（`OTA,INFO`、`start:长度:CRC32`）以LF立即结算；不带LF时按1000ms接收空闲判定命令结束。`start:...`在SPP上可一次写完，不再有20字节分片问题。
- BIN仍按256字节外层块组织，块间保留40ms暂停，最后一块之后不再补暂停。
- SPP写成功只代表本机socket已接受，不代表手表收到，更不代表固件有效；只有设备回传的`ok,len:N`才是CRC、长度、回读、向量和EEPROM提交全部通过的确认。
- 首次安全RFCOMM连接可能触发Android系统配对确认/PIN，允许一次即可；日常升级不要求每次手动配对或进入系统设置。

| APP长度 | 擦除扇区 | 覆盖范围 |
| --- | --- | --- |
| 8..65536字节 | sector 4 | `0x08010000..0x0801FFFF` |
| 65537..196608字节 | sector 4..5 | `0x08010000..0x0803FFFF` |
| 196609..327680字节 | sector 4..6 | `0x08010000..0x0805FFFF` |

最后镜像地址使用`APP_START_ADDR + length - 1`，边界长度不会多擦一个扇区。只在接收前擦除一次；短镜像范围以外的旧APP尾部可以保留。主引导、独立保底及日志均不在OTA可擦写范围内。

DMA2 Stream5/Channel4使用8192字节循环缓冲和HT/TC标记，主循环复制到512字节工作缓冲后按半字写内部Flash。首8字节向量暂存，长度、CRC、回读和向量通过后才提交，MSP最后写；奇数末字节补FF，不计入BIN长度或CRC。UART/DMA错误、缓冲覆盖或Flash失败会终止本轮。

保底不运行FreeRTOS、LVGL和传感器任务，屏蔽TPS3823的RST，不启用内部IWDG；接收超时由状态机处理，执行停滞没有看门狗复位。CRC仅用于检错，没有镜像签名或A/B回滚。

## 构建与验证

使用Keil ARMCC 5.06，分别构建主引导、保底和升级APP三个工程，普通APP仍可单独构建。输出目录分别为`MDK-ARM/bootloader/`、`MDK-ARM/recovery/`、`MDK-ARM/f411ceu6_upgrade_app/`。手机源码及APK重建脚本见[android-ota](../tools/android-ota/README.md)。

```powershell
python -X utf8 tests/bootloader/run.py
python -X utf8 tests/ota/run.py
python -X utf8 tests/battery_log/run.py
```

主机执行真实选择逻辑、保底状态机、蓝牙驱动、EEPROM及Flash接口，覆盖空白/遗留首装直接启动APP、精确有效标志、升级中/畸形/撕裂BL标志、EEPROM读取失败、无效APP向量、缺失或错误保底向量的跳转。新增`tests/bootloader/boot_entry_harness.c`包含真实`boot_main.c`的`boot_recovery_requested`辅助函数，用自定义`entry_host/main.h`最小HAL桩验证空闲高电平立即返回、提前松开、1990ms松开、连续按住满2000ms及uint32 tick回绕判定。完整320KiB镜像、奇偶长度、DMA回绕、CRC损坏、写后读回和掉电标志路径保持不变，模拟擦写模型拒绝sector 0..3及7，完整镜像后确认主引导、保底和日志内容保留。

构建、主机和模拟器证据不代替实板验证。本版新增或重建的APK、Recovery BIN和Upgrade APP BIN由父级实际构建、签名核对和测试后另行记录；本文不据旧版本证据声明新产物已通过测试、已烧录或已完成手机实测。首次部署、实际SPP连接（含首次系统配对）、BLE控制回执、Flash时序、断连后的完整重传与跳转仍需实机确认。上述主机测试清单是现有测试入口，不表示本版已重新运行。

上一版本（BLE OTA code22、一键BLE code23）的主机测试、Keil构建和签名记录属于历史证据，只说明当时版本，不自动适用于本版。2026-11-02三程序版本的Keil链接结果（主引导6008字节、保底18492字节、升级APP223660字节）同样仅供追溯。
