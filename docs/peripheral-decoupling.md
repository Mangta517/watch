# 外设层解耦

2026-11-13 工作区实现：采用 P1～P4 的组合，按已有模块职责收拢依赖。新增代码不分配动态内存，不增加任务。

| 职责 | 当前归属 |
| --- | --- |
| Bootloader、Recovery、APP 和日志分区 | `common/boot_layout.h`，日志起点直接引用 APP 结束地址 |
| USART1 完成/错误事件 | `kt6368a` 提供可选接管入口，`int_bootloader` 在 DMA 启动前绑定；启动失败或停止成功后解除，停止失败保留接管 |
| Flash 编程期间 UART 暂存 | `common/ram_uart_poll.h` 强制内联到 `battery_log_ram`；Flash 退出 BSY 后才递交接收回调 |
| 日志与蓝牙的连接 | `app_battery_log_init()` 绑定日志接收回调；日志驱动不依赖 `kt6368a.h` |
| STOP 外设资格 | LCD、蓝牙、电池 ADC 驱动各自查询硬件；`app_tickless` 汇总屏幕状态、总开关和驱动结果 |
| 电池单次 ADC 转换 | `interface/battery_adc.c`，保留启动、有限超时轮询、读值、停止及原始 HAL 错误诊断 |
| 返回键和唤醒键电平 | `interface/watch_key.c`，PA5 低有效、PA4 高有效；UI 继续管理消抖和长按 |
| 传感器生命周期 | `app_imu`、`app_baro`、`app_heart` 负责芯片初始化、恢复、休眠和零偏入口；`app_task` 继续安排总线占用及各服务的调度 |
| 驱动延时的操作系统适配 | `common/platform_delay.c`，七个驱动不再包含 FreeRTOS/task 头文件 |

`platform_delay_ms()` 保留普通任务等待，供 EEPROM ACK 轮询和兼容阻塞校准使用。`platform_delay_min_ms()` 保留硬件最短等待额外一拍的处理，供传感器初始化、LCD、触摸和看门狗使用。调度器未运行或构建 Bootloader/Recovery 时使用 HAL 延时。EEPROM 超时统一用 HAL 毫秒计时，保持 10ms 上限。

电池服务仍向快照和诊断发布具体 HAL 错误码，因此 ADC 窄接口保留 `HAL_StatusTypeDef`。IMU 快照和 EEPROM 零偏记录继续复用既有驱动数据类型；本次没有更改记录布局或迁移已有数据。

STOP 的硬件条件保持背光为零、SPI 空闲、TX DMA 未使能、蓝牙电源输出锁存器为低、USART1 时钟关闭、ADC 未使能。查询仍在原来的关中断区域执行。时钟恢复、RTC 补时、看门狗暂停授权、异常捕获和中断向量入口保持原流程。

验证包括现有主机回归、四个 Keil ARMCC5 目标，以及两套 APP 的 MAP 和反汇编检查。主机模型与构建不能代替真实 BLE/SPP OTA、写日志时串口并发、STOP 唤醒和电流测量。交付 BIN 基于当时完整工作区，包含此前尚未提交的业务改动；没有烧录或推送。
