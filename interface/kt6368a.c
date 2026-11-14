/**
 ******************************************************************************
 * @file    kt6368a.c
 * @brief   KT6368A驱动。USART1 ISR入队/重启接收，达到水位后回调应用。
 ******************************************************************************
 */
#include "kt6368a.h"
#include "usart.h"
#include <string.h>

/* 普通APP与保底恢复均开启BLE；仅保底恢复额外开启SPP，日常APP关闭SPP。 */
#if defined(F411_BOOTLOADER)
#define KT6368A_SPP_VALUE   "01"
#define KT6368A_SPP_COMMAND "AT+B501\r\n"
#else
#define KT6368A_SPP_VALUE   "00"
#define KT6368A_SPP_COMMAND "AT+B500\r\n"
#endif

typedef struct
{
    uint8_t rx_byte;
    uint8_t rx_buffer[KT6368A_RX_BUFFER_SIZE];
    uint32_t rx_time_ms[KT6368A_RX_BUFFER_SIZE];
    uint8_t rx_flags[(KT6368A_RX_BUFFER_SIZE + 3) / 4]; /* 每槽两位，接收入队路径写；任务轮询入队时屏蔽中断。 */
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    volatile uint32_t rx_dropped;
    volatile uint32_t rx_errors;
    volatile uint32_t rx_activity_ms;
    volatile uint8_t rx_have_byte;
    volatile uint8_t rx_loss_pending; /* 此前发生过队列丢字节或接收错误；空队列也要上报LOSS边界。 */
    volatile uint8_t powered;
    uint32_t power_on_tick;
} kt6368a_t;

static kt6368a_t ble;
static HAL_StatusTypeDef init_status = HAL_ERROR;
static void (*s_rx_callback)(void);
static kt6368a_rx_override_fn s_rx_override;

void kt6368a_set_rx_override(kt6368a_rx_override_fn callback)
{
    s_rx_override = callback;
}

uint8_t kt6368a_is_stop_idle(void)
{
    return (BLE_EN_GPIO_Port->ODR & BLE_EN_Pin) == 0 &&
           (RCC->APB2ENR & RCC_APB2ENR_USART1EN) == 0;
}

enum
{
    KT_START_OFF,
    KT_START_DISCHARGE,
    KT_START_BOOT,
    KT_START_QUERY,
    KT_START_REPLY,
    KT_START_SAVE,
    KT_START_RETRY
};

typedef struct
{
    uint32_t phase_ms;
    uint32_t rx_errors;
    uint32_t rx_dropped;
    char line[64];
    uint8_t length;
    uint8_t overflow;
    uint8_t phase;
    uint8_t query;
    uint8_t changed;
    uint8_t verifying;
    uint8_t attempts;
} kt6368a_start_struct;

static kt6368a_start_struct s_start;
/* 0空闲、1发送中、2结果待取；ISR只发布完成结果，不调用RTOS接口。 */
static volatile uint8_t s_tx_state;
static uint8_t s_uart_disabled;
static volatile HAL_StatusTypeDef s_tx_result;
static uint32_t s_tx_started_ms, s_tx_timeout_ms;

/* 从接收环形队列取出最多size字节；只推进读指针tail，不等待新数据。 */
uint16_t kt6368a_read(uint8_t *data, uint16_t size)
{
    /* 本次已取出的字节数，同时作为返回值。 */
    uint16_t count = 0;
    if (data == NULL)
    {
        /* 目的缓冲无效：返回0且不移动tail，数据仍留在队列中。 */
        return 0;
    }
    /* head由ISR推进、tail由读取方推进；两者不等表示队列中还有未读字节。 */
    while ((count < size) && (ble.rx_tail != ble.rx_head))
    {
        /* 先保证读到ISR已发布的槽位，再读取该槽数据，避免取到尚未写完的字节。 */
        __DMB();
        /* 取当前tail处字节并递增局部计数，槽位在循环末尾才归还。 */
        data[count++] = ble.rx_buffer[ble.rx_tail];
        /* 数据已复制到调用方缓冲，再保证后续tail推进不会被提前重排。 */
        __DMB();
        /* tail加1并按缓冲大小回绕；ISR此后可复用该槽。 */
        ble.rx_tail = (uint16_t)((ble.rx_tail + 1) % KT6368A_RX_BUFFER_SIZE);
    }
    /* 返回实际取出的字节数，0表示当前没有可读数据或size为0。 */
    return count;
}

/* 读取某槽位在rx_flags中记录的2位边界标志。 */
static uint8_t kt6368a_rx_flags(uint16_t slot)
{
    /* 每4个槽共用1字节、每槽占2位：先定位字节，再右移到本槽起始位，最后取低2位。 */
    return (uint8_t)((ble.rx_flags[slot / 4] >> ((slot % 4) * 2)) & 3);
}

/* 带时间/边界信息的接收：同一到达tick且无边界标志的连续字节作为一块返回。 */
uint16_t kt6368a_read_timed(uint8_t *data, uint16_t size, kt6368a_rx_info_struct *info)
{
    /* 本次已复制字节数，同时作为返回值。 */
    uint16_t count = 0;
    /* 遍历用槽位索引；每复制一槽就推进ble.rx_tail。 */
    uint16_t slot;
    /* 保存进入临界区前的PRIMASK，退出时原样恢复中断开关。 */
    uint32_t mask;
    if (data == NULL || info == NULL || size == 0)
    {
        /* 参数不完整：不读队列，也不改写info。 */
        return 0;
    }
    /* 空队列的活动时间和丢字节状态必须来自同一份ISR快照。 */
    mask = __get_PRIMASK();
    /* 短临界区读取队列状态，防止ISR在判断head/tail之间插入新字节。 */
    __disable_irq();
    if (ble.rx_tail == ble.rx_head)
    {
        /* 队列为空：上报最近一次UART活动时刻，供调用方计算空闲超时。 */
        info->received_ms = ble.rx_activity_ms;
        /* 记录本次检查时刻，避免用检查之后的时间结算残帧。 */
        info->sampled_ms = HAL_GetTick();
        info->flags = 0;
        if (ble.rx_loss_pending != 0)
        {
            /* 此前发生过队列丢字节或接收错误，空队列也要把LOSS边界交给调用方。 */
            info->flags = KT6368A_RX_LOSS;
        }
        /* 恢复进入前的中断状态后返回0。 */
        __set_PRIMASK(mask);
        return 0;
    }
    /* 队列非空：先恢复中断，后面的逐字节复制不长时间屏蔽ISR。 */
    __set_PRIMASK(mask);
    /* 从当前读指针开始，先把首字节的时间戳和边界标志作为本块基准。 */
    slot = ble.rx_tail;
    /* 确保先看到ISR已写入的槽内容，再读取时间戳和标志。 */
    __DMB();
    info->received_ms = ble.rx_time_ms[slot];
    info->flags = kt6368a_rx_flags(slot);
    /* 逐槽复制，直到填满size、追上新head，或遇到新的时间/边界分界。 */
    while (count < size && slot != ble.rx_head)
    {
        /* 每槽读取前确认ISR已发布该槽数据。 */
        __DMB();
        if (count != 0 && (ble.rx_time_ms[slot] != info->received_ms || kt6368a_rx_flags(slot) != 0))
        {
            /* 首字节之后出现不同到达tick或空闲/丢字节边界：本块到此结束，留给下次调用。 */
            break;
        }
        /* 复制该槽字节并推进局部计数。 */
        data[count++] = ble.rx_buffer[slot];
        /* 槽位加1并回绕，继续看下一槽。 */
        slot = (uint16_t)((slot + 1) % KT6368A_RX_BUFFER_SIZE);
        /* 确认本槽已复制完成，再把tail推进到下一槽，ISR可回收该槽。 */
        __DMB();
        ble.rx_tail = slot;
    }
    /* 返回本块实际复制的字节数。 */
    return count;
}

/* 丢弃当前队列中的全部未读字节，并清除空闲/丢字节状态。 */
void kt6368a_flush_rx(void)
{
    /* 保存原中断状态，临界区结束后恢复。 */
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    /* 直接把tail追到head：已入队但未读的字节全部作废。 */
    ble.rx_tail = ble.rx_head;
    /* 清除“已收过字节”标记，下一次到达会重新产生空闲边界。 */
    ble.rx_have_byte = 0;
    /* 清除尚未上报的丢字节标记。 */
    ble.rx_loss_pending = 0;
    /* 恢复进入前的中断状态。 */
    __set_PRIMASK(mask);
}

/* 查询接收队列中是否有未读字节。 */
uint8_t kt6368a_rx_pending(void)
{
    /* head与tail不等即表示队列非空，返回1。 */
    return ble.rx_head != ble.rx_tail;
}

/* 注册水位到达时由UART ISR调用的通知回调。 */
void kt6368a_set_rx_callback(void (*callback)(void))
{
    /* 只保存函数指针，NULL表示不通知。 */
    s_rx_callback = callback;
}

/* 返回累计丢弃字节数（队列满时每次丢弃加1）。 */
uint32_t kt6368a_get_rx_dropped(void)
{
    return ble.rx_dropped;
}
/* 返回累计UART接收错误次数。 */
uint32_t kt6368a_get_rx_errors(void)
{
    return ble.rx_errors;
}
/* 返回最近一次初始化/分步启动的最终状态。 */
HAL_StatusTypeDef kt6368a_get_init_status(void)
{
    return init_status;
}

/* 本板未接KT_LED状态脚，接口固定返回0表示“无法确认连接”。 */
uint8_t kt6368a_is_connected(void)
{
    /* KT_LED 未接到 STM32，保留接口：0 表示无法确认连接，不能用心率 PB15 推断。 */
    return 0;
}

/* 断电：中止在途发送、关闭USART1、释放UART引脚并拉低使能脚。 */
void kt6368a_power_off(void)
{
    /* 复用同一份GPIO初始化参数，把PA9/PA10切为模拟模式。 */
    GPIO_InitTypeDef gpio = {0};
    /* 保存原中断状态，供中止发送的临界区恢复。 */
    uint32_t mask = __get_PRIMASK();
    /* 清供电标志后，发送接口据此返回BUSY；read/read_timed仍返回已读字节数。 */
    ble.powered = 0;
    __disable_irq();
    if (s_tx_state == 1)
    {
        /* 正在异步发送：中止USART1发送，避免断电后完成中断再改状态。 */
        (void)HAL_UART_AbortTransmit(&huart1);
        /* 把结果定为失败，并置为“结果待取”，由发起者通过take_tx_result收走。 */
        s_tx_result = HAL_ERROR;
        s_tx_state = 2;
    }
    /* 恢复中断，后续DeInit等较慢操作不再屏蔽中断。 */
    __set_PRIMASK(mask);
    if (s_uart_disabled == 0)
    {
        /* 先停掉在途接收，再整体反初始化USART1。 */
        (void)HAL_UART_AbortReceive(&huart1);
        /* MSP DeInit关闭USART1时钟和中断；PA3/PA4仍使用GPIOA，不能关闭整口时钟。 */
        (void)HAL_UART_DeInit(&huart1);
        /* 记录UART已关闭，下次上电需要重新HAL_UART_Init。 */
        s_uart_disabled = 1;
    }
    /* BL1551B关断时将蓝牙电源接地；UART闲置高电平不能继续驱动模块。 */
    gpio.Pin = BLE_TX_Pin | BLE_RX_Pin;
    /* 模拟模式断开复用功能，避免经引脚向已断电模块反向供电。 */
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);
    /* 拉低BL1551B使能脚，切断模块供电。 */
    HAL_GPIO_WritePin(BLE_EN_GPIO_Port, BLE_EN_Pin, GPIO_PIN_RESET);
    /* 丢弃断电前残留的接收数据。 */
    kt6368a_flush_rx();
}

/* 内部上电流程：恢复UART、使能电源、配置引脚并启动一字节中断接收。 */
static HAL_StatusTypeDef kt6368a_power_enable(void)
{
    /* PA9/PA10复用为USART1的参数。 */
    GPIO_InitTypeDef gpio = {0};
    /* 保存HAL_UART_Init/Receive_IT的返回状态。 */
    HAL_StatusTypeDef status;
    if (ble.powered)
    {
        /* 已上电则直接成功，不重复初始化外设。 */
        return HAL_OK;
    }
    /* init调用位置保证UART已初始化；保留CubeMX生成的115200/8N1配置。 */
    if ((huart1.Instance != USART1) || (huart1.Init.BaudRate != 115200))
    {
        /* 句柄不是USART1或波特率被改过，拒绝上电以免与模块通信参数不符。 */
        return HAL_ERROR;
    }
    if (s_uart_disabled != 0)
    {
        /* 上次断电关闭过UART：重新初始化USART1外设。 */
        status = HAL_UART_Init(&huart1);
        if (status != HAL_OK)
        {
            /* 初始化失败则反初始化，保持“UART关闭”的一致状态并返回原错误。 */
            (void)HAL_UART_DeInit(&huart1);
            return status;
        }
        /* 初始化成功，清除“UART已关闭”标记。 */
        s_uart_disabled = 0;
    }
#if !defined(F411_BOOTLOADER)
    if (s_rx_callback != NULL)
    {
        /* MSP初始化把USART1设为0；通知任务前改为12，满足FreeRTOS的ISR优先级约束。 */
        HAL_NVIC_SetPriority(USART1_IRQn, 12, 0);
    }
#endif
    /* 置高BL1551B使能脚，给蓝牙模块供电。 */
    HAL_GPIO_WritePin(BLE_EN_GPIO_Port, BLE_EN_Pin, GPIO_PIN_SET);
    /* 记录上电时刻，供启动等待和发送前检查使用。 */
    ble.power_on_tick = HAL_GetTick();
    /* PA9/PA10切回USART1复用推挽输出/输入。 */
    gpio.Pin = BLE_TX_Pin | BLE_RX_Pin;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &gpio);
    /* 清除可能残留的溢出错误标志，避免刚开接收就进错误回调。 */
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    /* 清空断电期间/引脚切换期间可能收到的无效数据。 */
    kt6368a_flush_rx();
    /* 先置供电标志，再启动中断接收。 */
    ble.powered = 1;
    /* 启动一字节中断接收，后续字节在RxCplt回调里续接。 */
    status = HAL_UART_Receive_IT(&huart1, &ble.rx_byte, 1);
    if (status != HAL_OK)
    {
        /* 启动接收失败：回滚为断电状态并返回原错误。 */
        kt6368a_power_off();
        return status;
    }
    /* UART与电源均已就绪。 */
    return HAL_OK;
}

/* 阻塞式上电入口：先放电20ms，再上电并等待模块启动。 */
HAL_StatusTypeDef kt6368a_power_on(void)
{
    /* 保存上电结果，决定是否继续等待启动时间。 */
    HAL_StatusTypeDef status;
    if (ble.powered != 0)
    {
        /* 已上电直接成功，不重复延时。 */
        return HAL_OK;
    }
    /* 先保持断电20ms，让模块电源充分放电后再上电。 */
    HAL_Delay(20);
    /* 执行实际的上电与接收启动。 */
    status = kt6368a_power_enable();
    if (status == HAL_OK)
    {
        /* 上电成功后再等模块启动完成，之后才允许发AT命令。 */
        HAL_Delay(KT6368A_BOOT_DELAY_MS);
    }
    /* 返回上电结果；失败时不额外延时。 */
    return status;
}

/* 阻塞发送：检查长度、发送状态与模块启动时间后调用HAL_UART_Transmit。 */
HAL_StatusTypeDef kt6368a_write(const uint8_t *data, uint16_t size, uint32_t timeout_ms)
{
    if ((data == NULL) || (size == 0) || (size > KT6368A_TX_MAX_SIZE))
    {
        /* 空指针、空数据或超过单次上限：直接返回错误。 */
        return HAL_ERROR;
    }
    if (s_tx_state == 1 || s_start.phase != KT_START_OFF || !ble.powered ||
        ((uint32_t)(HAL_GetTick() - ble.power_on_tick) < KT6368A_BOOT_DELAY_MS))
    {
        /* 有异步发送在途、启动状态机占用UART、未供电或模块仍在启动等待期，均视为忙。 */
        return HAL_BUSY;
    }
    /* 直接把调用方缓冲交给HAL阻塞发送；返回值只代表UART发送结果。 */
    return HAL_UART_Transmit(&huart1, (uint8_t *)data, size, timeout_ms);
}

/* 异步发送：登记起始时刻和超时，启动中断发送后立即返回。 */
HAL_StatusTypeDef kt6368a_write_async(const uint8_t *data, uint16_t size, uint32_t timeout_ms)
{
    /* 保存HAL_UART_Transmit_IT的启动结果。 */
    HAL_StatusTypeDef status;
    if (data == NULL || size == 0 || size > KT6368A_TX_MAX_SIZE || timeout_ms == 0)
    {
        /* 参数非法或没有超时值：返回错误，不启动发送。 */
        return HAL_ERROR;
    }
    if (s_tx_state != 0 || s_start.phase != KT_START_OFF || !ble.powered ||
        (uint32_t)(HAL_GetTick() - ble.power_on_tick) < KT6368A_BOOT_DELAY_MS)
    {
        /* 发送中/结果未取、启动状态机占用、未供电或模块启动未完成：返回忙。 */
        return HAL_BUSY;
    }
    /* 记录发起时刻，供take_tx_result判断是否超时。 */
    s_tx_started_ms = HAL_GetTick();
    s_tx_timeout_ms = timeout_ms;
    /* 先发布“发送中”，再启动中断发送，避免完成中断先到而状态未置位。 */
    s_tx_state = 1;
    status = HAL_UART_Transmit_IT(&huart1, (uint8_t *)data, size);
    if (status != HAL_OK)
    {
        /* 启动失败：撤销发送中状态，调用方不会收到完成结果。 */
        s_tx_state = 0;
    }
    /* 返回启动结果；HAL_OK只表示发送已启动，不代表已发完。 */
    return status;
}

/* 查询发送是否仍在途或结果尚未被收取。 */
uint8_t kt6368a_tx_is_busy(void)
{
    /* 0空闲、1发送中、2结果待取；两种非零状态都返回忙。 */
    return s_tx_state != 0; /* 完成结果仍由发起者收取，其他异步写入继续等待。 */
}

/* 收取异步发送结果：必要时先判超时并中止发送，成功取到结果时清空状态。 */
uint8_t kt6368a_take_tx_result(HAL_StatusTypeDef *result)
{
    /* 进入时取一次tick，临界区内不再重复取，保证超时判断一致。 */
    uint32_t now = HAL_GetTick();
    /* 保存原中断状态，退出临界区时恢复。 */
    uint32_t mask = __get_PRIMASK();
    /* 1表示本次已取到结果，0表示尚无结果。 */
    uint8_t ready = 0;
    __disable_irq();
    /* 只中止TX，保持原RX接收；保护完成中断与超时判定的交接。 */
    if (s_tx_state == 1 && (uint32_t)(now - s_tx_started_ms) >= s_tx_timeout_ms)
    {
        /* 已超过登记的超时时间：中止发送，并把结果记为超时、状态改为待取。 */
        (void)HAL_UART_AbortTransmit(&huart1);
        s_tx_result = HAL_TIMEOUT;
        s_tx_state = 2;
    }
    if (s_tx_state == 2)
    {
        /* 结果已就绪：把成功/失败/超时原样交给调用方，并清回空闲。 */
        *result = s_tx_result;
        s_tx_state = 0;
        ready = 1;
    }
    /* 恢复中断后返回是否取到结果；返回0时*result未被写入。 */
    __set_PRIMASK(mask);
    return ready;
}

/* USART1发送完成中断回调：把结果发布为待取。 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart->Instance == USART1 && s_tx_state == 1)
    {
        /* 仅处理本模块发起且仍在途的发送，避免覆盖他人状态。 */
        s_tx_result = HAL_OK;
        /* 状态2表示完成结果等待发起者收取。 */
        s_tx_state = 2;
    }
}

/* 阻塞式AT命令：发送一行命令并等待以expected_prefix开头的响应行。 */
HAL_StatusTypeDef kt6368a_at_command(const char *command, const char *expected_prefix, char *response, uint16_t response_size,
                                     uint32_t timeout_ms)
{
    /* 逐字节拼装的单行响应缓冲，不含CRLF。 */
    char line[64];
    /* 当前行已累计的字符数。 */
    uint16_t length = 0;
    /* byte为本次读到的字节；too_long标记该行已超出line容量。 */
    uint8_t byte, too_long = 0;
    /* start为发送前记录的超时起点，涵盖发送与响应等待；errors/dropped为发送前的计数基准。 */
    uint32_t start, errors, dropped;
    /* 命令与期望前缀的长度，用于长度校验和前缀比较。 */
    size_t command_size, prefix_size;
    /* 保存发送及参数校验过程的返回状态。 */
    HAL_StatusTypeDef status;

    if ((command == NULL) || (expected_prefix == NULL) || (response == NULL) || (response_size == 0) || (timeout_ms == 0))
    {
        /* 任一指针为空或容量/超时为0：参数无效，返回错误。 */
        return HAL_ERROR;
    }
    /* 先把响应缓冲置为空串，未匹配到响应时调用方读到的是空内容。 */
    response[0] = '\0';
    /* 取命令和前缀长度。 */
    command_size = strlen(command);
    prefix_size = strlen(expected_prefix);
    if ((command_size < 7) || (command_size > 63) || (prefix_size == 0) || (strncmp(command, "AT+", 3) != 0) ||
        (strcmp(command + command_size - 2, "\r\n") != 0))
    {
        /* 命令长度不在7~63、不以AT+开头或结尾不是CRLF，视为非法命令。 */
        return HAL_ERROR;
    }
    /* 防止一次调用注入多条AT命令。 */
    if ((memchr(command, '\r', command_size - 2) != NULL) || (memchr(command, '\n', command_size - 2) != NULL))
    {
        /* 结尾CRLF之前的正文里仍含CR或LF：拒绝，避免拼出第二条命令。 */
        return HAL_ERROR;
    }
    if (!ble.powered || s_tx_state == 1)
    {
        /* 模块未供电或已有异步发送在途：返回忙。 */
        return HAL_BUSY;
    }
    /* 清空旧接收数据，避免把上一条命令的响应当成这次的应答。 */
    kt6368a_flush_rx();
    /* 记录发送前的错误与丢字节计数，等待期间一旦变化即判失败。 */
    errors = ble.rx_errors;
    dropped = ble.rx_dropped;
    /* 记录发送时刻，作为等待超时的起点。 */
    start = HAL_GetTick();
    /* 阻塞发送整条命令；timeout_ms同时作为本次发送的超时。 */
    status = HAL_UART_Transmit(&huart1, (uint8_t *)command, (uint16_t)command_size, timeout_ms);
    if (status != HAL_OK)
    {
        /* 发送失败直接返回HAL错误状态，不再等待响应。 */
        return status;
    }

    /* 在超时时间内轮询接收，逐字节拼行并匹配前缀。 */
    while ((uint32_t)(HAL_GetTick() - start) < timeout_ms)
    {
        if ((ble.rx_errors != errors) || (ble.rx_dropped != dropped))
        {
            /* 等待期间发生接收错误或队列丢字节：响应可能已损坏，返回错误。 */
            return HAL_ERROR;
        }
        if (kt6368a_read(&byte, 1) == 0)
        {
            /* 暂时没有新字节，继续下一轮轮询直到超时。 */
            continue;
        }
        if (byte == '\n')
        {
            /* 换行表示一行结束，先检查是否因过长被截断。 */
            if (too_long)
            {
                /* 本行超出line容量，无法可靠比较，返回错误。 */
                return HAL_ERROR;
            }
            /* 给当前行补字符串结束符，得到不含CRLF的整行。 */
            line[length] = '\0';
            if (strncmp(line, "ER+", 3) == 0)
            {
                /* 模块明确回ER+错误行：返回HAL_ERROR。 */
                return HAL_ERROR;
            }
            if (strncmp(line, expected_prefix, prefix_size) == 0)
            {
                /* 行首匹配期望前缀：再确认调用方缓冲放得下整行加结束符。 */
                if (length >= response_size)
                {
                    /* 响应超过调用方缓冲容量，返回错误而不是截断复制。 */
                    return HAL_ERROR;
                }
                /* 连同结束符一起复制整行，返回成功。 */
                memcpy(response, line, length + 1);
                return HAL_OK;
            }
            /* 不匹配的行丢弃，长度清零后重新拼下一行。 */
            length = 0;
        }
        else if (byte != '\r')
        {
            /* 非CR/LF的普通字符：容量够就追加，否则只标记溢出。 */
            if (length < sizeof(line) - 1)
            {
                line[length++] = (char)byte;
            }
            else
            {
                /* 预留1字节给结束符，已满则不再写入，仅记录溢出。 */
                too_long = 1;
            }
        }
    }
    /* 超时前未收到匹配行，返回HAL_TIMEOUT。 */
    return HAL_TIMEOUT;
}

/* 先查后改；设置响应可能被CR00关闭，因此依据后续查询和重新上电验证结果。 */
static HAL_StatusTypeDef kt6368a_set_if_needed(const char *query, const char *prefix, const char *value, const char *setting,
                                               uint8_t *changed)
{
    /* 保存查询响应整行，含前缀，不含CRLF。 */
    char response[64];
    /* 查询/设置过程的HAL状态。 */
    HAL_StatusTypeDef status;
    /* 先查询当前值。 */
    status = kt6368a_at_command(query, prefix, response, sizeof(response), KT6368A_AT_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        /* 查询失败（含ER+/超时/接收错误）：直接返回，不盲目写配置。 */
        return status;
    }
    /* 跳过响应行前缀，只比较其后的数值部分；相同则无需写入。 */
    if (strcmp(response + strlen(prefix), value) == 0)
    {
        return HAL_OK;
    }
    /* 当前值与目标不同：阻塞发送设置命令。 */
    status = HAL_UART_Transmit(&huart1, (uint8_t *)setting, (uint16_t)strlen(setting), KT6368A_AT_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        /* 发送失败直接返回，不置changed。 */
        return status;
    }
    /* 等待模块处理设置命令；这里不解析设置命令的回显。 */
    HAL_Delay(100);
    /* 通知调用方本次确实写过参数，后续需要重新上电并复查。 */
    *changed = 1;
    /* 写入动作本身未报错即返回成功，实际生效由调用方验证。 */
    return HAL_OK;
}

/* 阻塞式配置：设置BLE名称、打开BLE、按构建设置SPP，并复查。 */
HAL_StatusTypeDef kt6368a_configure(const char *name)
{
    /* setting为拼好的AT+BM名称设置命令；response保存每次查询的整行。 */
    char setting[40], response[64];
    /* 传入名称的字节长度。 */
    size_t name_size;
    /* 0表示三项均未改动，1表示至少写过一项，需要重新上电验证。 */
    uint8_t changed = 0;
    /* 各步骤HAL状态。 */
    HAL_StatusTypeDef status;
    if (name == NULL)
    {
        /* 名称为空指针：参数无效。 */
        return HAL_ERROR;
    }
    name_size = strlen(name);
    if ((name_size == 0) || (name_size > 30) || (strchr(name, '\r') != NULL) || (strchr(name, '\n') != NULL))
    {
        /* 名称须为1~30字节且不含CR/LF，否则会破坏AT命令格式。 */
        return HAL_ERROR;
    }
    if (!ble.powered)
    {
        /* 模块未供电：无法配置，返回忙。 */
        return HAL_BUSY;
    }

    /* 先查BLE名称，模块无响应时返回；保留SPP名称、UART波特率和MAC。 */
    /* 按"AT+BM" + 名称 + CRLF 的顺序拼设置命令。 */
    memcpy(setting, "AT+BM", 5);
    memcpy(setting + 5, name, name_size);
    memcpy(setting + 5 + name_size, "\r\n", 3);
    /* 查询TM+，不一致则写入刚拼好的BLE名称命令。 */
    status = kt6368a_set_if_needed("AT+TM\r\n", "TM+", name, setting, &changed);
    if (status != HAL_OK)
    {
        /* 名称查询/设置失败：终止配置。 */
        return status;
    }
    /* 查询T4+，要求为01（开启BLE）；不同则发AT+B401。 */
    status = kt6368a_set_if_needed("AT+T4\r\n", "T4+", "01", "AT+B401\r\n", &changed);
    if (status != HAL_OK)
    {
        /* BLE开关设置失败：终止配置。 */
        return status;
    }
    /* 查询T5+，普通APP要求SPP为00（关闭），保底恢复要求01（开启）。 */
    status = kt6368a_set_if_needed("AT+T5\r\n", "T5+", KT6368A_SPP_VALUE, KT6368A_SPP_COMMAND, &changed);
    if (status != HAL_OK)
    {
        /* SPP开关设置失败：终止配置。 */
        return status;
    }
    if (!changed)
    {
        return HAL_OK; /* 三项查询均匹配，无需重复验证 */
    }
    /* 有参数被改写：断电再上电，让模块以新参数重新初始化。 */
    kt6368a_power_off();
    status = kt6368a_power_on();
    if (status != HAL_OK)
    {
        /* 重新上电失败：返回上电错误，无法继续验证。 */
        return status;
    }
    /* 复查BLE名称：查询TM+并把整行放进response。 */
    status = kt6368a_at_command("AT+TM\r\n", "TM+", response, sizeof(response), KT6368A_AT_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        /* 查询失败：参数是否生效无法确认，返回原状态。 */
        return status;
    }
    if (strcmp(response + 3, name) != 0)
    {
        /* 跳过3字节前缀"TM+"后与目标名称不符：配置未生效。 */
        return HAL_ERROR;
    }
    /* 复查BLE开关：查询T4+。 */
    status = kt6368a_at_command("AT+T4\r\n", "T4+", response, sizeof(response), KT6368A_AT_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        /* 查询失败：无法确认BLE是否开启。 */
        return status;
    }
    if (strcmp(response, "T4+01") != 0)
    {
        /* 整行不是T4+01：BLE未按要求开启。 */
        return HAL_ERROR;
    }
    /* 复查SPP开关：查询T5+。 */
    status = kt6368a_at_command("AT+T5\r\n", "T5+", response, sizeof(response), KT6368A_AT_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        /* 查询失败：无法确认SPP状态。 */
        return status;
    }
    if (strcmp(response, "T5+" KT6368A_SPP_VALUE) == 0)
    {
        /* SPP状态与目标值一致：配置全部验证通过。 */
        return HAL_OK;
    }
    /* SPP状态仍不匹配：返回错误。 */
    return HAL_ERROR;
}

/* 初始化期间AT独占接收队列，UI通过is_pending暂停业务收发。 */
void kt6368a_init_begin(void)
{
    /* 先断电并复位接收统计，保证每次启动从干净状态开始。 */
    kt6368a_power_off();
    ble.rx_dropped = 0;
    ble.rx_errors = 0;
    /* 清空分步启动状态机的行缓冲、计数和阶段。 */
    memset(&s_start, 0, sizeof(s_start));
    /* 进入放电阶段，等待20ms后再上电。 */
    s_start.phase = KT_START_DISCHARGE;
    /* 记录放电阶段起点，service据此计算20ms。 */
    s_start.phase_ms = HAL_GetTick();
    /* 对外报告初始化进行中。 */
    init_status = HAL_BUSY;
}

/* 电源策略入口：1重新分步启动，0取消启动并断电。 */
void kt6368a_set_enabled(uint8_t enabled)
{
    if (enabled != 0)
    {
        /* 请求开启：从头进入分步启动流程。 */
        kt6368a_init_begin();
    }
    else
    {
        /* 显式关闭时取消未完成的启动，后续service不得重新上电。 */
        s_start.phase = KT_START_OFF;
        /* 未完成初始化按错误上报。 */
        init_status = HAL_ERROR;
        /* 关闭模块电源与UART。 */
        kt6368a_power_off();
    }
}

/* 查询分步启动是否仍在进行（非空闲阶段即为进行中）。 */
uint8_t kt6368a_init_is_pending(void)
{
    return s_start.phase != KT_START_OFF;
}

/* 结束分步启动：记录最终状态、回到空闲阶段并清空接收队列。 */
static HAL_StatusTypeDef kt6368a_start_finish(HAL_StatusTypeDef status)
{
    /* 把本次结果作为对外初始化状态。 */
    init_status = status;
    /* 状态机回到空闲，后续service不再推进。 */
    s_start.phase = KT_START_OFF;
    /* 丢弃AT交互期间残留的响应字节，避免混入业务数据。 */
    kt6368a_flush_rx();
    /* 返回同一状态供调用处直接转交。 */
    return status;
}

/* 当前参数处理完毕：推进到下一项查询，或进入重新上电验证/结束。 */
static void kt6368a_start_next(void)
{
    /* 每项查询重新计数重试次数。 */
    s_start.attempts = 0;
    /* 查询索引前进：0名称、1 BLE开关、2 SPP开关。 */
    ++s_start.query;
    if (s_start.query < 3)
    {
        /* 还有未查询项：回到查询阶段。 */
        s_start.phase = KT_START_QUERY;
    }
    else if (s_start.changed != 0 && s_start.verifying == 0)
    {
        /* 参数保存后保留原重新上电验证，不重复写已匹配参数。 */
        kt6368a_power_off();
        s_start.verifying = 1;
        s_start.query = 0;
        s_start.phase = KT_START_DISCHARGE;
        s_start.phase_ms = HAL_GetTick();
    }
    else
    {
        /* 无需验证（或验证已通过）：本次分步启动成功结束。 */
        (void)kt6368a_start_finish(HAL_OK);
    }
}

/* 瞬态错误仅重查当前参数；明确拒绝/验证不匹配仍直接结束。 */
static HAL_StatusTypeDef kt6368a_start_retry(HAL_StatusTypeDef status)
{
    if (s_start.attempts >= 3)
    {
        /* 同一参数已尝试3次仍失败：以本次错误结束启动。 */
        return kt6368a_start_finish(status);
    }
    /* 进入重试等待阶段，100ms后回到查询。 */
    s_start.phase = KT_START_RETRY;
    s_start.phase_ms = HAL_GetTick();
    /* 清掉可能残留的半行响应，重查时重新拼行。 */
    kt6368a_flush_rx();
    /* 对调用方仍表示初始化进行中。 */
    return HAL_BUSY;
}

/* 分步启动状态机：每次调用只推进一步，不长时间等待；返回HAL_BUSY表示仍在进行。 */
HAL_StatusTypeDef kt6368a_init_service(void)
{
    /* 三项AT查询命令，索引与s_start.query对应：0名称、1 BLE、2 SPP。 */
    static const char *const queries[] = {"AT+TM\r\n", "AT+T4\r\n", "AT+T5\r\n"};
    /* 三项响应各自的行前缀，用于匹配查询结果。 */
    static const char *const prefixes[] = {"TM+", "T4+", "T5+"};
    /* 本次调用的当前tick，用于各阶段计时。 */
    uint32_t now = HAL_GetTick();
    /* 本次最多处理的接收字节数，避免单次调用长时间占用调度。 */
    uint16_t budget = 64;
    /* 当前读到的接收字节。 */
    uint8_t byte;
    /* 需要写入时拼好的AT设置命令。 */
    char setting[40];
    /* 当前查询项对应的目标值（名称或"01"/SPP值）。 */
    const char *target;
    /* 发送/启用电源等步骤的HAL状态。 */
    HAL_StatusTypeDef status;
    if (s_start.phase == KT_START_OFF)
    {
        /* 状态机空闲：直接返回上次初始化结果。 */
        return init_status;
    }
    if (s_start.phase == KT_START_DISCHARGE)
    {
        if ((uint32_t)(now - s_start.phase_ms) >= 20)
        {
            /* 断电放电已满20ms：执行上电与接收启动。 */
            status = kt6368a_power_enable();
            if (status != HAL_OK)
            {
                /* 上电失败：以该状态结束本次启动。 */
                return kt6368a_start_finish(status);
            }
            /* 上电成功：进入启动等待阶段。 */
            s_start.phase = KT_START_BOOT;
        }
        /* 放电未满20ms或刚上电：本次调用结束，下次再推进。 */
        return HAL_BUSY;
    }
    if (s_start.phase == KT_START_BOOT)
    {
        if ((uint32_t)(now - ble.power_on_tick) >= KT6368A_BOOT_DELAY_MS)
        {
            /* 上电已满启动等待时间：可以开始发AT查询。 */
            s_start.phase = KT_START_QUERY;
        }
        /* 等待期间对调用方仍表示初始化进行中。 */
        return HAL_BUSY;
    }
    if (s_start.phase == KT_START_SAVE)
    {
        if ((uint32_t)(now - s_start.phase_ms) >= 100)
        {
            /* 设置命令发出后等待100ms，让模块保存参数，再处理下一项。 */
            kt6368a_start_next();
        }
        /* 等待期间返回当前初始化状态（此时仍为HAL_BUSY）。 */
        return init_status;
    }
    if (s_start.phase == KT_START_RETRY)
    {
        if ((uint32_t)(now - s_start.phase_ms) >= 100)
        {
            /* 重试等待满100ms：回到查询阶段重新发送当前项。 */
            s_start.phase = KT_START_QUERY;
        }
        return HAL_BUSY;
    }
    if (s_start.phase == KT_START_QUERY)
    {
        /* 发查询前清空旧字节，避免把上一项残留响应当成本次应答。 */
        kt6368a_flush_rx();
        /* 行缓冲长度与溢出标记清零，准备接收新响应。 */
        s_start.length = s_start.overflow = 0;
        /* 记录发送前的错误/丢字节计数基准，等待期间变化即判失败。 */
        s_start.rx_errors = ble.rx_errors;
        s_start.rx_dropped = ble.rx_dropped;
        /* 记录本次发送时刻，作为响应超时起点。 */
        s_start.phase_ms = now;
        /* 累计本项尝试次数，供重试上限判断。 */
        ++s_start.attempts;
        /* 阻塞发送当前查询命令，单次发送超时10ms。 */
        status = HAL_UART_Transmit(&huart1, (uint8_t *)queries[s_start.query], (uint16_t)strlen(queries[s_start.query]), 10);
        if (status != HAL_OK)
        {
            /* 发送失败属于瞬态错误：按重试策略处理。 */
            return kt6368a_start_retry(status);
        }
        /* 发送成功：进入等待响应阶段。 */
        s_start.phase = KT_START_REPLY;
        return HAL_BUSY;
    }
    /* 以下为KT_START_REPLY：等待并解析当前查询的响应。 */
    if (ble.rx_errors != s_start.rx_errors || ble.rx_dropped != s_start.rx_dropped)
    {
        /* 等待期间出现接收错误或丢字节：响应可能不完整，重查当前项。 */
        return kt6368a_start_retry(HAL_ERROR);
    }
    if ((uint32_t)(now - s_start.phase_ms) >= KT6368A_AT_TIMEOUT_MS)
    {
        /* 超过AT超时仍无有效响应：按超时重查当前项。 */
        return kt6368a_start_retry(HAL_TIMEOUT);
    }
    /* 每次调用最多消费budget个字节，逐字节拼行。 */
    while (budget-- != 0 && kt6368a_read(&byte, 1) != 0)
    {
        if (byte == '\r')
        {
            /* 回车不参与行内容，直接跳过。 */
            continue;
        }
        if (byte != '\n')
        {
            /* 普通字符：容量够就追加到行缓冲，否则只标记溢出。 */
            if (s_start.length < sizeof(s_start.line) - 1)
            {
                s_start.line[s_start.length++] = (char)byte;
            }
            else
            {
                s_start.overflow = 1;
            }
            continue;
        }
        if (s_start.overflow != 0)
        {
            /* 整行超出缓冲：无法可靠比较，按错误重查。 */
            return kt6368a_start_retry(HAL_ERROR);
        }
        /* 换行到达：补结束符，得到不含CRLF的一行。 */
        s_start.line[s_start.length] = '\0';
        /* 行缓冲长度清零，准备解析下一行。 */
        s_start.length = 0;
        if (strncmp(s_start.line, "ER+", 3) == 0)
        {
            /* 模块明确回ER+错误：这是确定失败，不重试，直接结束启动。 */
            return kt6368a_start_finish(HAL_ERROR);
        }
        if (strncmp(s_start.line, prefixes[s_start.query], 3) != 0)
        {
            /* 不是当前项的前缀：忽略该行，继续等下一行。 */
            continue;
        }
        /* 默认目标值"01"对应T4+（开启BLE）。 */
        target = "01";
        if (s_start.query == 0)
        {
            /* 第0项查询TM+：目标为期望的BLE名称。 */
            target = KT6368A_BLE_NAME;
        }
        else if (s_start.query == 2)
        {
            /* 第2项查询T5+：目标为当前构建的SPP值。 */
            target = KT6368A_SPP_VALUE;
        }
        if (strcmp(s_start.line + 3, target) == 0)
        {
            /* 跳过3字节前缀后与目标一致：当前项已满足，处理下一项。 */
            kt6368a_start_next();
            return init_status;
        }
        if (s_start.verifying != 0)
        {
            /* 已改过参数并重新上电复查仍不匹配：确定失败，直接结束。 */
            return kt6368a_start_finish(HAL_ERROR);
        }
        /* 与目标不一致且尚未验证：按当前项拼对应的设置命令。 */
        if (s_start.query == 0)
        {
            /* 名称项：拼"AT+BM" + 目标名称 + CRLF。 */
            memcpy(setting, "AT+BM", 5);
            memcpy(setting + 5, target, strlen(target));
            memcpy(setting + 5 + strlen(target), "\r\n", 3);
        }
        else if (s_start.query == 1)
        {
            /* BLE项：写AT+B401开启BLE。 */
            strcpy(setting, "AT+B401\r\n");
        }
        else
        {
            /* SPP项：写当前构建的SPP开关命令。 */
            strcpy(setting, KT6368A_SPP_COMMAND);
        }
        /* 阻塞发送设置命令，单次超时10ms。 */
        status = HAL_UART_Transmit(&huart1, (uint8_t *)setting, (uint16_t)strlen(setting), 10);
        if (status != HAL_OK)
        {
            /* 发送失败属于瞬态错误：按重试策略处理。 */
            return kt6368a_start_retry(status);
        }
        /* 标记改过参数，三项处理完后需要重新上电验证。 */
        s_start.changed = 1;
        /* 进入保存等待阶段。 */
        s_start.phase = KT_START_SAVE;
        s_start.phase_ms = HAL_GetTick();
        return HAL_BUSY;
    }
    /* 本次预算内没有解析出完整有效行：保持等待，下次调用继续。 */
    return HAL_BUSY;
}

/* 兼容旧调用的阻塞初始化：断电、清统计、上电并立即配置名称。 */
HAL_StatusTypeDef kt6368a_init(void)
{
    /* 初始按失败处理，任一后续步骤成功才更新。 */
    init_status = HAL_ERROR;
    /* 先断电释放模块，保证从确定的初始状态开始。 */
    kt6368a_power_off();
    /* 清空接收错误与丢字节计数。 */
    ble.rx_dropped = 0;
    ble.rx_errors = 0;
    /* 阻塞上电，内部包含20ms放电与启动等待。 */
    init_status = kt6368a_power_on();
    if (init_status == HAL_OK)
    {
        /* 上电成功才配置BLE名称与SPP开关。 */
        init_status = kt6368a_configure(KT6368A_BLE_NAME);
    }
    /* 无论配置结果如何，都清掉AT交互残留的接收数据。 */
    kt6368a_flush_rx();
    /* 返回上电或配置的最终状态。 */
    return init_status;
}

/* ISR/轮询共用的入队函数：记录到达时刻与边界标志，按水位通知。 */
static void kt6368a_rx_push(uint8_t byte, uint8_t notify)
{
    /* head+1后的位置，用于判断队列是否已满。 */
    uint16_t next;
    /* 本字节到达时的HAL tick，存入槽位供分块使用。 */
    uint32_t now_ms;
    /* 本字节的2位边界标志；shift为该槽在标志字节中的位偏移。 */
    uint8_t flags = 0, shift;
    /* 取当前tick作为到达时间与空闲判断基准。 */
    now_ms = HAL_GetTick();
    if (ble.rx_have_byte == 0 || (uint32_t)(now_ms - ble.rx_activity_ms) >= KT6368A_RX_IDLE_MS)
    {
        /* 首次收字节或距上次活动已满空闲阈值：标记新的空闲边界。 */
        flags = KT6368A_RX_GAP;
    }
    if (ble.rx_loss_pending != 0)
    {
        /* 之前有丢弃尚未上报：叠加丢字节标志。 */
        flags |= KT6368A_RX_LOSS;
    }
    /* 更新最近活动时刻与“已收过字节”标记。 */
    ble.rx_activity_ms = now_ms;
    ble.rx_have_byte = 1;
    /* 计算head的下一个槽位并回绕。 */
    next = (uint16_t)((ble.rx_head + 1) % KT6368A_RX_BUFFER_SIZE);
    if (next == ble.rx_tail)
    {
        /* 下一槽就是读指针：队列已满，本字节丢弃。 */
        ble.rx_dropped++;
        /* 记下丢字节待上报，后续字节带上LOSS标志。 */
        ble.rx_loss_pending = 1;
    }
    else
    {
        /* 有空间：写入数据、到达时间和2位边界标志。 */
        ble.rx_buffer[ble.rx_head] = byte;
        ble.rx_time_ms[ble.rx_head] = now_ms;
        /* 每槽2位：位偏移为槽内序号乘2。 */
        shift = (uint8_t)((ble.rx_head % 4) * 2);
        /* 先清掉本槽原2位，再写入本次标志。 */
        ble.rx_flags[ble.rx_head / 4] = (uint8_t)((ble.rx_flags[ble.rx_head / 4] & ~(3 << shift)) | (flags << shift));
        /* 本字节已入队，清除待上报的丢字节标记。 */
        ble.rx_loss_pending = 0;
        /* 确保数据与标志先写完，再发布新head。 */
        __DMB();
        ble.rx_head = next;
        /* 按水位唤醒，避免115200串口逐字节触发业务任务；余量保留给调度延迟。 */
        if ((uint16_t)((next + KT6368A_RX_BUFFER_SIZE - ble.rx_tail) % KT6368A_RX_BUFFER_SIZE) == 128 && notify != 0 && s_rx_callback != NULL)
        {
            /* 队列中未读字节达到128且允许通知：调用已注册回调（ISR上下文）。 */
            s_rx_callback();
        }
    }
}

/* 查询接收是否可用（模块已供电即视为接收使能）。 */
uint8_t kt6368a_rx_is_enabled(void)
{
    return ble.powered;
}

/* Flash写入期间的中断屏蔽轮询入口：把收到的字节送入同一接收队列。 */
void kt6368a_receive_polled(uint8_t byte, uint8_t errors)
{
    /* Flash写入期间由sensor屏蔽中断后调用；HAL的一字节接收仍保持在途。 */
    if (ble.powered == 0)
    {
        /* 模块未供电：不接收，直接返回。 */
        return;
    }
    if (errors != 0)
    {
        /* SR报告接收错误：累计错误数并标记丢字节边界，不把该字节当数据。 */
        ++ble.rx_errors;
        ble.rx_loss_pending = 1;
        /* 错误同样算一次UART活动，更新活动时刻与已收字节标记。 */
        ble.rx_activity_ms = HAL_GetTick();
        ble.rx_have_byte = 1;
        return;
    }
    kt6368a_rx_push(byte, 0); /* 任务上下文不调用FromISR通知；sensor轮末通知LVGL。 */
}

/* USART1接收完成中断：把字节入队并重新挂起下一字节接收。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart1)
    {
        return;
    }
    if (s_rx_override != NULL && s_rx_override(0) != 0)
    {
        return;
    }
    if (!ble.powered)
    {
        return;
    }
    if (huart->ErrorCode != HAL_UART_ERROR_NONE)
    {
        /* 带错误码的完成事件不当作正常字节，留给错误回调处理。 */
        return;
    }
    /* 正常字节入队，并允许达到水位时通知业务任务。 */
    kt6368a_rx_push(ble.rx_byte, 1);
    if (HAL_UART_Receive_IT(huart, &ble.rx_byte, 1) != HAL_OK)
    {
        /* 续接接收失败：累计错误并标记丢字节，接收可能就此停止。 */
        ble.rx_errors++;
        ble.rx_loss_pending = 1;
    }
}

/* USART1错误中断：计数、标记边界，清错误标志后重挂接收。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart1)
    {
        return;
    }
    if (s_rx_override != NULL && s_rx_override(1) != 0)
    {
        return;
    }
    if (!ble.powered)
    {
        return;
    }
    /* 累计错误次数，并记录一次活动、标记丢字节边界。 */
    ble.rx_errors++;
    ble.rx_activity_ms = HAL_GetTick();
    ble.rx_have_byte = 1;
    ble.rx_loss_pending = 1;
    /* 此路径未启用UART RX DMA（Bootloader DMA激活时已在上方返回）；AbortReceive仅关闭接收中断，不在ISR等待DMA。 */
    (void)HAL_UART_AbortReceive(huart);
    /* 清除溢出错误标志，避免下一次接收立刻再次报错。 */
    __HAL_UART_CLEAR_OREFLAG(huart);
    if (HAL_UART_Receive_IT(huart, &ble.rx_byte, 1) != HAL_OK)
    {
        /* 重挂接收仍失败：再计一次错误，接收保持停止。 */
        ble.rx_errors++;
    }
}
