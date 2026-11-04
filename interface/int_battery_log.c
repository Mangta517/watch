#include "int_battery_log.h"
#include "kt6368a.h"
#include <string.h>

/* 测试替换寄存器的副作用，固件直接写半字、读取BSY，不调用Flash中的HAL。 */
#ifndef BATTERY_LOG_PROGRAM_HALFWORD
#define BATTERY_LOG_PROGRAM_HALFWORD(address, value) (*(volatile uint16_t *)(address) = (value))
#endif
#ifndef BATTERY_LOG_POLL_STATUS
#define BATTERY_LOG_POLL_STATUS() (FLASH->SR)
#endif

/* 两个APP的scatter把此段复制到SRAM；循环及常量均不能依赖Flash取指。 */
#if defined(__CC_ARM)
__attribute__((section("battery_log_ram"), noinline))
#endif
static uint8_t battery_log_program_halfword(uint32_t address, uint16_t value, uint16_t received[8], uint8_t *count, uint8_t uart_enabled)
{
    /* status保存最近一次读到的Flash状态寄存器快照。 */
    uint32_t status, uart_status;
    /* byte把USART1数据字节和SR错误位拼成一个16位值，交给轮询接收入口。 */
    uint16_t byte;
    /* 只改PSIZE字段，设为半字编程并置PG位，进入编程状态。 */
    FLASH->CR = (FLASH->CR & ~FLASH_CR_PSIZE) | FLASH_PSIZE_HALF_WORD | FLASH_CR_PG;
    /* 直接向目标地址写入半字，由硬件启动本次编程；该宏不经过Flash中的HAL函数。 */
    BATTERY_LOG_PROGRAM_HALFWORD(address, value);
    /* 轮询BSY直到编程结束；期间利用空档把UART收到的字节取出，避免中断屏蔽时丢数据。 */
    do
    {
        /* 读取Flash状态寄存器，稍后判断忙位和错误标志。 */
        status = BATTERY_LOG_POLL_STATUS();
        /* 仅当调用方告知UART接收已使能时才查询USART1。 */
        if (uart_enabled != 0)
        {
            /* 读取USART1状态寄存器，RXNE表示接收数据寄存器非空，低4位为PE/FE/NF/ORE错误标志。 */
            uart_status = USART1->SR;
            /* 有数据可读或有任一接收错误时都需要读DR，读DR会同时清除这些标志。 */
            if ((uart_status & (USART_SR_RXNE | 0x0F)) != 0)
            {
                /* 低8位保存数据，bit8..11保存PE/FE/NF/ORE标志，合成一个值交给上层区分。 */
                byte = (uint16_t)((USART1->DR & 0xFF) | ((uart_status & 0x0F) << 8));
                /* 缓冲区最多存放8个字节，未满时追加并递增计数。 */
                if (*count < 8)
                {
                    received[(*count)++] = byte;
                }
                /* 缓冲已满：丢弃该字节，并在第8项置溢出标记。 */
                else
                {
                    received[7] |= 0x0800; /* 不把异常长等待导致的溢出伪装成完整消息。 */
                }
            }
        }
    }
    while ((status & FLASH_SR_BSY) != 0);
    /* 编程结束，清除PG位退出编程模式。 */
    FLASH->CR &= ~FLASH_CR_PG;
    /* 任一操作/写保护/对齐/并行/编程顺序错误置位即返回1，否则返回0。 */
    return (status & (FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR)) != 0;
}

static void battery_log_refresh_cache(void)
{
    /* 记录调用前数据Cache是否已使能，结束后按原状态恢复。 */
    uint32_t enabled = FLASH->ACR & FLASH_ACR_DCEN;
    /* 先关闭数据Cache，才能安全复位其内容。 */
    __HAL_FLASH_DATA_CACHE_DISABLE();
    /* 清空数据Cache，避免读到擦写前的旧内容。 */
    __HAL_FLASH_DATA_CACHE_RESET();
    /* 原来处于使能状态时才重新打开，不改变调用方的Cache配置。 */
    if (enabled != 0)
    {
        __HAL_FLASH_DATA_CACHE_ENABLE();
    }
}

void int_battery_log_read(uint32_t slot, uint8_t record[BATTERY_LOG_RECORD_BYTES])
{
    /* 逐字节读取的循环下标。 */
    uint32_t i;
    /* 槽号超出容量：没有对应记录，用0xFF填充表示未写入的擦除态。 */
    if (slot >= BATTERY_LOG_CAPACITY)
    {
        memset(record, 0xFF, BATTERY_LOG_RECORD_BYTES);
        return;
    }
    /* 按记录起始地址加槽偏移逐字节读取，volatile保证每次都真正访问Flash。 */
    for (i = 0; i < BATTERY_LOG_RECORD_BYTES; ++i)
    {
        record[i] = *(volatile const uint8_t *)(BATTERY_LOG_START_ADDR + slot * BATTERY_LOG_RECORD_BYTES + i);
    }
}

uint8_t int_battery_log_write(uint32_t slot, const uint8_t record[BATTERY_LOG_RECORD_BYTES])
{
    /* 暂存写入前读到的槽内容，也用于最后读回校验。 */
    uint8_t old[BATTERY_LOG_RECORD_BYTES];
    /* 逐字节检查擦除态、逐半字编程的循环下标。 */
    uint32_t i;
    /* 由相邻两个字节拼成的半字，作为一次编程的数据。 */
    uint16_t value;
    /* 编程忙等期间从UART取出的字节及错误标志，最多缓存8个。 */
    uint16_t received[8];
    /* count为已缓存个数，byte为轮询下标，uart_enabled表示KT6368A接收是否使能。 */
    uint8_t count, byte, uart_enabled;
    /* 进入临界区前保存的PRIMASK值。 */
    uint32_t mask;
    /* HAL解锁、编程、加锁的累计结果，最后与读回校验共同决定返回值。 */
    HAL_StatusTypeDef result;

    /* 槽号越界直接判为参数错误。 */
    if (slot >= BATTERY_LOG_CAPACITY)
    {
        return 1;
    }
    /* 先读该槽当前内容，用于判断是否仍为擦除态。 */
    int_battery_log_read(slot, old);
    /* 任一字节不是0xFF说明该槽已写过，拒绝覆盖并返回失败。 */
    for (i = 0; i < sizeof(old); ++i)
    {
        if (old[i] != 0xFF)
        {
            return 1;
        }
    }
    /* sensor持有日志锁；单次编程在SRAM轮询UART，半字之间恢复中断，最后写CRC。 */
    result = HAL_FLASH_Unlock();
    /* 只有解锁成功才进行擦除标志清除和编程。 */
    if (result == HAL_OK)
    {
        /* 清除上一次遗留的完成与各类错误标志，避免影响本次判断。 */
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                               FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
        /* 每两个字节组成一个半字写入，因此步长为2。 */
        for (i = 0; i < BATTERY_LOG_RECORD_BYTES; i += 2)
        {
            /* 低字节在前、高字节在后拼成STM32小端半字。 */
            value = (uint16_t)((uint16_t)record[i] | ((uint16_t)record[i + 1] << 8));
            /* 保存当前中断屏蔽状态。 */
            mask = __get_PRIMASK();
            /* 编程期间关中断，避免Flash忙时进入需要从Flash取指的中断。 */
            __disable_irq();
            /* 本次编程期间轮询到的UART字节数从0开始。 */
            count = 0;
            /* 查询KT6368A是否已使能接收，决定忙等时是否顺带取UART数据。 */
            uart_enabled = kt6368a_rx_is_enabled();
            /* 默认本次半字编程成功，失败时再改为HAL_ERROR。 */
            result = HAL_OK;
            /* 在SRAM中轮询完成半字编程，返回非0表示Flash错误标志置位。 */
            if (battery_log_program_halfword(BATTERY_LOG_START_ADDR + slot * BATTERY_LOG_RECORD_BYTES + i,
                                            value, received, &count, uart_enabled) != 0)
            {
                result = HAL_ERROR;
            }
            /* 把忙等期间缓存的字节逐个送进KT6368A统一接收流程。 */
            for (byte = 0; byte < count; ++byte)
            {
                kt6368a_receive_polled((uint8_t)received[byte], (uint8_t)(received[byte] >> 8));
            }
            /* 恢复进入本次编程前的中断屏蔽状态。 */
            __set_PRIMASK(mask);
            /* 本次半字失败则停止后续编程。 */
            if (result != HAL_OK)
            {
                break;
            }
        }
        /* 加锁失败也视为整体失败。 */
        if (HAL_FLASH_Lock() != HAL_OK)
        {
            result = HAL_ERROR;
        }
        /* 擦写后复位数据Cache，防止后续读到旧数据。 */
        battery_log_refresh_cache();
    }
    /* 重新读取该槽，与待写内容比对做读回校验。 */
    int_battery_log_read(slot, old);
    /* 内容不一致说明编程未真正生效，判为失败。 */
    if (memcmp(record, old, sizeof(old)) != 0)
    {
        result = HAL_ERROR;
    }
    /* 对外统一为0成功、1失败。 */
    return result != HAL_OK;
}

void int_battery_log_erase(void)
{
    /* 擦除参数结构体，先整体清零再逐项填写。 */
    FLASH_EraseInitTypeDef erase = {0};
    /* HAL在擦除出错时回填的扇区号，本函数不据此分支。 */
    uint32_t sector_error;
    /* 解锁失败则无法擦除，直接返回。 */
    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return;
    }
    /* 选择按扇区擦除方式。 */
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    /* 固定擦除第7扇区，即日志所在的0x08060000起128KB。 */
    erase.Sector = FLASH_SECTOR_7;
    /* 只擦除一个扇区。 */
    erase.NbSectors = 1;
    /* 供电电压范围3，对应2.7V～3.6V的编程/擦除参数。 */
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    /* 清除历史完成与错误标志，避免干扰本次擦除。 */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    /* 执行扇区擦除，返回状态被忽略；随后仍会加锁。 */
    (void)HAL_FLASHEx_Erase(&erase, &sector_error);
    /* 擦除后重新加锁Flash，返回值同样被忽略。 */
    (void)HAL_FLASH_Lock();
    /* 复位数据Cache，保证之后读到擦除后的新内容。 */
    battery_log_refresh_cache();
}
