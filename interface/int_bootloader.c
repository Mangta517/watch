#include "int_bootloader.h"
#include "usart.h"
#include "kt6368a.h"
#include "bl24c02.h"
#include <string.h>

#define BOOTLOADER_WORK_SIZE     512

/* BL24C02F的0xC0..0xC3：BL标识、状态及反码，避开校准和开机时长记录。 */
static uint8_t int_bootloader_write_flag(uint8_t state)
{
    /* 4字节记录：前两字节为"BL"标识，后两字节为状态及状态反码。 */
    uint8_t record[4] = {0x42, 0x4C, 0, 0};
    /* 写后读回的缓冲区，用于确认EEPROM实际保存的内容。 */
    uint8_t verify[4];
    /* 填入状态字节。 */
    record[2] = state;
    /* 填入状态反码，读回时可校验两字节是否互补。 */
    record[3] = (uint8_t)(state ^ 0xFF);
    /* 写入、读回、比较任一步失败都返回1，只有三步都一致才算成功。 */
    if (bl24c02_write(BOOTLOADER_EEPROM_FLAG_ADDR, record, sizeof(record)) != 0 ||
        bl24c02_read(BOOTLOADER_EEPROM_FLAG_ADDR, verify, sizeof(verify)) != 0 ||
        memcmp(record, verify, sizeof(record)) != 0)
    {
        return 1;
    }
    /* 写入内容与读回内容一致，标志已持久化。 */
    return 0;
}

uint32_t uart1_rx_full_len;
uint32_t flash_write_offset;
uint32_t uart1_rx_complete_tick;
uint8_t uart1_rx_idle;
volatile uint32_t uart1_rx_error_count;
uint32_t uart1_rx_overflow_count;

static uint8_t rx_dma_buf[2][BOOTLOADER_DMA_HALF_SIZE];
static volatile uint32_t dma_wrap_count;
static volatile uint32_t dma_ready_end[2];
static volatile uint8_t dma_active;
static uint32_t dma_read_total;
static uint32_t dma_last_total;
static uint32_t rx_last_tick;
static uint32_t app_length;
static uint8_t last_byte;
static uint8_t last_byte_pending;
static uint8_t app_vectors[8];
static uint32_t image_crc;
static uint32_t expected_crc;
static uint8_t crc_enabled;

/* CRC-32/ISO-HDLC，与主机zlib.crc32一致；只计算BIN实际字节。 */
static void int_bootloader_crc_update(const uint8_t *data, uint16_t size)
{
    /* 待计算数据区的字节下标。 */
    uint16_t i;
    /* 当前字节内逐位处理的位下标，0～7。 */
    uint8_t bit;
    /* 逐字节喂入CRC。 */
    for (i = 0; i < size; ++i)
    {
        /* 当前字节异或到CRC累加器低8位。 */
        image_crc ^= data[i];
        /* 每字节处理8位。 */
        for (bit = 0; bit < 8; ++bit)
        {
            /* 最低位为1时按反射多项式0xEDB88320右移并异或。 */
            if ((image_crc & 1) != 0)
            {
                image_crc = (image_crc >> 1) ^ 0xEDB88320U;
            }
            /* 最低位为0时仅右移一位。 */
            else
            {
                image_crc >>= 1;
            }
        }
    }
}

void int_bootloader_expect_crc(uint32_t crc)
{
    /* 保存主机下发的期望CRC值。 */
    expected_crc = crc;
    /* 置使能标记，完成阶段才会比对CRC。 */
    crc_enabled = 1;
}

static uint32_t int_bootloader_sector(uint32_t address)
{
    /* 0x08000000～0x0800FFFF为4个16KB扇区，按0x4000步长换算扇区号。 */
    if (address < 0x08010000)
    {
        return (address - FLASH_BASE) / 0x4000;
    }
    /* 0x08010000～0x0801FFFF是单个64KB扇区4。 */
    if (address < 0x08020000)
    {
        return FLASH_SECTOR_4;
    }
    /* 0x08020000之后为128KB扇区，从扇区5起按0x20000步长递增。 */
    return FLASH_SECTOR_5 + (address - 0x08020000) / 0x20000;
}

uint8_t int_bootloader_erase(uint32_t length)
{
    /* 擦除参数结构体，先整体清零再逐项填写。 */
    FLASH_EraseInitTypeDef erase = {0};
    /* HAL回填的出错扇区号。 */
    uint32_t sector_error = 0;
    /* 镜像首字节所在扇区号。 */
    uint32_t first_sector;
    /* 镜像末字节所在扇区号。 */
    uint32_t last_sector;
    /* 解锁、擦除、加锁的累计状态。 */
    HAL_StatusTypeDef status;

    /* 长度不足以容纳向量表或超过APP区上限，直接判为参数错误。 */
    if (length < sizeof(app_vectors) || length > APP_MAX_SIZE)
    {
        return 1;
    }
    /* 先持久化升级中并读回确认；失败时不擦除原APP。 */
    if (int_bootloader_write_flag(BOOTLOADER_FLAG_UPDATING) != 0)
    {
        return 1;
    }
    /* 计算起始地址对应扇区。 */
    first_sector = int_bootloader_sector(APP_START_ADDR);
    /* 计算末字节地址对应扇区，用length-1避免刚好越到下一扇区。 */
    last_sector = int_bootloader_sector(APP_START_ADDR + length - 1);
    /* 选择按扇区擦除。 */
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    /* 从首扇区开始。 */
    erase.Sector = first_sector;
    /* 连续擦除覆盖整个镜像的扇区数。 */
    erase.NbSectors = last_sector - first_sector + 1;
    /* 供电电压范围3，对应2.7V～3.6V的擦除参数。 */
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    /* 解锁Flash控制寄存器，失败则无法擦除。 */
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK)
    {
        return 1;
    }
    /* 清除历史完成与错误标志。 */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    /* 执行扇区擦除。 */
    status = HAL_FLASHEx_Erase(&erase, &sector_error);
    /* 无论擦除结果如何都要加锁；加锁失败同样记为失败。 */
    if (HAL_FLASH_Lock() != HAL_OK)
    {
        status = HAL_ERROR;
    }
    /* 对外统一为0成功、1失败。 */
    return (status != HAL_OK);
}

uint8_t int_bootloader_rx_dma_active(void)
{
    /* 返回DMA接收是否已启动的标志，1表示正在接收。 */
    return dma_active;
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *uart)
{
    /* 只处理USART1且在DMA接收活动期间的半传输完成。 */
    if (uart == &huart1 && dma_active != 0)
    {
        /* 用回绕计数换算出前半区结束的绝对字节位置，供主循环判断可复制范围。 */
        dma_ready_end[0] = dma_wrap_count * BOOTLOADER_DMA_SIZE + BOOTLOADER_DMA_HALF_SIZE;
    }
}

void int_bootloader_rx_complete(void)
{
    /* 一整轮DMA接收完成，回绕计数加1。 */
    ++dma_wrap_count;
    /* 发布后半区结束的绝对字节位置。 */
    dma_ready_end[1] = dma_wrap_count * BOOTLOADER_DMA_SIZE;
}

void int_bootloader_rx_error(void)
{
    /* 累计UART/DMA接收错误次数，主循环据此中止传输。 */
    ++uart1_rx_error_count;
}

static uint8_t int_bootloader_rx_override(uint8_t is_error)
{
    if (dma_active == 0)
    {
        return 0;
    }
    if (is_error != 0)
    {
        int_bootloader_rx_error();
    }
    else
    {
        int_bootloader_rx_complete();
    }
    return 1;
}

uint8_t int_bootloader_stop(void)
{
    /* 中止USART1当前DMA接收。 */
    HAL_StatusTypeDef status = HAL_UART_AbortReceive(&huart1);
    /* 中止成功才清除活动标志。 */
    if (status == HAL_OK)
    {
        dma_active = 0;
        kt6368a_set_rx_override(NULL);
    }
    /* 中止失败则累计错误计数，便于上层识别异常。 */
    else
    {
        ++uart1_rx_error_count;
    }
    /* 对外统一为0成功、1失败。 */
    return (status != HAL_OK);
}

uint8_t bootloader_start_dma(uint32_t length)
{
    /* HAL启动DMA接收的返回状态。 */
    HAL_StatusTypeDef status;
    /* 长度不足以容纳向量表或超过APP区上限，直接判为参数错误。 */
    if (length < sizeof(app_vectors) || length > APP_MAX_SIZE)
    {
        return 1;
    }
    /* 先停掉可能仍在进行的上一次接收，失败则不启动。 */
    if (int_bootloader_stop() != 0)
    {
        return 1;
    }
    /* 清零回绕计数，从本轮传输起点重新计数。 */
    dma_wrap_count = 0;
    /* 两个半区的就绪位置都归零。 */
    dma_ready_end[0] = dma_ready_end[1] = 0;
    /* 已消费和上次快照的累计字节数都归零。 */
    dma_read_total = dma_last_total = 0;
    /* 对外接收长度和Flash写入偏移归零。 */
    uart1_rx_full_len = flash_write_offset = 0;
    /* 清除完成时刻和空闲标志。 */
    uart1_rx_complete_tick = 0;
    uart1_rx_idle = 0;
    /* 错误与溢出计数清零。 */
    uart1_rx_error_count = uart1_rx_overflow_count = 0;
    /* 清除上一幅镜像残留的奇数字节缓存。 */
    last_byte_pending = 0;
    /* 记录本幅镜像的预期总长度。 */
    app_length = length;
    /* CRC内部累加器初值取0xFFFFFFFF，最终异或0xFFFFFFFF后与zlib.crc32默认计算结果一致。 */
    image_crc = 0xFFFFFFFFU;
    /* 默认不启用CRC校验，等待主机下发期望值。 */
    crc_enabled = 0;
    /* 记录最近一次收到数据的时刻，用于空闲与超时判断。 */
    rx_last_tick = HAL_GetTick();
    /* 向量暂存区先填擦除态，避免与上一幅镜像混淆。 */
    memset(app_vectors, 0xFF, sizeof(app_vectors));
    /* 清除USART1溢出错误标志，防止启动后立即报错。 */
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    /* 标记DMA接收进入活动状态。 */
    kt6368a_set_rx_override(int_bootloader_rx_override);
    dma_active = 1;
    /* 启动8192字节循环DMA接收，缓冲区为双半区数组首地址。 */
    status = HAL_UART_Receive_DMA(&huart1, &rx_dma_buf[0][0], BOOTLOADER_DMA_SIZE);
    /* 启动失败则撤销活动标志，避免主循环误以为在收数据。 */
    if (status != HAL_OK)
    {
        dma_active = 0;
        kt6368a_set_rx_override(NULL);
    }
    /* 对外统一为0成功、1失败。 */
    return (status != HAL_OK);
}

static uint32_t int_bootloader_dma_total(void)
{
    /* 保存当前中断屏蔽状态，读完快照后恢复。 */
    uint32_t mask = __get_PRIMASK();
    /* 回绕计数快照。 */
    uint32_t wraps;
    /* DMA剩余未传输字节数快照。 */
    uint32_t remaining;
    /* 屏蔽可屏蔽中断，防止快照期间回绕中断修改软件计数；DMA硬件仍继续运行。 */
    __disable_irq();
    /* 读取回绕计数。 */
    wraps = dma_wrap_count;
    /* 读取DMA剩余计数。 */
    remaining = __HAL_DMA_GET_COUNTER(huart1.hdmarx);
    /* 传输完成标志已置位但完成中断尚未处理时，需要手工补算这一轮。 */
    if (__HAL_DMA_GET_FLAG(huart1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(huart1.hdmarx)) != RESET)
    {
        /* 补上尚未计入的回绕。 */
        ++wraps;
        /* 重新读取剩余计数。 */
        remaining = __HAL_DMA_GET_COUNTER(huart1.hdmarx);
        /* 剩余为0代表整轮刚结束，按一整轮长度计算。 */
        if (remaining == 0)
        {
            remaining = BOOTLOADER_DMA_SIZE;
        }
    }
    /* 恢复中断屏蔽状态。 */
    __set_PRIMASK(mask);
    /* 确保快照读取先于后续使用完成。 */
    __DMB();
    /* 回绕轮数乘每轮长度，加上本轮已收字节数，得到累计接收字节数。 */
    return wraps * BOOTLOADER_DMA_SIZE + BOOTLOADER_DMA_SIZE - remaining;
}

static uint8_t int_bootloader_program(uint32_t offset, uint16_t value)
{
    /* 把APP区偏移换算成实际Flash地址。 */
    uint32_t address = APP_START_ADDR + offset;
    /* 偏移超出APP区或剩余空间不足一个半字，拒绝编程。 */
    if (offset >= APP_MAX_SIZE || APP_MAX_SIZE - offset < 2)
    {
        return 1;
    }
    /* 以半字为单位编程，HAL返回非成功即判失败。 */
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, value) != HAL_OK)
    {
        return 1;
    }
    /* 读回该半字，与写入值比对确认编程生效。 */
    if (*(volatile uint16_t *)address != value)
    {
        return 1;
    }
    /* 编程并读回一致，返回成功。 */
    return 0;
}

static uint8_t int_bootloader_write(const uint8_t *data, uint16_t size)
{
    /* 累计写入状态，0表示目前全部成功。 */
    uint8_t status = 0;
    /* 由相邻两字节拼成的半字。 */
    uint16_t value;
    /* 本批数据的字节下标。 */
    uint16_t i;
    /* 解锁Flash，失败则无法编程。 */
    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return 1;
    }
    /* 逐字节处理，前8字节进向量暂存区，其余按半字编程。 */
    for (i = 0; i < size; ++i)
    {
        /* 镜像前8字节是初始MSP和复位向量，先暂存不写Flash。 */
        if (flash_write_offset < sizeof(app_vectors))
        {
            /* 先保留向量，整幅镜像校验成功后再提交，断电后不会跳入半幅程序。 */
            app_vectors[flash_write_offset++] = data[i];
        }
        /* 正文阶段：还没有待配对的低字节时，先缓存当前字节。 */
        else if (last_byte_pending == 0)
        {
            last_byte = data[i];
            last_byte_pending = 1;
        }
        /* 已有低字节时，与当前高字节拼成半字并编程。 */
        else
        {
            value = (uint16_t)((uint16_t)last_byte | ((uint16_t)data[i] << 8));
            status = int_bootloader_program(flash_write_offset, value);
            /* 编程失败则停止后续写入。 */
            if (status != 0)
            {
                break;
            }
            /* 编程成功，写入偏移前进一个半字，清除待配对标记。 */
            flash_write_offset += 2;
            last_byte_pending = 0;
        }
    }
    /* 加锁失败也视为整体失败。 */
    if (HAL_FLASH_Lock() != HAL_OK)
    {
        status = 1;
    }
    /* 返回0表示本批全部写入成功，1表示中途失败。 */
    return status;
}

uint8_t int_bootloader_process(void)
{
    /* 本次最多搬运512字节到栈上工作区，再交给写入函数。 */
    uint8_t work[BOOTLOADER_WORK_SIZE];
    /* DMA累计已接收的绝对字节数。 */
    uint32_t received;
    /* 本次调用的毫秒时刻，供空闲与超时判断复用。 */
    uint32_t now = HAL_GetTick();
    /* 当前消费位置所在的双半区编号（0或1）。 */
    uint32_t half;
    /* 当前半区的结束绝对位置。 */
    uint32_t half_end;
    /* 本次允许复制到的绝对位置，默认不复制。 */
    uint32_t copy_end;
    /* 已接收但尚未消费的字节数。 */
    uint32_t available;
    /* 本次实际搬进work的字节数。 */
    uint16_t count = 0;

    /* DMA未启动或已记录接收错误时无法继续，返回失败。 */
    if (dma_active == 0 || uart1_rx_error_count != 0)
    {
        return 1;
    }
    /* 取一次DMA累计接收字节数快照。 */
    received = int_bootloader_dma_total();
    /* 对外发布当前累计接收长度。 */
    uart1_rx_full_len = received;
    /* 累计值发生变化说明刚收到新数据，刷新活动时刻并清除空闲/完成标记。 */
    if (received != dma_last_total)
    {
        dma_last_total = received;
        rx_last_tick = now;
        uart1_rx_idle = 0;
        uart1_rx_complete_tick = 0;
    }
    /* 尚未消费的字节数等于累计接收减去已消费。 */
    available = received - dma_read_total;
    /* 未消费量达到DMA缓冲容量，按溢出处理。 */
    if (available >= BOOTLOADER_DMA_SIZE)
    {
        uart1_rx_overflow_count += available;
        return 1;
    }
    /* 收到的字节超过本幅镜像长度，判为异常。 */
    if (received > app_length)
    {
        return 1;
    }
    /* 尚未收满且距最近观测到新数据已满30秒，判为接收超时。 */
    if (received < app_length && (uint32_t)(now - rx_last_tick) >= BOOTLOADER_TRANSFER_TIMEOUT_MS)
    {
        return 1;
    }
    /* 用已消费位置除以半区长度再取模，得到当前正在消费的半区编号。 */
    half = (dma_read_total / BOOTLOADER_DMA_HALF_SIZE) % 2;
    /* 该半区结束位置＝已消费位置加上本半区剩余字节。 */
    half_end = dma_read_total + BOOTLOADER_DMA_HALF_SIZE - dma_read_total % BOOTLOADER_DMA_HALF_SIZE;
    /* 默认本拍不复制，只有确认数据稳定后才放宽到half_end或received。 */
    copy_end = dma_read_total;
    /* 该半区已被DMA标记为收满，可以整段复制。 */
    if (dma_ready_end[half] == half_end)
    {
        copy_end = half_end;
    }
    /* 半区未收满但线路已空闲10ms，按当前已收位置复制。 */
    else if ((uint32_t)(now - rx_last_tick) >= BOOTLOADER_RX_IDLE_MS)
    {
        copy_end = received;
    }
    /* 从环形缓冲按绝对位置取字节，最多取work容量。 */
    while (dma_read_total + count < copy_end && count < sizeof(work))
    {
        /* 绝对位置对8192取模得到缓冲内偏移。 */
        uint32_t position = (dma_read_total + count) % BOOTLOADER_DMA_SIZE;
        /* 偏移除以半区长度选半区，取模得半区内偏移，逐字节搬入work。 */
        work[count++] = rx_dma_buf[position / BOOTLOADER_DMA_HALF_SIZE][position % BOOTLOADER_DMA_HALF_SIZE];
    }
    /* 复制期间DMA可能回绕；被覆盖的数据不能继续按原偏移写入。 */
    __DMB();
    /* 复制期间源DMA缓冲可能已被覆盖，本批work不再视为可靠。 */
    if (int_bootloader_dma_total() - dma_read_total >= BOOTLOADER_DMA_SIZE)
    {
        ++uart1_rx_overflow_count;
        return 1;
    }
    /* 有数据才进行Flash写入与CRC累计。 */
    if (count != 0)
    {
        /* 把本批字节按半字写入APP区，失败即中止。 */
        if (int_bootloader_write(work, count) != 0)
        {
            return 1;
        }
        /* 主机已下发期望CRC时才累计本批数据。 */
        if (crc_enabled != 0)
        {
            int_bootloader_crc_update(work, count);
        }
        /* 消费位置前进本批字节数。 */
        dma_read_total += count;
    }
    /* 已收字节全部消费、线路空闲满10ms且DMA计数不再增长，判定本次接收结束。 */
    if (received != 0 && dma_read_total == received && uart1_rx_idle == 0 &&
        (uint32_t)(now - rx_last_tick) >= BOOTLOADER_RX_IDLE_MS && int_bootloader_dma_total() == received)
    {
        /* 记录完成时刻，供上层判断结束已保持多久。 */
        uart1_rx_complete_tick = HAL_GetTick();
        /* 置空闲完成标志。 */
        uart1_rx_idle = 1;
    }
    /* 本拍正常推进，未发现错误。 */
    return 0;
}

static uint8_t int_bootloader_vectors_valid(uint32_t stack, uint32_t reset, uint32_t end)
{
    /* 栈顶需落在SRAM 0x20000000～0x20020000内且8字节对齐；复位向量需为Thumb地址并落在APP区。 */
    return (stack > 0x20000000 && stack <= 0x20020000 && (stack & 7) == 0 &&
            (reset & 1) != 0 && (reset & ~1U) >= APP_START_ADDR + 8 && (reset & ~1U) < end);
}

uint8_t int_bootloader_app_is_valid(void)
{
    /* EEPROM中读回的有效标志记录。 */
    uint8_t record[4];
    /* 期望的标志内容："BL"标识、VALID状态、以及UPDATING值作为反码。 */
    const uint8_t valid[4] = {0x42, 0x4C, BOOTLOADER_FLAG_VALID, BOOTLOADER_FLAG_UPDATING};
    /* APP区向量不合法，或标志读取失败，都判为不可启动。 */
    if (int_bootloader_vectors_valid(*(volatile uint32_t *)APP_START_ADDR,
                                     *(volatile uint32_t *)(APP_START_ADDR + 4), APP_FLASH_END_ADDR) == 0 ||
        bl24c02_read(BOOTLOADER_EEPROM_FLAG_ADDR, record, sizeof(record)) != 0)
    {
        return 0;
    }
    /* 向量合法且标志记录与期望完全一致才返回1。 */
    return (memcmp(record, valid, sizeof(record)) == 0);
}

uint8_t int_bootloader_finish(void)
{
    /* 收尾阶段的累计状态，0表示目前全部成功。 */
    uint8_t status = 0;
    /* 从向量暂存区解出的初始MSP值。 */
    uint32_t stack;
    /* 从向量暂存区解出的复位向量。 */
    uint32_t reset;
    /* 提交向量时的偏移游标，也用于读取DMA剩余计数。 */
    uint32_t offset;
    /* 由相邻两字节拼成的半字。 */
    uint16_t value;

    /* 先推进一次接收，再核对长度与写入进度是否全部对齐。 */
    if (int_bootloader_process() != 0 || uart1_rx_full_len != app_length || dma_read_total != app_length ||
        flash_write_offset + last_byte_pending != app_length)
    {
        return 1;
    }
    /* 停止DMA接收，失败或期间出现接收错误都不能提交。 */
    if (int_bootloader_stop() != 0 || uart1_rx_error_count != 0)
    {
        return 1;
    }
    /* Abort后NDTR保持停止位置，拒绝最后一次快照之后多收的字节。 */
    offset = __HAL_DMA_GET_COUNTER(huart1.hdmarx);
    /* 剩余计数为0按整轮长度处理。 */
    if (offset == 0)
    {
        offset = BOOTLOADER_DMA_SIZE;
    }
    /* 停止时的剩余计数必须正好等于镜像长度在最后一轮中的落点。 */
    if (offset != BOOTLOADER_DMA_SIZE - app_length % BOOTLOADER_DMA_SIZE)
    {
        return 1;
    }
    /* CRC失败时向量仍为擦除态，无需再次擦除扇区。 */
    if (crc_enabled != 0 && (image_crc ^ 0xFFFFFFFFU) != expected_crc)
    {
        return 1;
    }
    /* 从暂存区取出前4字节作为初始栈顶。 */
    memcpy(&stack, app_vectors, sizeof(stack));
    /* 取出后4字节作为复位向量。 */
    memcpy(&reset, app_vectors + 4, sizeof(reset));
    /* 用本幅实际长度作为向量合法范围上界，非法则不提交。 */
    if (int_bootloader_vectors_valid(stack, reset, APP_START_ADDR + app_length) == 0)
    {
        return 1;
    }
    /* 提交向量需要写Flash，先解锁。 */
    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return 1;
    }
    /* 镜像长度为奇数时，最后一个字节还未配对，需补0xFF组成半字写入。 */
    if (last_byte_pending != 0)
    {
        /* 最后一个奇数字节补FF写半字，补齐字节不计入镜像长度。 */
        status = int_bootloader_program(flash_write_offset, (uint16_t)(last_byte | 0xFF00));
        /* 补写成功才把写入偏移推进1字节并清除待配对标记。 */
        if (status == 0)
        {
            ++flash_write_offset;
            last_byte_pending = 0;
        }
    }
    /* 从向量末尾向前提交，MSP低半字最后写；中途MSP可能仅写入高半字，完整向量尚不能通过校验。 */
    for (offset = sizeof(app_vectors); offset != 0 && status == 0; offset -= 2)
    {
        /* 从暂存区取相邻两字节，低字节在前拼成半字。 */
        value = (uint16_t)((uint16_t)app_vectors[offset - 2] | ((uint16_t)app_vectors[offset - 1] << 8));
        /* 从后往前写：先写复位向量，低半字最后写，初始MSP在循环末尾才完整。 */
        status = int_bootloader_program(offset - 2, value);
    }
    /* 加锁失败也判为失败。 */
    if (HAL_FLASH_Lock() != HAL_OK)
    {
        status = 1;
    }
    /* 正文与向量提交成功，且启用时CRC检查通过后，再写VALID标志。 */
    if (status == 0 && int_bootloader_write_flag(BOOTLOADER_FLAG_VALID) != 0)
    {
        status = 1;
    }
    /* 返回0表示整幅镜像已提交并置为有效。 */
    return status;
}
