#include "int_bootloader.h"
#include "usart.h"
#include "bl24c02.h"
#include <string.h>

#define BOOTLOADER_WORK_SIZE     512
#define BOOTLOADER_FLAG_UPDATING 0x5A
#define BOOTLOADER_FLAG_VALID    0xA5

/* BL24C02F的0xC0..0xC3：BL标识、状态及反码，避开校准和开机时长记录。 */
static uint8_t int_bootloader_write_flag(uint8_t state)
{
    uint8_t record[4] = {0x42, 0x4C, 0, 0};
    uint8_t verify[4];
    record[2] = state;
    record[3] = (uint8_t)(state ^ 0xFF);
    if (bl24c02_write(BOOTLOADER_EEPROM_FLAG_ADDR, record, sizeof(record)) != 0 ||
        bl24c02_read(BOOTLOADER_EEPROM_FLAG_ADDR, verify, sizeof(verify)) != 0 ||
        memcmp(record, verify, sizeof(record)) != 0)
    {
        return 1;
    }
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
    uint16_t i;
    uint8_t bit;
    for (i = 0; i < size; ++i)
    {
        image_crc ^= data[i];
        for (bit = 0; bit < 8; ++bit)
        {
            if ((image_crc & 1) != 0)
            {
                image_crc = (image_crc >> 1) ^ 0xEDB88320U;
            }
            else
            {
                image_crc >>= 1;
            }
        }
    }
}

void int_bootloader_expect_crc(uint32_t crc)
{
    expected_crc = crc;
    crc_enabled = 1;
}

static uint32_t int_bootloader_sector(uint32_t address)
{
    if (address < 0x08010000)
    {
        return (address - FLASH_BASE) / 0x4000;
    }
    if (address < 0x08020000)
    {
        return FLASH_SECTOR_4;
    }
    return FLASH_SECTOR_5 + (address - 0x08020000) / 0x20000;
}

uint8_t int_bootloader_erase(uint32_t length)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t sector_error = 0;
    uint32_t first_sector;
    uint32_t last_sector;
    HAL_StatusTypeDef status;

    if (length < sizeof(app_vectors) || length > APP_MAX_SIZE)
    {
        return 1;
    }
    /* 先持久化升级中并读回确认；失败时不擦除原APP。 */
    if (int_bootloader_write_flag(BOOTLOADER_FLAG_UPDATING) != 0)
    {
        return 1;
    }
    first_sector = int_bootloader_sector(APP_START_ADDR);
    last_sector = int_bootloader_sector(APP_START_ADDR + length - 1);
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = first_sector;
    erase.NbSectors = last_sector - first_sector + 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK)
    {
        return 1;
    }
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    status = HAL_FLASHEx_Erase(&erase, &sector_error);
    if (HAL_FLASH_Lock() != HAL_OK)
    {
        status = HAL_ERROR;
    }
    return (status != HAL_OK);
}

uint8_t int_bootloader_rx_dma_active(void)
{
    return dma_active;
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart1 && dma_active != 0)
    {
        dma_ready_end[0] = dma_wrap_count * BOOTLOADER_DMA_SIZE + BOOTLOADER_DMA_HALF_SIZE;
    }
}

void int_bootloader_rx_complete(void)
{
    ++dma_wrap_count;
    dma_ready_end[1] = dma_wrap_count * BOOTLOADER_DMA_SIZE;
}

void int_bootloader_rx_error(void)
{
    ++uart1_rx_error_count;
}

uint8_t int_bootloader_stop(void)
{
    HAL_StatusTypeDef status = HAL_UART_AbortReceive(&huart1);
    if (status == HAL_OK)
    {
        dma_active = 0;
    }
    else
    {
        ++uart1_rx_error_count;
    }
    return (status != HAL_OK);
}

uint8_t bootloader_start_dma(uint32_t length)
{
    HAL_StatusTypeDef status;
    if (length < sizeof(app_vectors) || length > APP_MAX_SIZE)
    {
        return 1;
    }
    if (int_bootloader_stop() != 0)
    {
        return 1;
    }
    dma_wrap_count = 0;
    dma_ready_end[0] = dma_ready_end[1] = 0;
    dma_read_total = dma_last_total = 0;
    uart1_rx_full_len = flash_write_offset = 0;
    uart1_rx_complete_tick = 0;
    uart1_rx_idle = 0;
    uart1_rx_error_count = uart1_rx_overflow_count = 0;
    last_byte_pending = 0;
    app_length = length;
    image_crc = 0xFFFFFFFFU;
    crc_enabled = 0;
    rx_last_tick = HAL_GetTick();
    memset(app_vectors, 0xFF, sizeof(app_vectors));
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    dma_active = 1;
    status = HAL_UART_Receive_DMA(&huart1, &rx_dma_buf[0][0], BOOTLOADER_DMA_SIZE);
    if (status != HAL_OK)
    {
        dma_active = 0;
    }
    return (status != HAL_OK);
}

static uint32_t int_bootloader_dma_total(void)
{
    uint32_t mask = __get_PRIMASK();
    uint32_t wraps;
    uint32_t remaining;
    __disable_irq();
    wraps = dma_wrap_count;
    remaining = __HAL_DMA_GET_COUNTER(huart1.hdmarx);
    if (__HAL_DMA_GET_FLAG(huart1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(huart1.hdmarx)) != RESET)
    {
        ++wraps;
        remaining = __HAL_DMA_GET_COUNTER(huart1.hdmarx);
        if (remaining == 0)
        {
            remaining = BOOTLOADER_DMA_SIZE;
        }
    }
    __set_PRIMASK(mask);
    __DMB();
    return wraps * BOOTLOADER_DMA_SIZE + BOOTLOADER_DMA_SIZE - remaining;
}

static uint8_t int_bootloader_program(uint32_t offset, uint16_t value)
{
    uint32_t address = APP_START_ADDR + offset;
    if (offset >= APP_MAX_SIZE || APP_MAX_SIZE - offset < 2)
    {
        return 1;
    }
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, value) != HAL_OK)
    {
        return 1;
    }
    if (*(volatile uint16_t *)address != value)
    {
        return 1;
    }
    return 0;
}

static uint8_t int_bootloader_write(const uint8_t *data, uint16_t size)
{
    uint8_t status = 0;
    uint16_t value;
    uint16_t i;
    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return 1;
    }
    for (i = 0; i < size; ++i)
    {
        if (flash_write_offset < sizeof(app_vectors))
        {
            /* 先保留向量，整幅镜像校验成功后再提交，断电后不会跳入半幅程序。 */
            app_vectors[flash_write_offset++] = data[i];
        }
        else if (last_byte_pending == 0)
        {
            last_byte = data[i];
            last_byte_pending = 1;
        }
        else
        {
            value = (uint16_t)((uint16_t)last_byte | ((uint16_t)data[i] << 8));
            status = int_bootloader_program(flash_write_offset, value);
            if (status != 0)
            {
                break;
            }
            flash_write_offset += 2;
            last_byte_pending = 0;
        }
    }
    if (HAL_FLASH_Lock() != HAL_OK)
    {
        status = 1;
    }
    return status;
}

uint8_t int_bootloader_process(void)
{
    uint8_t work[BOOTLOADER_WORK_SIZE];
    uint32_t received;
    uint32_t now = HAL_GetTick();
    uint32_t half;
    uint32_t half_end;
    uint32_t copy_end;
    uint32_t available;
    uint16_t count = 0;

    if (dma_active == 0 || uart1_rx_error_count != 0)
    {
        return 1;
    }
    received = int_bootloader_dma_total();
    uart1_rx_full_len = received;
    if (received != dma_last_total)
    {
        dma_last_total = received;
        rx_last_tick = now;
        uart1_rx_idle = 0;
        uart1_rx_complete_tick = 0;
    }
    available = received - dma_read_total;
    if (available >= BOOTLOADER_DMA_SIZE)
    {
        uart1_rx_overflow_count += available;
        return 1;
    }
    if (received > app_length)
    {
        return 1;
    }
    if (received < app_length && (uint32_t)(now - rx_last_tick) >= BOOTLOADER_TRANSFER_TIMEOUT_MS)
    {
        return 1;
    }
    half = (dma_read_total / BOOTLOADER_DMA_HALF_SIZE) % 2;
    half_end = dma_read_total + BOOTLOADER_DMA_HALF_SIZE - dma_read_total % BOOTLOADER_DMA_HALF_SIZE;
    copy_end = dma_read_total;
    if (dma_ready_end[half] == half_end)
    {
        copy_end = half_end;
    }
    else if ((uint32_t)(now - rx_last_tick) >= BOOTLOADER_RX_IDLE_MS)
    {
        copy_end = received;
    }
    while (dma_read_total + count < copy_end && count < sizeof(work))
    {
        uint32_t position = (dma_read_total + count) % BOOTLOADER_DMA_SIZE;
        work[count++] = rx_dma_buf[position / BOOTLOADER_DMA_HALF_SIZE][position % BOOTLOADER_DMA_HALF_SIZE];
    }
    /* 复制期间DMA可能回绕；被覆盖的数据不能继续按原偏移写入。 */
    __DMB();
    if (int_bootloader_dma_total() - dma_read_total >= BOOTLOADER_DMA_SIZE)
    {
        ++uart1_rx_overflow_count;
        return 1;
    }
    if (count != 0)
    {
        if (int_bootloader_write(work, count) != 0)
        {
            return 1;
        }
        if (crc_enabled != 0)
        {
            int_bootloader_crc_update(work, count);
        }
        dma_read_total += count;
    }
    if (received != 0 && dma_read_total == received && uart1_rx_idle == 0 &&
        (uint32_t)(now - rx_last_tick) >= BOOTLOADER_RX_IDLE_MS && int_bootloader_dma_total() == received)
    {
        uart1_rx_complete_tick = HAL_GetTick();
        uart1_rx_idle = 1;
    }
    return 0;
}

static uint8_t int_bootloader_vectors_valid(uint32_t stack, uint32_t reset, uint32_t end)
{
    return (stack > 0x20000000 && stack <= 0x20020000 && (stack & 7) == 0 &&
            (reset & 1) != 0 && (reset & ~1U) >= APP_START_ADDR + 8 && (reset & ~1U) < end);
}

uint8_t int_bootloader_app_is_valid(void)
{
    uint8_t record[4];
    const uint8_t valid[4] = {0x42, 0x4C, BOOTLOADER_FLAG_VALID, BOOTLOADER_FLAG_UPDATING};
    if (int_bootloader_vectors_valid(*(volatile uint32_t *)APP_START_ADDR,
                                     *(volatile uint32_t *)(APP_START_ADDR + 4), APP_FLASH_END_ADDR) == 0 ||
        bl24c02_read(BOOTLOADER_EEPROM_FLAG_ADDR, record, sizeof(record)) != 0)
    {
        return 0;
    }
    return (memcmp(record, valid, sizeof(record)) == 0);
}

uint8_t int_bootloader_finish(void)
{
    uint8_t status = 0;
    uint32_t stack;
    uint32_t reset;
    uint32_t offset;
    uint16_t value;

    if (int_bootloader_process() != 0 || uart1_rx_full_len != app_length || dma_read_total != app_length ||
        flash_write_offset + last_byte_pending != app_length)
    {
        return 1;
    }
    if (int_bootloader_stop() != 0 || uart1_rx_error_count != 0)
    {
        return 1;
    }
    /* Abort后NDTR保持停止位置，拒绝最后一次快照之后多收的字节。 */
    offset = __HAL_DMA_GET_COUNTER(huart1.hdmarx);
    if (offset == 0)
    {
        offset = BOOTLOADER_DMA_SIZE;
    }
    if (offset != BOOTLOADER_DMA_SIZE - app_length % BOOTLOADER_DMA_SIZE)
    {
        return 1;
    }
    /* CRC失败时向量仍为擦除态，无需再次擦除扇区。 */
    if (crc_enabled != 0 && (image_crc ^ 0xFFFFFFFFU) != expected_crc)
    {
        return 1;
    }
    memcpy(&stack, app_vectors, sizeof(stack));
    memcpy(&reset, app_vectors + 4, sizeof(reset));
    if (int_bootloader_vectors_valid(stack, reset, APP_START_ADDR + app_length) == 0)
    {
        return 1;
    }
    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return 1;
    }
    if (last_byte_pending != 0)
    {
        /* 最后一个奇数字节补FF写半字，补齐字节不计入镜像长度。 */
        status = int_bootloader_program(flash_write_offset, (uint16_t)(last_byte | 0xFF00));
        if (status == 0)
        {
            ++flash_write_offset;
            last_byte_pending = 0;
        }
    }
    /* 先写复位向量，最后写MSP；写入中断时MSP仍为擦除态。 */
    for (offset = sizeof(app_vectors); offset != 0 && status == 0; offset -= 2)
    {
        value = (uint16_t)((uint16_t)app_vectors[offset - 2] | ((uint16_t)app_vectors[offset - 1] << 8));
        status = int_bootloader_program(offset - 2, value);
    }
    if (HAL_FLASH_Lock() != HAL_OK)
    {
        status = 1;
    }
    /* 正文、CRC和向量全部提交成功后，才允许下次上电启动APP。 */
    if (status == 0 && int_bootloader_write_flag(BOOTLOADER_FLAG_VALID) != 0)
    {
        status = 1;
    }
    return status;
}
