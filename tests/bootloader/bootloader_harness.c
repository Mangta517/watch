/* 复用现有蓝牙模块/AT模拟，执行真实驱动的IT到DMA交接及回调路由。 */
#define main        ble_transport_regression_main
#define HAL_GetTick ble_transport_get_tick
#include "../ble/ble_harness.c"
#undef main
#undef HAL_GetTick
#include "app_bootloader.h"
#include "crc_vectors.h"
#include "bl24c02.h"
#include "my_i2c.h"
#include <setjmp.h>
#include <windows.h>

uint32_t test_dma_remaining;
uint32_t test_dma_tc;
static uint8_t *test_dma_data;
static uint32_t test_dma_position;
static uint8_t flash_locked = 1;
static uint8_t erase_fail, program_fail, corrupt_write, dma_fail;
static uint32_t erase_sector, erase_count, program_count;
static uint32_t jump_count;
static jmp_buf reset_target;
static uint8_t reset_expected;
static uint8_t fixed_tick;
static uint8_t eeprom[BL24C02_CAPACITY_BYTES];
static uint8_t eeprom_active, eeprom_missing, eeprom_read_fail, eeprom_write_fail;
static uint8_t eeprom_protected, eeprom_corrupt, eeprom_partial;
static uint32_t eeprom_writes, flash_erases;

void my_i2c_init(my_i2c_bus_t bus)
{
    assert(bus == MY_I2C_BUS_EEPROM && eeprom_active == 0);
    eeprom_active = 1;
}

void my_i2c_suspend(my_i2c_bus_t bus)
{
    assert(bus == MY_I2C_BUS_EEPROM && eeprom_active != 0);
    eeprom_active = 0;
}

uint8_t my_i2c_probe(my_i2c_bus_t bus, uint8_t device)
{
    assert(bus == MY_I2C_BUS_EEPROM && device == BL24C02_ADDR && eeprom_active != 0);
    return eeprom_missing;
}

uint8_t my_i2c_read_bytes(my_i2c_bus_t bus, uint8_t device, uint8_t address, uint8_t *data, uint16_t length)
{
    assert(my_i2c_probe(bus, device) == 0 && address + length <= sizeof(eeprom));
    if (eeprom_read_fail != 0)
    {
        return 1;
    }
    memcpy(data, eeprom + address, length);
    return 0;
}

uint8_t my_i2c_write_bytes(my_i2c_bus_t bus, uint8_t device, uint8_t address, const uint8_t *data, uint16_t length)
{
    assert(my_i2c_probe(bus, device) == 0 && address == BOOTLOADER_EEPROM_FLAG_ADDR && length == 4);
    ++eeprom_writes;
    if (eeprom_write_fail != 0)
    {
        return 1;
    }
    if (eeprom_partial != 0)
    {
        memcpy(eeprom + address, data, eeprom_partial);
        return 1;
    }
    if (eeprom_protected == 0)
    {
        memcpy(eeprom + address, data, length);
    }
    if (eeprom_corrupt != 0)
    {
        eeprom[address + 3] ^= 1;
    }
    return 0;
}

uint32_t HAL_GetTick(void)
{
    if (fixed_tick != 0)
    {
        return tick;
    }
    return ble_transport_get_tick();
}

static const uint32_t sector_addresses[] =
    {
        0x08000000, 0x08004000, 0x08008000, 0x0800C000, 0x08010000,
        0x08020000, 0x08040000, 0x08060000, 0x08080000};

HAL_StatusTypeDef HAL_FLASH_Unlock(void)
{
    flash_locked = 0;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FLASH_Lock(void)
{
    flash_locked = 1;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *erase, uint32_t *error)
{
    assert(!flash_locked && erase->Sector >= 2 && erase->Sector + erase->NbSectors <= 7);
    assert(memcmp(eeprom + BOOTLOADER_EEPROM_FLAG_ADDR, "\x42\x4C\x5A\xA5", 4) == 0);
    ++flash_erases;
    *error = 0xFFFFFFFFU;
    erase_sector = erase->Sector;
    erase_count = erase->NbSectors;
    if (erase_fail != 0)
    {
        return HAL_ERROR;
    }
    memset((void *)(uintptr_t)sector_addresses[erase_sector], 0xFF,
           sector_addresses[erase_sector + erase_count] - sector_addresses[erase_sector]);
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t address, uint64_t value)
{
    uint16_t *destination = (uint16_t *)(uintptr_t)address;
    assert(!flash_locked && type == FLASH_TYPEPROGRAM_HALFWORD && (address & 1) == 0);
    assert(address >= APP_START_ADDR && address + 2 <= APP_FLASH_END_ADDR);
    if (program_fail != 0)
    {
        return HAL_ERROR;
    }
    assert((*destination & value) == value);
    *destination = (uint16_t)value;
    if (corrupt_write != 0)
    {
        *destination ^= 1;
    }
    ++program_count;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size)
{
    assert(uart == &huart1 && !rx_armed && size == BOOTLOADER_DMA_SIZE);
    if (dma_fail != 0)
    {
        return (HAL_StatusTypeDef)dma_fail;
    }
    test_dma_data = data;
    test_dma_position = 0;
    test_dma_remaining = size;
    test_dma_tc = 0;
    return HAL_OK;
}

void NVIC_SystemReset(void)
{
    assert(reset_expected != 0);
    longjmp(reset_target, 1);
}

void int_bootloader_jump_to_app(void)
{
    assert(int_bootloader_app_is_valid() != 0);
    ++jump_count;
}

static void dma_feed(const uint8_t *data, uint32_t size, uint8_t defer_tc)
{
    uint32_t i;
    for (i = 0; i < size; ++i)
    {
        assert(int_bootloader_rx_dma_active() != 0);
        test_dma_data[test_dma_position++] = data[i];
        test_dma_remaining = BOOTLOADER_DMA_SIZE - test_dma_position;
        if (test_dma_position == BOOTLOADER_DMA_HALF_SIZE)
        {
            HAL_UART_RxHalfCpltCallback(&huart1);
        }
        if (test_dma_position == BOOTLOADER_DMA_SIZE)
        {
            test_dma_position = 0;
            test_dma_remaining = BOOTLOADER_DMA_SIZE;
            test_dma_tc = 1;
            if (defer_tc == 0)
            {
                test_dma_tc = 0;
                HAL_UART_RxCpltCallback(&huart1);
            }
        }
    }
}

static void drain(void)
{
    unsigned i;
    assert(int_bootloader_process() == 0);
    tick += BOOTLOADER_RX_IDLE_MS;
    for (i = 0; i < 20; ++i)
    {
        assert(int_bootloader_process() == 0);
    }
}

static void make_image(uint8_t *image, uint32_t size)
{
    uint32_t i;
    uint32_t stack = 0x20020000;
    uint32_t reset = APP_START_ADDR + 9;
    for (i = 0; i < size; ++i)
    {
        image[i] = (uint8_t)(i * 37 + 11);
    }
    memcpy(image, &stack, 4);
    memcpy(image + 4, &reset, 4);
}

static void erase_tests(void)
{
    unsigned i;
    uint32_t boundary;
    uint32_t length;
    memset((void *)(uintptr_t)FLASH_BASE, 0x5A, APP_FLASH_END_ADDR - FLASH_BASE);
    assert(int_bootloader_erase(0) == 1);
    assert(int_bootloader_erase(7) == 1);
    assert(int_bootloader_erase(APP_MAX_SIZE + 1) == 1);
    assert(int_bootloader_erase(UINT32_MAX) == 1);
    for (i = 3; i <= 7; ++i)
    {
        boundary = sector_addresses[i];
        length = boundary - APP_START_ADDR;
        assert(int_bootloader_erase(length) == 0);
        assert(erase_sector == 2 && erase_count == i - 2 && flash_locked);
        if (i < 7)
        {
            assert(int_bootloader_erase(length + 1) == 0);
            assert(erase_count == i - 1);
        }
    }
    for (i = 0; i < APP_START_ADDR - FLASH_BASE; ++i)
    {
        assert(*(uint8_t *)(uintptr_t)(FLASH_BASE + i) == 0x5A);
    }
    erase_fail = 1;
    assert(int_bootloader_erase(100) == 1 && flash_locked);
    erase_fail = 0;
    puts("PASS: image-size erase boundaries (16/64/128 KiB sectors), exact end, maximum image and boot protection");
}

static void transfer_test(uint32_t length, uint32_t chunk, uint32_t crc)
{
    uint8_t *image = malloc(length);
    uint32_t offset = 0;
    uint32_t count;
    assert(image != NULL);
    make_image(image, length);
    assert(int_bootloader_erase(length) == 0);
    assert(bootloader_start_dma(length) == 0);
    int_bootloader_expect_crc(crc);
    while (offset < length)
    {
        count = length - offset;
        if (count > chunk)
        {
            count = chunk;
        }
        dma_feed(image + offset, count, 0);
        drain();
        offset += count;
        assert(int_bootloader_app_is_valid() == 0);
    }
    assert(uart1_rx_full_len == length && uart1_rx_idle && flash_locked);
    assert(int_bootloader_finish() == 0 && flash_locked && !int_bootloader_rx_dma_active());
    assert(flash_write_offset == length && int_bootloader_app_is_valid());
    assert(memcmp(image, (void *)(uintptr_t)APP_START_ADDR, length) == 0);
    if ((length & 1) != 0)
    {
        assert(*(uint8_t *)(uintptr_t)(APP_START_ADDR + length) == 0xFF);
    }
    free(image);
}

static void eeprom_flag_tests(void)
{
    uint8_t image[101];
    uint8_t *flag = eeprom + BOOTLOADER_EEPROM_FLAG_ADDR;
    const uint8_t valid[4] = {0x42, 0x4C, 0xA5, 0x5A};
    uint32_t before = eeprom_writes;
    uint32_t erased = flash_erases;
    uint32_t started;
    uint8_t i;
    make_image(image, sizeof(image));
    memcpy((void *)(uintptr_t)APP_START_ADDR, image, sizeof(image));
    memset(flag, 0xFF, 4);
    assert(!int_bootloader_app_is_valid());
    memset(flag, 0, 4);
    assert(!int_bootloader_app_is_valid());
    memcpy(flag, valid, 4);
    assert(int_bootloader_app_is_valid());
    for (i = 0; i < 4; ++i)
    {
        flag[i] ^= 1;
        assert(!int_bootloader_app_is_valid());
        flag[i] ^= 1;
    }
    eeprom_read_fail = 1;
    assert(!int_bootloader_app_is_valid());
    eeprom_read_fail = 0;
    *(uint32_t *)(uintptr_t)APP_START_ADDR = UINT32_MAX;
    assert(!int_bootloader_app_is_valid());
    memcpy((void *)(uintptr_t)APP_START_ADDR, image, sizeof(image));
    assert(eeprom_writes == before); /* 普通启动检查不消耗写寿命。 */

    eeprom_missing = 1;
    tick = UINT32_MAX - 5;
    started = tick;
    assert(!int_bootloader_app_is_valid());
    assert((uint32_t)(tick - started) == 10 && eeprom_active == 0);
    assert(int_bootloader_erase(sizeof(image)) == 1);
    eeprom_missing = 0;
    eeprom_write_fail = 1;
    assert(int_bootloader_erase(sizeof(image)) == 1);
    eeprom_write_fail = 0;
    eeprom_protected = 1;
    assert(int_bootloader_erase(sizeof(image)) == 1);
    eeprom_protected = 0;
    eeprom_read_fail = 1;
    assert(int_bootloader_erase(sizeof(image)) == 1);
    eeprom_read_fail = 0;
    for (i = 1; i < 4; ++i)
    {
        memcpy(flag, valid, 4);
        eeprom_partial = i;
        assert(int_bootloader_erase(sizeof(image)) == 1);
    }
    eeprom_partial = 0;
    assert(flash_erases == erased);
    assert(memcmp((void *)(uintptr_t)APP_START_ADDR, image, sizeof(image)) == 0);

    /* 模拟擦除失败后重启：原APP向量仍合法，但升级中标志禁止启动。 */
    erase_fail = 1;
    assert(int_bootloader_erase(sizeof(image)) == 1);
    erase_fail = 0;
    assert(!int_bootloader_app_is_valid());
    assert(memcmp(flag, "\x42\x4C\x5A\xA5", 4) == 0);

    /* 正文和向量已完整时，最终标志写失败或读回不符仍不能报成功。 */
    for (i = 0; i < 3; ++i)
    {
        assert(int_bootloader_erase(sizeof(image)) == 0);
        assert(bootloader_start_dma(sizeof(image)) == 0);
        int_bootloader_expect_crc(TEST_CRC_101);
        dma_feed(image, sizeof(image), 0);
        drain();
        if (i == 0)
        {
            eeprom_write_fail = 1;
        }
        else if (i == 1)
        {
            eeprom_protected = 1;
        }
        else
        {
            eeprom_corrupt = 1;
        }
        assert(int_bootloader_finish() == 1);
        eeprom_write_fail = eeprom_protected = eeprom_corrupt = 0;
        assert(memcmp((void *)(uintptr_t)APP_START_ADDR, image, sizeof(image)) == 0);
        assert(!int_bootloader_app_is_valid() && eeprom_active == 0);
    }
    transfer_test(sizeof(image), 51, TEST_CRC_101);
    assert(memcmp(flag, valid, 4) == 0 && int_bootloader_app_is_valid());
    puts("PASS: EEPROM boot flags, blank/corrupt/read failure, bounded timeout/wrap, erase ordering, torn writes, final commit failure and retry");
}

static void receive_error_tests(void)
{
    uint8_t image[BOOTLOADER_DMA_SIZE + 16];
    uint32_t before;
    make_image(image, sizeof(image));
    assert(int_bootloader_erase(sizeof(image)) == 0);
    assert(bootloader_start_dma(sizeof(image)) == 0);
    dma_feed(image, BOOTLOADER_DMA_SIZE, 0);
    before = program_count;
    assert(int_bootloader_process() == 1 && uart1_rx_overflow_count != 0 && program_count == before);

    assert(bootloader_start_dma(100) == 0);
    dma_feed(image, 101, 0);
    assert(int_bootloader_process() == 1);
    assert(bootloader_start_dma(100) == 0);
    dma_feed(image, 99, 0);
    drain();
    assert(int_bootloader_finish() == 1 && !int_bootloader_app_is_valid());

    assert(int_bootloader_erase(100) == 0);
    assert(bootloader_start_dma(100) == 0);
    HAL_UART_ErrorCallback(&huart1);
    assert(uart1_rx_error_count == 1 && int_bootloader_process() == 1 && !rx_armed);

    assert(bootloader_start_dma(100) == 0);
    dma_feed(image, 100, 0);
    tick += 20;
    assert(int_bootloader_process() == 0);
    program_fail = 1;
    tick += 20;
    assert(int_bootloader_process() == 1);
    program_fail = 0;

    assert(int_bootloader_erase(100) == 0);
    assert(bootloader_start_dma(100) == 0);
    dma_feed(image, 100, 0);
    tick += 20;
    assert(int_bootloader_process() == 0);
    corrupt_write = 1;
    tick += 20;
    assert(int_bootloader_process() == 1 && flash_locked);
    corrupt_write = 0;

    assert(int_bootloader_erase(sizeof(image)) == 0);
    assert(bootloader_start_dma(sizeof(image)) == 0);
    dma_feed(image, BOOTLOADER_DMA_HALF_SIZE, 0);
    drain();
    dma_feed(image + BOOTLOADER_DMA_HALF_SIZE, BOOTLOADER_DMA_HALF_SIZE, 1);
    assert(int_bootloader_process() == 0);
    assert(uart1_rx_full_len == BOOTLOADER_DMA_SIZE); /* TC中断延迟时快照补上回绕。 */
    test_dma_tc = 0;
    HAL_UART_RxCpltCallback(&huart1);
    int_bootloader_stop();

    for (dma_fail = HAL_ERROR; dma_fail <= HAL_TIMEOUT; ++dma_fail)
    {
        assert(bootloader_start_dma(100) == 1 && !int_bootloader_rx_dma_active());
    }
    dma_fail = 0;
    puts("PASS: overflow, extra/missing bytes, UART/DMA failure, programming/readback failure and delayed TC snapshot");
}

static void command_feed(const char *text)
{
    while (*text != '\0')
    {
        inject((uint8_t)*text++);
    }
    app_bootloader_work();
}

static void state_machine_tests(void)
{
    uint8_t image[101];
    uint32_t i;
    int_bootloader_stop();
    connected = 0;
    fixed_tick = 0;
    assert(kt6368a_init() == HAL_OK);
    fixed_tick = 1;
    connected = 1;
    app_bootloader_init();
    command_feed("start:999999999999999999999\n");
    assert(boot_state == BOOTLOADER_STATE_INIT && rx_armed);
    command_feed("start:-1\n");
    assert(boot_state == BOOTLOADER_STATE_INIT);
    command_feed("sta");
    command_feed("rt:101");
    tick += BOOTLOADER_RX_IDLE_MS;
    app_bootloader_work();
    assert(boot_state == BOOTLOADER_STATE_RUN);
    app_bootloader_work();
    assert(boot_state == BOOTLOADER_STATE_REC_DATA && !rx_armed && int_bootloader_rx_dma_active());
    make_image(image, sizeof(image));
    dma_feed(image, 51, 0);
    tick = UINT32_MAX - 15;
    app_bootloader_work();
    tick += 20;
    app_bootloader_work();
    assert(uart1_rx_idle);
    dma_feed(image + 51, 50, 0);
    app_bootloader_work();
    assert(!uart1_rx_idle);
    tick += 20;
    for (i = 0; i < 4; ++i)
    {
        app_bootloader_work();
    }
    assert(uart1_rx_idle && boot_state == BOOTLOADER_STATE_REC_DATA);
    tick += BOOTLOADER_COMPLETE_IDLE_MS;
    app_bootloader_work();
    assert(boot_state == BOOTLOADER_STATE_CHECK_DATA);
    app_bootloader_work();
    assert(boot_state == BOOTLOADER_STATE_JUMP_TO_APP);
    assert(memcmp(image, (void *)(uintptr_t)APP_START_ADDR, sizeof(image)) == 0);
    app_bootloader_work();
    assert(jump_count == 0);
    tick += 500;
    reset_expected = 1;
    if (setjmp(reset_target) == 0)
    {
        app_bootloader_work();
        assert(0 && "jump stub returns into reset path");
    }
    reset_expected = 0;
    assert(jump_count == 1);

    /* 缺包超时必须复位，不能提交向量或进入成功状态。 */
    int_bootloader_stop();
    connected = 0;
    fixed_tick = 0;
    assert(kt6368a_init() == HAL_OK);
    fixed_tick = 1;
    connected = 1;
    app_bootloader_init();
    command_feed("start:101\n");
    app_bootloader_work();
    dma_feed(image, 100, 0);
    app_bootloader_work();
    tick += 20;
    app_bootloader_work();
    app_bootloader_work();
    tick += BOOTLOADER_TRANSFER_TIMEOUT_MS - 21;
    app_bootloader_work();
    assert(boot_state == BOOTLOADER_STATE_REC_DATA);
    ++tick;
    reset_expected = 1;
    if (setjmp(reset_target) == 0)
    {
        app_bootloader_work();
        assert(0 && "incomplete image must reset");
    }
    reset_expected = 0;
    assert(!int_bootloader_app_is_valid());
    puts("PASS: legacy command, malformed length, erase-before-DMA, length completion, tick wrap, SPP reply window and partial transfer timeout");
}

static void crc_error_tests(void)
{
    uint8_t image[101];
    unsigned i;
    const unsigned positions[] = {4, 50, 100};
    for (i = 0; i < sizeof(positions) / sizeof(positions[0]); ++i)
    {
        make_image(image, sizeof(image));
        image[positions[i]] ^= 0x10;
        assert(int_bootloader_erase(sizeof(image)) == 0);
        assert(bootloader_start_dma(sizeof(image)) == 0);
        int_bootloader_expect_crc(TEST_CRC_101);
        dma_feed(image, sizeof(image), 0);
        drain();
        assert(int_bootloader_finish() == 1);
        assert(!int_bootloader_app_is_valid());
        assert(*(uint32_t *)(uintptr_t)APP_START_ADDR == UINT32_MAX);
    }
    puts("PASS: zlib CRC detects equal-length vector/body/odd-tail corruption before vector commit");
}

static void start_command(const char *text)
{
    int_bootloader_stop();
    connected = 0;
    fixed_tick = 0;
    assert(kt6368a_init() == HAL_OK);
    fixed_tick = 1;
    connected = 1;
    app_bootloader_init();
    command_feed(text);
    assert(boot_state == BOOTLOADER_STATE_RUN);
    app_bootloader_work();
    assert(boot_state == BOOTLOADER_STATE_REC_DATA);
}

static void crc_command_tests(void)
{
    uint8_t image[101];
    char text[64];
    unsigned i;
    const char *invalid[] =
        {
            "start:101:\n", "start:101:1234567\n", "start:101:123456789\n",
            "start:101:1234567g\n", "start::12345678\n", "start:101:12345678:x\n"};
    int_bootloader_stop();
    connected = 0;
    fixed_tick = 0;
    assert(kt6368a_init() == HAL_OK);
    fixed_tick = 1;
    connected = 1;
    app_bootloader_init();
    for (i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i)
    {
        command_feed(invalid[i]);
        assert(boot_state == BOOTLOADER_STATE_INIT && rx_armed);
    }
    for (i = 0; i < 2; ++i)
    {
        if (i == 0)
        {
            snprintf(text, sizeof(text), "start:101:%08lx\n", (unsigned long)TEST_CRC_101);
        }
        else
        {
            snprintf(text, sizeof(text), "start:101:%08lX\n", (unsigned long)TEST_CRC_101);
        }
        start_command(text);
        make_image(image, sizeof(image));
        dma_feed(image, sizeof(image), 0);
        app_bootloader_work();
        tick += BOOTLOADER_RX_IDLE_MS;
        app_bootloader_work();
        tick += BOOTLOADER_COMPLETE_IDLE_MS - 1;
        app_bootloader_work();
        assert(boot_state == BOOTLOADER_STATE_REC_DATA);
        ++tick;
        app_bootloader_work();
        assert(boot_state == BOOTLOADER_STATE_CHECK_DATA);
        app_bootloader_work();
        assert(boot_state == BOOTLOADER_STATE_JUMP_TO_APP && int_bootloader_app_is_valid());
    }
    /* 全零CRC是有效字段，不能被误判成关闭CRC。 */
    start_command("start:101:00000000\n");
    make_image(image, sizeof(image));
    dma_feed(image, sizeof(image), 0);
    drain();
    assert(int_bootloader_finish() == 1 && !int_bootloader_app_is_valid());
    puts("PASS: strict CRC command parsing, upper/lower hex, 100ms completion guard and zero CRC rejection");
}

static void timeout_and_pause_tests(void)
{
    uint8_t image[101];
    make_image(image, sizeof(image));
    start_command("start:101\n");
    /* 没有收到首字节时同样计时，且跨tick回绕仍能复位。 */
    tick = UINT32_MAX - 1000;
    assert(bootloader_start_dma(sizeof(image)) == 0);
    tick += BOOTLOADER_TRANSFER_TIMEOUT_MS - 1;
    app_bootloader_work();
    assert(boot_state == BOOTLOADER_STATE_REC_DATA);
    ++tick;
    reset_expected = 1;
    if (setjmp(reset_target) == 0)
    {
        app_bootloader_work();
        assert(0 && "zero-byte transfer must reset at timeout");
    }
    reset_expected = 0;
    assert(!int_bootloader_app_is_valid());

    start_command("start:101\n");
    dma_feed(image, 51, 0);
    app_bootloader_work();
    tick += 2500;
    app_bootloader_work();
    assert(boot_state == BOOTLOADER_STATE_REC_DATA && uart1_rx_idle);
    tick += BOOTLOADER_TRANSFER_TIMEOUT_MS - 2501;
    dma_feed(image + 51, 50, 0);
    app_bootloader_work();
    assert(boot_state == BOOTLOADER_STATE_REC_DATA && !uart1_rx_idle);
    tick += BOOTLOADER_RX_IDLE_MS;
    app_bootloader_work();
    tick += BOOTLOADER_COMPLETE_IDLE_MS;
    app_bootloader_work();
    app_bootloader_work();
    assert(boot_state == BOOTLOADER_STATE_JUMP_TO_APP && int_bootloader_app_is_valid());

    start_command("start:101\n");
    dma_feed(image, sizeof(image), 0);
    drain();
    tick += BOOTLOADER_COMPLETE_IDLE_MS - 1;
    dma_feed(image, 1, 0);
    reset_expected = 1;
    if (setjmp(reset_target) == 0)
    {
        app_bootloader_work();
        assert(0 && "extra byte during completion guard must reset");
    }
    reset_expected = 0;
    assert(!int_bootloader_app_is_valid());
    puts("PASS: zero-byte timeout, timeout tick wrap, pauses over 2s, progress renewal and extra byte during completion guard");
}

int main(void)
{
    void *mapped = VirtualAlloc((void *)(uintptr_t)FLASH_BASE, BATTERY_LOG_END_ADDR - FLASH_BASE,
                                MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE);
    assert(mapped == (void *)(uintptr_t)FLASH_BASE);
    assert(ble_transport_regression_main() == 0); /* 同一套AT模拟验证升级模式开启SPP。 */
    assert(kt6368a_init() == HAL_OK);
    fixed_tick = 1;
    connected = 1;
    memset((void *)(uintptr_t)BATTERY_LOG_START_ADDR, 0xA6, BATTERY_LOG_END_ADDR - BATTERY_LOG_START_ADDR);
    memset(eeprom, 0x3C, sizeof(eeprom));
    erase_tests();
    transfer_test(101, 1, TEST_CRC_101);
    transfer_test(512, 127, TEST_CRC_512);
    transfer_test(10001, 4096, TEST_CRC_10001);
    transfer_test(APP_MAX_SIZE, 4001, TEST_CRC_360448);
    for (uint32_t address = BATTERY_LOG_START_ADDR; address < BATTERY_LOG_END_ADDR; ++address)
    {
        assert(*(const uint8_t *)(uintptr_t)address == 0xA6);
    }
    puts("PASS: byte-exact odd/even, fragmented, multiple DMA wraps, full 352 KiB image, deferred vectors and odd tail");
    eeprom_flag_tests();
    receive_error_tests();
    crc_error_tests();
    state_machine_tests();
    crc_command_tests();
    timeout_and_pause_tests();
    for (uint32_t address = 0; address < sizeof(eeprom); ++address)
    {
        if (address < BOOTLOADER_EEPROM_FLAG_ADDR || address >= BOOTLOADER_EEPROM_FLAG_ADDR + 4)
        {
            assert(eeprom[address] == 0x3C);
        }
    }
    assert(VirtualFree(mapped, 0, MEM_RELEASE));
    return 0;
}
