/* 复用现有蓝牙模块/AT模拟，执行真实驱动的IT到DMA交接及回调路由。 */
#define main ble_transport_regression_main
#define HAL_GetTick ble_transport_get_tick
#include "../ble/ble_harness.c"
#undef main
#undef HAL_GetTick
#include "app_bootloader.h"
#include "crc_vectors.h"
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

uint32_t HAL_GetTick(void)
{
    if (fixed_tick != 0U)
    {
        return tick;
    }
    return ble_transport_get_tick();
}

static const uint32_t sector_addresses[] =
{
    0x08000000U, 0x08004000U, 0x08008000U, 0x0800C000U, 0x08010000U,
    0x08020000U, 0x08040000U, 0x08060000U, 0x08080000U
};

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
    assert(!flash_locked && erase->Sector >= 2U && erase->Sector + erase->NbSectors <= 8U);
    *error = 0xFFFFFFFFU;
    erase_sector = erase->Sector;
    erase_count = erase->NbSectors;
    if (erase_fail != 0U)
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
    assert(!flash_locked && type == FLASH_TYPEPROGRAM_HALFWORD && (address & 1U) == 0U);
    assert(address >= APP_START_ADDR && address + 2U <= APP_FLASH_END_ADDR);
    if (program_fail != 0U)
    {
        return HAL_ERROR;
    }
    assert((*destination & value) == value);
    *destination = (uint16_t)value;
    if (corrupt_write != 0U)
    {
        *destination ^= 1U;
    }
    ++program_count;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size)
{
    assert(uart == &huart1 && !rx_armed && size == BOOTLOADER_DMA_SIZE);
    if (dma_fail != 0U)
    {
        return HAL_ERROR;
    }
    test_dma_data = data;
    test_dma_position = 0;
    test_dma_remaining = size;
    test_dma_tc = 0;
    return HAL_OK;
}

void NVIC_SystemReset(void)
{
    assert(reset_expected != 0U);
    longjmp(reset_target, 1);
}

void int_bootloader_jump_to_app(void)
{
    assert(int_bootloader_app_is_valid() != 0U);
    ++jump_count;
}

static void dma_feed(const uint8_t *data, uint32_t size, uint8_t defer_tc)
{
    uint32_t i;
    for (i = 0; i < size; ++i)
    {
        assert(int_bootloader_rx_dma_active() != 0U);
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
            if (defer_tc == 0U)
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
    assert(int_bootloader_process() == HAL_OK);
    tick += BOOTLOADER_RX_IDLE_MS;
    for (i = 0; i < 20U; ++i)
    {
        assert(int_bootloader_process() == HAL_OK);
    }
}

static void make_image(uint8_t *image, uint32_t size)
{
    uint32_t i;
    uint32_t stack = 0x20020000U;
    uint32_t reset = APP_START_ADDR + 9U;
    for (i = 0; i < size; ++i)
    {
        image[i] = (uint8_t)(i * 37U + 11U);
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
    assert(int_bootloader_erase(0) == HAL_ERROR);
    assert(int_bootloader_erase(7) == HAL_ERROR);
    assert(int_bootloader_erase(APP_MAX_SIZE + 1U) == HAL_ERROR);
    assert(int_bootloader_erase(UINT32_MAX) == HAL_ERROR);
    for (i = 3; i <= 8; ++i)
    {
        boundary = sector_addresses[i];
        length = boundary - APP_START_ADDR;
        assert(int_bootloader_erase(length) == HAL_OK);
        assert(erase_sector == 2U && erase_count == i - 2U && flash_locked);
        if (i < 8)
        {
            assert(int_bootloader_erase(length + 1U) == HAL_OK);
            assert(erase_count == i - 1U);
        }
    }
    for (i = 0; i < APP_START_ADDR - FLASH_BASE; ++i)
    {
        assert(*(uint8_t *)(uintptr_t)(FLASH_BASE + i) == 0x5A);
    }
    erase_fail = 1;
    assert(int_bootloader_erase(100) == HAL_ERROR && flash_locked);
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
    assert(int_bootloader_erase(length) == HAL_OK);
    assert(bootloader_start_dma(length) == HAL_OK);
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
        assert(int_bootloader_app_is_valid() == 0U);
    }
    assert(uart1_rx_full_len == length && uart1_rx_idle && flash_locked);
    assert(int_bootloader_finish() == HAL_OK && flash_locked && !int_bootloader_rx_dma_active());
    assert(flash_write_offset == length && int_bootloader_app_is_valid());
    assert(memcmp(image, (void *)(uintptr_t)APP_START_ADDR, length) == 0);
    if ((length & 1U) != 0U)
    {
        assert(*(uint8_t *)(uintptr_t)(APP_START_ADDR + length) == 0xFF);
    }
    free(image);
}

static void receive_error_tests(void)
{
    uint8_t image[BOOTLOADER_DMA_SIZE + 16U];
    uint32_t before;
    make_image(image, sizeof(image));
    assert(int_bootloader_erase(sizeof(image)) == HAL_OK);
    assert(bootloader_start_dma(sizeof(image)) == HAL_OK);
    dma_feed(image, BOOTLOADER_DMA_SIZE, 0);
    before = program_count;
    assert(int_bootloader_process() == HAL_ERROR && uart1_rx_overflow_count != 0U && program_count == before);

    assert(bootloader_start_dma(100) == HAL_OK);
    dma_feed(image, 101, 0);
    assert(int_bootloader_process() == HAL_ERROR);
    assert(bootloader_start_dma(100) == HAL_OK);
    dma_feed(image, 99, 0);
    drain();
    assert(int_bootloader_finish() == HAL_ERROR && !int_bootloader_app_is_valid());

    assert(int_bootloader_erase(100) == HAL_OK);
    assert(bootloader_start_dma(100) == HAL_OK);
    HAL_UART_ErrorCallback(&huart1);
    assert(uart1_rx_error_count == 1U && int_bootloader_process() == HAL_ERROR && !rx_armed);

    assert(bootloader_start_dma(100) == HAL_OK);
    dma_feed(image, 100, 0);
    tick += 20;
    assert(int_bootloader_process() == HAL_OK);
    program_fail = 1;
    tick += 20;
    assert(int_bootloader_process() == HAL_ERROR);
    program_fail = 0;

    assert(int_bootloader_erase(100) == HAL_OK);
    assert(bootloader_start_dma(100) == HAL_OK);
    dma_feed(image, 100, 0);
    tick += 20;
    assert(int_bootloader_process() == HAL_OK);
    corrupt_write = 1;
    tick += 20;
    assert(int_bootloader_process() == HAL_ERROR && flash_locked);
    corrupt_write = 0;

    assert(int_bootloader_erase(sizeof(image)) == HAL_OK);
    assert(bootloader_start_dma(sizeof(image)) == HAL_OK);
    dma_feed(image, BOOTLOADER_DMA_HALF_SIZE, 0);
    drain();
    dma_feed(image + BOOTLOADER_DMA_HALF_SIZE, BOOTLOADER_DMA_HALF_SIZE, 1);
    assert(int_bootloader_process() == HAL_OK);
    assert(uart1_rx_full_len == BOOTLOADER_DMA_SIZE); /* TC中断延迟时快照补上回绕。 */
    test_dma_tc = 0;
    HAL_UART_RxCpltCallback(&huart1);
    int_bootloader_stop();

    dma_fail = 1;
    assert(bootloader_start_dma(100) == HAL_ERROR && !int_bootloader_rx_dma_active());
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
    tick = UINT32_MAX - 15U;
    app_bootloader_work();
    tick += 20U;
    app_bootloader_work();
    assert(uart1_rx_idle);
    dma_feed(image + 51, 50, 0);
    app_bootloader_work();
    assert(!uart1_rx_idle);
    tick += 20U;
    for (i = 0; i < 4U; ++i)
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
    assert(jump_count == 0U);
    tick += 500U;
    reset_expected = 1;
    if (setjmp(reset_target) == 0)
    {
        app_bootloader_work();
        assert(0 && "jump stub returns into reset path");
    }
    reset_expected = 0;
    assert(jump_count == 1U);

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
    tick += 20U;
    app_bootloader_work();
    app_bootloader_work();
    tick += BOOTLOADER_TRANSFER_TIMEOUT_MS - 21U;
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
        image[positions[i]] ^= 0x10U;
        assert(int_bootloader_erase(sizeof(image)) == HAL_OK);
        assert(bootloader_start_dma(sizeof(image)) == HAL_OK);
        int_bootloader_expect_crc(TEST_CRC_101);
        dma_feed(image, sizeof(image), 0);
        drain();
        assert(int_bootloader_finish() == HAL_ERROR);
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
        "start:101:1234567g\n", "start::12345678\n", "start:101:12345678:x\n"
    };
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
    for (i = 0; i < 2U; ++i)
    {
        if (i == 0U)
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
        tick += BOOTLOADER_COMPLETE_IDLE_MS - 1U;
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
    assert(int_bootloader_finish() == HAL_ERROR && !int_bootloader_app_is_valid());
    puts("PASS: strict CRC command parsing, upper/lower hex, 100ms completion guard and zero CRC rejection");
}

static void timeout_and_pause_tests(void)
{
    uint8_t image[101];
    make_image(image, sizeof(image));
    start_command("start:101\n");
    /* 没有收到首字节时同样计时，且跨tick回绕仍能复位。 */
    tick = UINT32_MAX - 1000U;
    assert(bootloader_start_dma(sizeof(image)) == HAL_OK);
    tick += BOOTLOADER_TRANSFER_TIMEOUT_MS - 1U;
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
    tick += 2500U;
    app_bootloader_work();
    assert(boot_state == BOOTLOADER_STATE_REC_DATA && uart1_rx_idle);
    tick += BOOTLOADER_TRANSFER_TIMEOUT_MS - 2501U;
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
    tick += BOOTLOADER_COMPLETE_IDLE_MS - 1U;
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
    void *mapped = VirtualAlloc((void *)(uintptr_t)FLASH_BASE, APP_FLASH_END_ADDR - FLASH_BASE,
                               MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE);
    assert(mapped == (void *)(uintptr_t)FLASH_BASE);
    assert(ble_transport_regression_main() == 0); /* 同一套AT模拟验证升级模式开启SPP。 */
    assert(kt6368a_init() == HAL_OK);
    fixed_tick = 1;
    connected = 1;
    erase_tests();
    transfer_test(101, 1, TEST_CRC_101);
    transfer_test(512, 127, TEST_CRC_512);
    transfer_test(10001, 4096, TEST_CRC_10001);
    transfer_test(APP_MAX_SIZE, 4001, TEST_CRC_491520);
    puts("PASS: byte-exact odd/even, fragmented, multiple DMA wraps, full 480 KiB image, deferred vectors and odd tail");
    receive_error_tests();
    crc_error_tests();
    state_machine_tests();
    crc_command_tests();
    timeout_and_pause_tests();
    assert(VirtualFree(mapped, 0, MEM_RELEASE));
    return 0;
}
