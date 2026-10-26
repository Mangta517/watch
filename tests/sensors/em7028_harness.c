/* 编译真实 EM7028 驱动；模拟手册随机读时序、寄存器和真实 HEART_LED 宏。 */
#include "em7028.h"
#include "my_i2c.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

GPIO_TypeDef host_gpio_a, host_gpio_b;
static uint8_t registers[256];
static uint8_t phase, selected_reg, rollover, ignore_ctrl, ignore_mode;
static uint32_t tick;
static unsigned ack_count, write_count, fail_ack, fail_write;
static unsigned pid_reads, bad_pid_reads;
static GPIO_PinState light;

static void mock_reset(void)
{
    memset(registers, 0, sizeof(registers));
    registers[0x00] = 0x36;
    registers[0x01] = 0xFF; /* 两种模式及保留位均置位，用于检查读后修改 */
    registers[0x0E] = 0xFF;
    phase = 0;
    tick = 0;
    ack_count = 0;
    write_count = 0;
    fail_ack = 0;
    fail_write = 0;
    pid_reads = bad_pid_reads = 0;
    rollover = 0;
    ignore_ctrl = 0;
    ignore_mode = 0;
    light = GPIO_PIN_RESET;
}

uint32_t HAL_GetTick(void)
{
    return tick;
}

void HAL_Delay(uint32_t ms)
{
    tick += ms;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    assert(port == GPIOB && port == HEART_LED_GPIO_Port);
    assert(pin == (1U << 15) && pin == HEART_LED_Pin);
    light = state;
}

void my_i2c_start(my_i2c_bus_t bus)
{
    /* 每次 START 前必须已有 STOP，禁止误用通用连续读时序。 */
    assert(bus == MY_I2C_BUS_SENSOR && phase == 0);
    phase = 1;
}

void my_i2c_stop(my_i2c_bus_t bus)
{
    assert(bus == MY_I2C_BUS_SENSOR && phase != 0);
    phase = 0;
}

void my_i2c_write_byte(my_i2c_bus_t bus, uint8_t data)
{
    assert(bus == MY_I2C_BUS_SENSOR);
    if (phase == 1)
    {
        assert(data == 0x48 || data == 0x49);
        if (data == 0x48)
        {
            phase = 2;
        }
        else
        {
            phase = 4;
        }
    }
    else
    {
        assert(phase == 2);
        selected_reg = data;
        phase = 3;
    }
}

uint8_t my_i2c_wait_ack(my_i2c_bus_t bus)
{
    assert(bus == MY_I2C_BUS_SENSOR && phase >= 2 && phase <= 4);
    if (++ack_count == fail_ack)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

uint8_t my_i2c_read_byte(my_i2c_bus_t bus, uint8_t ack)
{
    uint8_t value;
    assert(bus == MY_I2C_BUS_SENSOR && phase == 4 && ack == 1);
    assert(selected_reg != 0x30 && selected_reg != 0x31);
    value = registers[selected_reg];
    if (selected_reg == 0x00)
    {
        ++pid_reads;
        if (bad_pid_reads != 0)
        {
            --bad_pid_reads;
            value = 0xFF;
        }
    }
    if (selected_reg == 0x28 && rollover != 0)
    {
        ++registers[0x29];
        rollover = 0;
    }
    phase = 5;
    return value;
}

uint8_t my_i2c_write_bytes(my_i2c_bus_t bus, uint8_t dev, uint8_t reg, const uint8_t *data, uint16_t len)
{
    assert(bus == MY_I2C_BUS_SENSOR && dev == 0x24 && len == 1 && data != NULL && phase == 0);
    if (++write_count == fail_write)
    {
        return 1;
    }
    assert(reg != 0x0F); /* 不依赖手册未给出的软复位恢复时间 */
    if ((reg != 0x0D || ignore_ctrl == 0) && (reg != 0x01 || ignore_mode == 0))
    {
        registers[reg] = data[0];
    }
    return 0;
}

int main(void)
{
    uint16_t raw = 0xCAFE;
    em7028_fault fault;
    unsigned i, init_acks, init_writes;
    mock_reset();
    fail_ack = 1;
    assert(em7028_start() == EM7028_ERROR);
    em7028_get_fault(&fault);
    assert(fault.kind == EM7028_FAULT_READ && fault.reg == 0x00);
    assert(em7028_stop() == EM7028_NOT_READY && light == GPIO_PIN_RESET);
    assert(em7028_read_raw(&raw) == EM7028_NOT_READY && raw == 0xCAFE);
    assert(em7028_read_raw(NULL) == EM7028_ERROR);

    /* 偶发FF后读到36可继续；持续FF必须有界失败，不能带错PID写配置。 */
    for (i = 1; i <= 2; ++i)
    {
        mock_reset();
        bad_pid_reads = i;
        assert(em7028_init() == EM7028_OK && pid_reads == i + 1 && light == GPIO_PIN_RESET);
        em7028_get_fault(&fault);
        assert(fault.kind == EM7028_FAULT_NONE);
    }
    mock_reset();
    registers[0x00] = 0xFF;
    assert(em7028_init() == EM7028_ERROR && pid_reads == 3 && write_count == 0 && phase == 0);
    em7028_get_fault(&fault);
    assert(fault.kind == EM7028_FAULT_PID && fault.reg == 0 && fault.value == 0xFF);
    mock_reset();
    bad_pid_reads = 1;
    fail_ack = 4;
    assert(em7028_init() == EM7028_ERROR && write_count == 0 && phase == 0);
    em7028_get_fault(&fault);
    assert(fault.kind == EM7028_FAULT_READ && fault.reg == 0);

    mock_reset();
    assert(em7028_init() == EM7028_OK);
    init_acks = ack_count;
    init_writes = write_count;
    assert(registers[0x01] == 0x77 && registers[0x0E] == 0x7F && registers[0x0D] == 0x47);
    assert(light == GPIO_PIN_RESET && phase == 0 && tick == 0);
    assert(em7028_read_raw(&raw) == EM7028_NOT_READY);
    assert(em7028_start() == EM7028_OK && registers[0x01] == 0x7F && light == GPIO_PIN_SET);
    assert(em7028_read_raw(&raw) == EM7028_NOT_READY && raw == 0xCAFE);
    tick += EM7028_SAMPLE_PERIOD_MS - 1;
    assert(em7028_read_raw(&raw) == EM7028_NOT_READY);
    ++tick;
    registers[0x28] = 0x34;
    registers[0x29] = 0x12;
    assert(em7028_read_raw(&raw) == EM7028_OK && raw == 0x1234);
    assert(em7028_start() == EM7028_OK);
    assert(em7028_read_raw(&raw) == EM7028_OK); /* 重复启动不重置首帧等待 */
    rollover = 1;
    raw = 0xCAFE;
    assert(em7028_read_raw(&raw) == EM7028_NOT_READY && raw == 0xCAFE);
    em7028_get_fault(&fault);
    assert(fault.kind == EM7028_FAULT_DATA_CHANGED && fault.reg == 0x29);
    assert(em7028_read_raw(&raw) == EM7028_OK && raw == 0x1334);

    /* 三个单字节事务的各 ACK 阶段失败均终止总线，且不覆盖上次输出。 */
    for (i = 1; i <= 9; ++i)
    {
        fail_ack = ack_count + i;
        raw = 0xCAFE;
        assert(em7028_read_raw(&raw) == EM7028_ERROR && raw == 0xCAFE && phase == 0);
        em7028_get_fault(&fault);
        if (i >= 4 && i <= 6)
        {
            assert(fault.kind == EM7028_FAULT_READ && fault.reg == 0x28);
        }
        else
        {
            assert(fault.kind == EM7028_FAULT_READ && fault.reg == 0x29);
        }
    }
    fail_ack = 0;
    fail_write = write_count + 1;
    assert(em7028_stop() == EM7028_ERROR && light == GPIO_PIN_RESET);
    assert(em7028_read_raw(&raw) == EM7028_NOT_READY);
    fail_write = 0;
    assert(em7028_stop() == EM7028_OK && registers[0x01] == 0x77);
    registers[0x0D] = 0x32; /* 模拟停止后芯片复位，不能继续信任旧 ready */
    assert(em7028_start() == EM7028_OK && registers[0x0D] == 0x47);
    assert(em7028_stop() == EM7028_OK);
    fail_write = write_count + 1;
    assert(em7028_start() == EM7028_ERROR && light == GPIO_PIN_RESET);
    fail_write = 0;
    tick = UINT32_MAX - 10;
    assert(em7028_start() == EM7028_OK);
    tick += EM7028_SAMPLE_PERIOD_MS;
    assert(em7028_read_raw(&raw) == EM7028_OK); /* HAL tick 回绕 */
    assert(em7028_stop() == EM7028_OK);

    for (i = 1; i <= init_acks; ++i)
    {
        mock_reset();
        fail_ack = i;
        assert(em7028_init() == EM7028_ERROR && light == GPIO_PIN_RESET && phase == 0);
        fail_ack = 0;
        assert(em7028_start() == EM7028_OK); /* 用户重试 START 能恢复此前初始化失败 */
        assert(em7028_stop() == EM7028_OK);
    }
    for (i = 1; i <= init_writes; ++i)
    {
        mock_reset();
        fail_write = i;
        assert(em7028_init() == EM7028_ERROR && light == GPIO_PIN_RESET);
        em7028_get_fault(&fault);
        assert(fault.kind == EM7028_FAULT_WRITE);
        fail_write = 0;
        assert(em7028_start() == EM7028_OK);
        assert(em7028_stop() == EM7028_OK);
    }
    mock_reset();
    registers[0x00] = 0x7E; /* 用户实板报告值；不能误当作正确 PID 继续配置 */
    assert(em7028_init() == EM7028_ERROR && write_count == 0);
    em7028_get_fault(&fault);
    assert(fault.kind == EM7028_FAULT_PID && fault.reg == 0 && fault.value == 0x7E);
    mock_reset();
    ignore_ctrl = 1;
    assert(em7028_init() == EM7028_ERROR && light == GPIO_PIN_RESET);
    em7028_get_fault(&fault);
    assert(fault.kind == EM7028_FAULT_CONFIG && fault.reg == 0x0D);
    mock_reset();
    ignore_mode = 1;
    assert(em7028_init() == EM7028_ERROR && light == GPIO_PIN_RESET);
    em7028_get_fault(&fault);
    assert(fault.kind == EM7028_FAULT_CONFIG && fault.reg == 0x01 && fault.value == 0xFF);
    mock_reset();
    assert(em7028_init() == EM7028_OK && em7028_start() == EM7028_OK);
    registers[0x00] = 0x35;
    assert(em7028_init() == EM7028_ERROR && light == GPIO_PIN_RESET);
    assert(em7028_read_raw(&raw) == EM7028_NOT_READY);
    puts("PASS: EM7028 address/STOP-START/NACK, PID/config/reserved bits, start retries failed init, standby/start/stop, little endian, "
         "first frame/tick wrap, torn read and fault diagnostics.");
    return 0;
}

uint8_t my_i2c_prepare(my_i2c_bus_t bus)
{
    assert(bus == MY_I2C_BUS_SENSOR && phase == 0);
    return 0;
}
