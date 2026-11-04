/**
  ******************************************************************************
  * @file    app_reset.c
  * @brief   应用层-复位原因与异常现场记录 (RTC备份域 BKP1..BKP19)
  *          记录布局: 1字magic + 1字版本 + 16字 app_reset_info_struct + 1字CRC，共19字，
  *          BKP0 留给 RTC 校时标记，本模块只占用 BKP1..BKP19。
  *          数据归属: s_previous 只在开机读一次并对外发布(app_reset_get_info)；
  *          s_current 是本次运行正在更新的记录，喂狗检查点和异常现场都写它。
  *          写入时机: 开机先提交一份"本次运行中"的骨架；此后每次实际喂狗顺带推进检查点；
  *          硬件异常、LVGL断言、栈溢出时提交现场并停机，等外部看门狗复位。
  *          跨复位能力: 备份域在系统复位后保留，但被清过(掉电或RTC复位)时magic/CRC会失配，
  *          此时只保留本次读到的 RCC 复位标志，其余字段按无效处理。
  ******************************************************************************
***/
#include "app_reset.h"
#include "main.h"
#include <stddef.h>
#include <string.h>

#define APP_RESET_MAGIC        0x52535431
#define APP_RESET_VERSION      1
#define APP_RESET_RECORD_WORDS 19
#define APP_RESET_SRAM_START   0x20000000
#define APP_RESET_SRAM_END     0x20020000
#define APP_RESET_STACK_ERRORS (SCB_CFSR_MSTKERR_Msk | SCB_CFSR_MUNSTKERR_Msk | SCB_CFSR_MLSPERR_Msk | \
                                SCB_CFSR_STKERR_Msk | SCB_CFSR_UNSTKERR_Msk | SCB_CFSR_LSPERR_Msk)

static app_reset_info_struct s_previous;
static app_reset_info_struct s_current;
static uint8_t s_have_feed;
static volatile uint8_t s_stopped;

static void reset_backup_enable(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_PWREN; /* 打开PWR外设时钟，才能访问PWR寄存器。 */
    (void)RCC->APB1ENR; /* 读回一次，等时钟使能实际生效。 */
    PWR->CR |= PWR_CR_DBP; /* 置位备份域写保护解锁，允许写RTC备份寄存器。 */
    (void)PWR->CR; /* APB1分频下DBP生效有延迟，按手册先读回。 */
    RCC->BDCR |= RCC_BDCR_RTCEN; /* 使能RTC时钟，保持备份域可访问。 */
}

static volatile uint32_t *reset_backup_word(uint32_t index)
{
    /* BKP0保留给RTC校时标记；本记录仅占BKP1..19。 */
    return (volatile uint32_t *)((uintptr_t)&RTC->BKP0R + (index + 1) * sizeof(uint32_t)); /* index=0对应BKP1。 */
}

/* 反射CRC32(多项式0xEDB88320)，覆盖版本到最后一个数据字，不含magic和CRC自身。
 * 它只校验记录前后是否一致：备份域被清空和记录被写坏都会失配，无法仅凭CRC区分两者。 */
static uint32_t reset_checksum(const uint32_t *words)
{
    uint32_t crc = UINT32_MAX; /* 反射CRC32初值。 */
    uint32_t i, bit;
    /* 跳过magic(words[0])，从版本字一直算到最后一个数据字，不含CRC自身。 */
    for (i = 1; i < APP_RESET_RECORD_WORDS - 1; ++i)
    {
        crc ^= words[i]; /* 当前数据字并入CRC低32位。 */
        /* 每个字固定处理32位。 */
        for (bit = 0; bit < 32; ++bit)
        {
            /* 最低位为1时右移后异或反射多项式0xEDB88320；否则只右移一位。 */
            if ((crc & 1) != 0)
            {
                crc = (crc >> 1) ^ 0xEDB88320U;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc ^ UINT32_MAX; /* 结果取反后返回，作为记录最后一个字。 */
}

/* 先撤销magic，再写数据和CRC，最后恢复magic；未完成提交的数据不作为有效记录接受。 */
static void reset_commit(const uint32_t *words)
{
    uint32_t i;
    reset_backup_enable(); /* 先解锁备份域，后续写入才有效。 */
    *reset_backup_word(0) = 0; /* 先撤销提交标记；中途复位不能把半条记录当成有效。 */
    __DMB(); /* 保证magic清零先于数据写入完成。 */
    /* 依次写版本、16字现场和CRC(下标1..18)。 */
    for (i = 1; i < APP_RESET_RECORD_WORDS; ++i)
    {
        *reset_backup_word(i) = words[i];
    }
    __DMB(); /* 保证数据与CRC全部写完后才恢复magic。 */
    *reset_backup_word(0) = APP_RESET_MAGIC; /* 最后写magic，作为整条记录提交完成的标志。 */
    __DSB(); /* 等所有备份域写入完成再返回。 */
}

static void reset_store(void)
{
    uint32_t words[APP_RESET_RECORD_WORDS];
    words[0] = APP_RESET_MAGIC; /* 记录首字为提交标记。 */
    words[1] = APP_RESET_VERSION; /* 第二字为记录格式版本。 */
    memcpy(words + 2, &s_current, sizeof(s_current)); /* 16字现场结构体展开到words[2..17]。 */
    words[APP_RESET_RECORD_WORDS - 1] = reset_checksum(words); /* 最后一字放覆盖版本到现场数据的CRC。 */
    reset_commit(words); /* 按"先撤magic、再写数据、最后回magic"的顺序提交。 */
}

/* 开机读上一份记录：先取RCC复位标志，再整份读出BKP；只有magic、版本和CRC都对上才
 * 采信其余字段，否则只保留本次reset_flags。随后清复位标志并立即落本次运行的骨架。 */
void app_reset_init(void)
{
    uint32_t words[APP_RESET_RECORD_WORDS];
    uint32_t flags = RCC->CSR; /* 先锁存本次复位的RCC标志，后面清标志前读取。 */
    uint32_t mask = __get_PRIMASK(); /* 保存进入前的中断屏蔽状态，退出时恢复。 */
    uint32_t i;
    __disable_irq(); /* 读取备份域和提交骨架期间屏蔽可屏蔽中断。 */
    reset_backup_enable(); /* 解锁备份域并打开RTC时钟。 */
    /* 整份19字读出备份域记录。 */
    for (i = 0; i < APP_RESET_RECORD_WORDS; ++i)
    {
        words[i] = *reset_backup_word(i);
    }
    memset(&s_previous, 0, sizeof(s_previous)); /* 先按无有效上一记录处理。 */
    /* magic、版本和CRC三者都对上，才采信记录里的现场字段。 */
    if (words[0] == APP_RESET_MAGIC && words[1] == APP_RESET_VERSION &&
        words[APP_RESET_RECORD_WORDS - 1] == reset_checksum(words))
    {
        memcpy(&s_previous, words + 2, sizeof(s_previous)); /* 取出16字现场结构体。 */
        s_previous.valid = 1; /* 标记这份记录内容有效。 */
    }
    s_previous.reset_flags = flags; /* 无论记录是否有效，都保留本次RCC复位标志。 */
    __HAL_RCC_CLEAR_RESET_FLAGS(); /* 标志已并入上一记录，立刻清除，避免下次开机重复计入。 */
    memset(&s_current, 0, sizeof(s_current)); /* 本次运行记录从全0开始。 */
    s_current.valid = 1; /* 本次骨架标记为有效。 */
    s_have_feed = 0; /* 尚无喂狗起点。 */
    s_stopped = 0; /* 尚未被异常接管。 */
    reset_store(); /* 先落一份"本次运行中"的骨架(kind=RUNNING)，喂狗和异常都在它上面推进。 */
    __set_PRIMASK(mask); /* 恢复进入前的中断屏蔽状态。 */
}

/* 实际喂狗后推进检查点：首拍建立起点，之后统计最大喂狗间隔。字段快照和提交时短暂屏蔽中断，
 * CRC计算时恢复原中断屏蔽状态；异常接管后不再覆盖现场。 */
void app_reset_watchdog_feed(uint32_t now_ms)
{
    uint32_t words[APP_RESET_RECORD_WORDS];
    uint32_t mask = __get_PRIMASK();
    uint32_t gap_ms;
    __disable_irq(); /* 更新内存检查点字段时短暂屏蔽可屏蔽中断。 */
    /* 异常已接管(s_stopped)时不再推进检查点，直接恢复中断返回。 */
    if (s_stopped != 0)
    {
        __set_PRIMASK(mask);
        return;
    }
    /* 不是第一次喂狗时，算出距上次喂狗的毫秒差。 */
    if (s_have_feed != 0)
    {
        gap_ms = now_ms - s_current.last_feed_ms;
        /* 本次间隔超过历史最大值才更新最大值。 */
        if (gap_ms > s_current.max_feed_gap_ms)
        {
            s_current.max_feed_gap_ms = gap_ms;
        }
    }
    s_current.uptime_ms = now_ms; /* 记录本次运行已到达的时刻。 */
    s_current.last_feed_ms = now_ms; /* 作为下一次间隔计算的起点。 */
    s_have_feed = 1; /* 已建立首个喂狗起点。 */
    words[0] = APP_RESET_MAGIC; /* 组一份待提交记录：magic。 */
    words[1] = APP_RESET_VERSION; /* 版本字。 */
    memcpy(words + 2, &s_current, sizeof(s_current)); /* 16字检查点现场。 */
    __set_PRIMASK(mask); /* 内存字段已取快照，先恢复中断再算CRC。 */

    /* 正常喂狗的CRC不屏蔽UART；异常若在此期间接管，旧检查点不能覆盖异常现场。 */
    words[APP_RESET_RECORD_WORDS - 1] = reset_checksum(words);
    __disable_irq();
    /* 再次确认未被异常接管，才把这份检查点提交到备份域。 */
    if (s_stopped == 0)
    {
        reset_commit(words); /* RTC清备份域后，下一拍仍重新提交完整记录。 */
    }
    __set_PRIMASK(mask);
}

/* 异常与停机共用的现场采集；先置 s_stopped，之后的正常喂狗不得再覆盖这份记录。 */
static void reset_capture_registers(uint32_t kind)
{
    s_stopped = 1; /* 先标记停机，后续正常喂狗不得再覆盖这份记录。 */
    s_current.kind = kind; /* 记录停机/异常类型枚举。 */
    s_current.uptime_ms = HAL_GetTick(); /* 记录发生时刻的毫秒 tick。 */
    s_current.cfsr = SCB->CFSR; /* 配置/用法/总线/内存管理故障状态寄存器。 */
    s_current.hfsr = SCB->HFSR; /* 硬故障状态寄存器。 */
    s_current.mmfar = SCB->MMFAR; /* 内存管理故障地址寄存器。 */
    s_current.bfar = SCB->BFAR; /* 总线故障地址寄存器。 */
}

/* 硬件异常入口调用：只有非栈类故障、栈指针4字节对齐且整段压栈帧都落在SRAM内时，
 * 才采信硬件帧里的LR/PC/xPSR；EXC_RETURN bit4=0 说明还压了S0..S15/FPSCR，边界按26字算。 */
void app_reset_capture_fault(uint32_t kind, const uint32_t *stack, uint32_t exc_return)
{
    uintptr_t address = (uintptr_t)stack; /* 异常入口传入的原始栈指针。 */
    uint32_t frame_bytes = 8 * sizeof(uint32_t); /* 基本硬件压栈帧为8字。 */
    uint32_t mask = __get_PRIMASK();
    __disable_irq(); /* 采集与提交期间关中断。 */
    /* 已有现场时先到的优先，本次不再覆盖。 */
    if (s_stopped == 0)
    {
        reset_capture_registers(kind); /* 置停机标记并抓取CFSR/HFSR/MMFAR/BFAR。 */
        s_current.sp = (uint32_t)address; /* 记录异常栈指针。 */
        s_current.exc_return = exc_return; /* 记录EXC_RETURN值。 */
        /* EXC_RETURN bit4=0表示还压入了S0..S15/FPSCR，帧长为26字。 */
        if ((exc_return & (1U << 4)) == 0)
        {
            frame_bytes = 26 * sizeof(uint32_t);
        }
        /* 非栈类故障、栈指针4字节对齐且整个压栈帧都落在SRAM范围内，才采信帧内容。 */
        if ((s_current.cfsr & APP_RESET_STACK_ERRORS) == 0 && (address & 3) == 0 &&
            address >= APP_RESET_SRAM_START && address <= APP_RESET_SRAM_END - frame_bytes)
        {
            /* Cortex-M4F扩展帧的R0..xPSR仍在前8字，S0..S15/FPSCR在其后。 */
            s_current.lr = stack[5]; /* 压栈帧第5字为LR。 */
            s_current.pc = stack[6]; /* 第6字为故障返回地址PC。 */
            s_current.xpsr = stack[7]; /* 第7字为xPSR。 */
        }
        reset_store(); /* 把采集到的现场提交到备份域。 */
    }
    __set_PRIMASK(mask); /* 恢复中断屏蔽状态。 */
}

/* 无硬件帧的停机记录：记录kind、pc、task_tag并落盘，随后由调用方死循环等看门狗。
 * s_stopped已置位时直接返回，先到的现场优先。 */
void app_reset_capture_stop(uint32_t kind, uint32_t pc, uint32_t task_tag)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq(); /* 采集与提交期间关中断。 */
    /* 先到的现场优先；已停机时不再覆盖。 */
    if (s_stopped == 0)
    {
        reset_capture_registers(kind); /* 置停机标记并抓取故障状态寄存器。 */
        s_current.pc = pc; /* 记录调用方给出的停机地址。 */
        s_current.task_tag = task_tag; /* 记录任务名字节等辅助标记。 */
        reset_store(); /* 提交到备份域，之后由调用方死循环等看门狗。 */
    }
    __set_PRIMASK(mask);
}

/* LVGL断言：记录调用者返回地址作为pc后停机，不尝试恢复LVGL。 */
void app_reset_assert_stop(void)
{
    __disable_irq(); /* 断言停机路径不再恢复中断。 */
    /* ARMCC用内建返回地址取调用者地址；GCC/Clang同样取当前函数返回地址。 */
#if defined(__CC_ARM)
    app_reset_capture_stop(APP_RESET_LVGL_ASSERT, (uint32_t)__return_address(), 0);
#else
    app_reset_capture_stop(APP_RESET_LVGL_ASSERT, (uint32_t)(uintptr_t)__builtin_return_address(0), 0);
#endif
    /* 记录后原地死循环，等外部看门狗复位。 */
    for (;;)
    {
    }
}

/* 只发布开机时读到的上一份记录；本次运行的进展要到下一次复位后才会被这里读到。 */
void app_reset_get_info(app_reset_info_struct *info)
{
    /* 调用方给了输出缓冲才复制，避免空指针写入。 */
    if (info != NULL)
    {
        *info = s_previous; /* 整份复制开机时固定的上一启动快照。 */
    }
}
