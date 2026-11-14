#ifndef RAM_UART_POLL_H
#define RAM_UART_POLL_H

#include "main.h"

/* Flash忙时必须内联到调用方的SRAM段，不能跳转到Flash中的函数。 */
#if defined(__CC_ARM)
static __forceinline void ram_uart_poll_usart1(uint16_t received[8], uint8_t *count)
#else
static inline __attribute__((always_inline)) void ram_uart_poll_usart1(uint16_t received[8], uint8_t *count)
#endif
{
    uint32_t status = USART1->SR;
    uint16_t byte;

    if ((status & (USART_SR_RXNE | 0x0F)) != 0)
    {
        /* 低8位为数据，bit8..11保留PE/FE/NF/ORE；读取DR清除接收状态。 */
        byte = (uint16_t)((USART1->DR & 0xFF) | ((status & 0x0F) << 8));
        if (*count < 8)
        {
            received[(*count)++] = byte;
        }
        else
        {
            received[7] |= 0x0800;
        }
    }
}

#endif
