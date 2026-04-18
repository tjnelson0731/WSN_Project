/**
 * uart.c — USART2 polling driver, 8N1
 *
 * Pins:
 *   PA2  USART2_TX  AF7
 *   PA3  USART2_RX  AF7
 *
 * Clock source selection (CCIPR register, bits [3:2] USART2SEL):
 *   00 = PCLK1 (default after reset) — we use this.
 *   PCLK1 = 80 MHz after SystemClock_Config().
 *
 * BRR calculation (OVER8 = 0 → oversampling by 16):
 *
 *   BRR = f_PCLK1 / baud
 *
 *   Example: baud = 115 200
 *   BRR = 80 000 000 / 115 200 = 694.44 → 694  (truncation, not rounding)
 *   Actual baud = 80 000 000 / 694 ≈ 115 274 bps  (error ≈ +0.064 %)
 *   This is well within the UART ±2 % tolerance.
 *
 * Note: The ESP32-C6 is 3.3 V logic — direct connection is safe.
 *       Cross-connect TX↔RX: STM PA2 → ESP32 RX, STM PA3 ← ESP32 TX.
 */

#include "stm32l452xx.h"
#include "uart.h"

void UART_Init(uint32_t baud)
{
    /* Enable clocks */
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOAEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
    __DSB();

    /* ---- PA2 (TX) and PA3 (RX): Alternate function 7 ----------------- */
    /* MODER: 10 = Alternate function */
    GPIOA->MODER = (GPIOA->MODER
                     & ~(GPIO_MODER_MODE2_Msk | GPIO_MODER_MODE3_Msk))
                 | (2UL << GPIO_MODER_MODE2_Pos)
                 | (2UL << GPIO_MODER_MODE3_Pos);

    /* AFRL: AF7 for PA2, PA3 */
    GPIOA->AFR[0] = (GPIOA->AFR[0]
                      & ~(GPIO_AFRL_AFSEL2_Msk | GPIO_AFRL_AFSEL3_Msk))
                  | (7UL << GPIO_AFRL_AFSEL2_Pos)
                  | (7UL << GPIO_AFRL_AFSEL3_Pos);

    /* High speed, no pull (USART lines are actively driven) */
    GPIOA->OSPEEDR |= (3UL << GPIO_OSPEEDR_OSPEED2_Pos)
                   |  (3UL << GPIO_OSPEEDR_OSPEED3_Pos);
    GPIOA->PUPDR   &= ~(GPIO_PUPDR_PUPD2_Msk | GPIO_PUPDR_PUPD3_Msk);

    /* ---- Configure USART2 --------------------------------------------- */
    /* Disable USART before writing configuration registers               */
    USART2->CR1 = 0;
    USART2->CR2 = 0;
    USART2->CR3 = 0;

    /*
     * BRR: with OVER8 = 0 (default), BRR = f_PCLK1 / baud.
     * No rounding correction is applied — the resulting error is < 0.1 %.
     */
    USART2->BRR = (uint16_t)(80000000UL / baud);

    /*
     * CR1:
     *   M[1:0] = 00  → 8 data bits (default)
     *   PCE    = 0   → no parity   (default)
     *   TE     = 1   → transmitter enable
     *   RE     = 1   → receiver enable
     *   UE     = 1   → USART enable  (write last)
     */
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

/* ---------------------------------------------------------------------- */

void UART_SendChar(char c)
{
    /* Wait until transmit data register is empty (TXE = 1)              */
    while (!(USART2->ISR & USART_ISR_TXE));
    USART2->TDR = (uint8_t)c;
}

void UART_SendString(const char *s)
{
    while (*s) {
        UART_SendChar(*(s++));
    }
}

void UART_SendBytes(const uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        UART_SendChar((char)buf[i]);
    }
}

uint8_t UART_ReceiveByte(void)
{
    /* Block until receive data register is not empty (RXNE = 1)         */
    while (!(USART2->ISR & USART_ISR_RXNE));
    return (uint8_t)USART2->RDR;
}

int UART_DataAvailable(void)
{
    return (USART2->ISR & USART_ISR_RXNE) ? 1 : 0;
}
