/**
 * uart1.c — USART1 interrupt-driven RX, polling TX, 8N1
 *
 * Pins:
 *   PA9   USART1_TX  AF7   → ESP32-C6 RX
 *   PA10  USART1_RX  AF7   ← ESP32-C6 TX
 *
 * RX is interrupt-driven: every received byte is pushed into a 64-byte ring
 * buffer inside USART1_IRQHandler.  This prevents the overrun errors (ORE)
 * that occur when polling code sleeps for 10 ms while a full 8-byte frame
 * arrives in < 1 ms at 115200 baud.
 *
 * TX remains polling (busy-wait on TXE) — fine because TX is infrequent and
 * there is no timing-sensitive work blocked behind it.
 *
 * Clock source: PCLK2 = 80 MHz (default USART1SEL after SystemClock_Config).
 * BRR = 80 000 000 / 115 200 = 694  (error < 0.07 %)
 */

#include <stdint.h>
#include "stm32l452xx.h"
#include "uart1.h"

/* -------------------------------------------------------------------------
 * Ring buffer (power-of-two size for cheap masking)
 * ------------------------------------------------------------------------- */

#define RX_BUF_SIZE  64u          /* must be a power of two */
#define RX_BUF_MASK  (RX_BUF_SIZE - 1u)

static volatile uint8_t  s_rx_buf[RX_BUF_SIZE];
static volatile uint32_t s_rx_head = 0u;   /* IRQ writes here  */
static volatile uint32_t s_rx_tail = 0u;   /* main code reads here */

/* -------------------------------------------------------------------------
 * Init
 * ------------------------------------------------------------------------- */

void UART1_Init(uint32_t baud)
{
    /* ---- Enable clocks --------------------------------------------------- */
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    __DSB();

    /* ---- PA9 (TX) and PA10 (RX): Alternate function 7 ------------------- */
    GPIOA->MODER = (GPIOA->MODER
                     & ~(GPIO_MODER_MODE9_Msk | GPIO_MODER_MODE10_Msk))
                 | (2UL << GPIO_MODER_MODE9_Pos)
                 | (2UL << GPIO_MODER_MODE10_Pos);

    GPIOA->AFR[1] = (GPIOA->AFR[1]
                      & ~(GPIO_AFRH_AFSEL9_Msk | GPIO_AFRH_AFSEL10_Msk))
                  | (7UL << GPIO_AFRH_AFSEL9_Pos)
                  | (7UL << GPIO_AFRH_AFSEL10_Pos);

    GPIOA->OSPEEDR |= (3UL << GPIO_OSPEEDR_OSPEED9_Pos)
                   |  (3UL << GPIO_OSPEEDR_OSPEED10_Pos);
    GPIOA->PUPDR   &= ~(GPIO_PUPDR_PUPD9_Msk | GPIO_PUPDR_PUPD10_Msk);

    /* ---- Configure USART1 ------------------------------------------------ */
    USART1->CR1 = 0;
    USART1->CR2 = 0;
    USART1->CR3 = 0;

    USART1->BRR = (uint16_t)(80000000UL / baud);

    /*
     * CR1: enable TE, RE, RXNEIE (fires on RXNE and on ORE when RXNE=1),
     * then set UE last.
     */
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE
                | USART_CR1_RXNEIE   /* RX-not-empty interrupt */
                | USART_CR1_UE;

    /* ---- Enable USART1 IRQ in NVIC --------------------------------------- */
    NVIC_SetPriority(USART1_IRQn, 1);   /* priority 1 — below SysTick if used */
    NVIC_EnableIRQ(USART1_IRQn);
}

/* -------------------------------------------------------------------------
 * USART1 interrupt handler
 * Every received byte is stored in the ring buffer.
 * Error flags (ORE, FE, NE) are cleared so the receiver never stalls.
 * ------------------------------------------------------------------------- */

void USART1_IRQHandler(void)
{
    uint32_t isr = USART1->ISR;

    /* Clear any error flags first so the receiver stays enabled */
    if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE)) {
        USART1->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF;
    }

    /* If a byte is available, push it into the ring buffer */
    if (isr & USART_ISR_RXNE) {
        uint8_t byte = (uint8_t)USART1->RDR;   /* reading RDR also clears RXNE */

        uint32_t next_head = (s_rx_head + 1u) & RX_BUF_MASK;
        if (next_head != s_rx_tail) {           /* drop byte on overflow */
            s_rx_buf[s_rx_head] = byte;
            s_rx_head = next_head;
        }
    }
}

/* -------------------------------------------------------------------------
 * TX — polling (unchanged)
 * ------------------------------------------------------------------------- */

void UART1_SendChar(char c)
{
    while (!(USART1->ISR & USART_ISR_TXE));
    USART1->TDR = (uint8_t)c;
}

void UART1_SendString(const char *s)
{
    while (*s) {
        UART1_SendChar(*(s++));
    }
}

void UART1_SendBytes(const uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        UART1_SendChar((char)buf[i]);
    }
}

/* -------------------------------------------------------------------------
 * RX — reads from the ring buffer filled by the IRQ
 * ------------------------------------------------------------------------- */

int UART1_DataAvailable(void)
{
    return (s_rx_head != s_rx_tail) ? 1 : 0;
}

uint8_t UART1_ReceiveByte(void)
{
    /* Block until a byte is in the ring buffer */
    while (s_rx_head == s_rx_tail);

    uint8_t byte    = s_rx_buf[s_rx_tail];
    s_rx_tail       = (s_rx_tail + 1u) & RX_BUF_MASK;
    return byte;
}
