#ifndef UART1_H
#define UART1_H

#include <stdint.h>

/**
 * UART1 — USART1, 8N1, interrupt-driven RX / polling TX
 *
 *   PA9   USART1_TX  AF7   → ESP32-C6 RX
 *   PA10  USART1_RX  AF7   ← ESP32-C6 TX
 *
 * RX is interrupt-driven: USART1_IRQHandler stores every received byte in a
 * 64-byte ring buffer so the main loop can sleep without losing bytes to ORE.
 *
 * TX remains polling (busy-wait on TXE).
 *
 * USART1 is clocked from PCLK2 = 80 MHz (default USART1SEL).
 * BRR = 80 000 000 / 115 200 = 694  (error < 0.07 %)
 *
 * USART2 (PA2/PA3) is kept intact for the virtual COM port debug channel.
 */

/** Configure USART1 and enable the RXNE interrupt; call once after SystemClock_Config(). */
void UART1_Init(uint32_t baud);

/** Blocking transmit helpers (poll TXE flag). */
void UART1_SendChar  (char c);
void UART1_SendString(const char *s);
void UART1_SendBytes (const uint8_t *buf, uint32_t len);

/** Non-blocking: returns 1 if at least one byte is waiting in the RX ring buffer. */
int UART1_DataAvailable(void);

/** Returns the next byte from the RX ring buffer; blocks if the buffer is empty. */
uint8_t UART1_ReceiveByte(void);

#endif /* UART1_H */
