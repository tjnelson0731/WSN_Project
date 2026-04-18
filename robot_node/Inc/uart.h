#ifndef UART_H
#define UART_H

#include <stdint.h>

/**
 * UART — USART2, 8N1
 *
 *   PA2  USART2_TX  AF7   → ST-Link virtual COM port (debug channel)
 *   PA3  USART2_RX  AF7   ← ST-Link virtual COM port (debug channel)
 *
 * USART1 (PA9/PA10) is used for the ESP32-C6 link — see uart1.h.
 *
 * BRR calculation (oversampling × 16, PCLK1 = 80 MHz):
 *   BRR = f_PCLK1 / baud = 80 000 000 / 115 200 = 694
 *   Actual baud = 80 000 000 / 694 ≈ 115 274 bps  (error < 0.07 %)
 */

/** Configure USART2; call once after SystemClock_Config(). */
void UART_Init(uint32_t baud);

/** Blocking transmit helpers (poll TXE flag). */
void UART_SendChar (char c);
void UART_SendString(const char *s);
void UART_SendBytes (const uint8_t *buf, uint32_t len);

/** Blocking receive: waits until RXNE is set. */
uint8_t UART_ReceiveByte(void);

/** Non-blocking: returns 1 if a byte is waiting in the receive register. */
int UART_DataAvailable(void);

#endif /* UART_H */
