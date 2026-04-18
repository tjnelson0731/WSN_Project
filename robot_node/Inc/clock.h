#ifndef CLOCK_H
#define CLOCK_H

#include "stdint.h"

/**
 * SystemClock_Config()
 *
 * Configures the main PLL from HSI16 and initializes the TIM6 delay timer:
 *
 *   VCO_in  = HSI16 / PLLM  = 16 / 1  =  16 MHz
 *   VCO_out = VCO_in * PLLN = 16 * 10 = 160 MHz
 *   SYSCLK  = VCO_out / PLLR = 160 / 2 =  80 MHz
 *
 *   HCLK  (AHB)  = 80 MHz  (prescaler /1)
 *   PCLK1 (APB1) = 80 MHz  (prescaler /1)  — USART2, TIM3, TIM6
 *   PCLK2 (APB2) = 80 MHz  (prescaler /1)
 *
 * Flash latency is set to 4 WS (required for 64–80 MHz @ VDD 1.8–3.6 V).
 * Voltage scaling Range 1 is enforced (required for 80 MHz operation).
 *
 * After this call, TIM6 is ready and delay_us / delay_ms are available.
 */
void SystemClock_Config(void);

/**
 * delay_us() / delay_ms()
 *
 * Blocking spin-wait delays using TIM6 in one-pulse mode (OPM).
 * TIM6 runs at 1 MHz (1 tick = 1 µs), so delay_us has 1 µs resolution.
 *
 * delay_us:
 *   Maximum: 65535 µs (limited by the 16-bit TIM6 ARR register).
 *   Use delay_ms for longer delays.
 *
 * delay_ms:
 *   Implemented as a loop of 1000 µs calls; no practical upper limit.
 *
 * Note: interrupts are not disabled — an ISR firing mid-delay extends it
 * by the ISR's execution time.  For tight timing windows, disable IRQs
 * around the call or keep all ISRs short.
 */
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

#endif /* CLOCK_H */
