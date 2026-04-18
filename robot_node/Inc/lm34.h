#ifndef LM34_H
#define LM34_H

#include <stdint.h>

/**
 * LM34 analog temperature sensor — ADC1 channel 5, pin PA0
 *
 * Wiring:
 *   LM34 V+   → 3.3 V
 *   LM34 VOUT → PA0  (+ optional 100 nF cap to GND near the pin)
 *   LM34 GND  → GND
 *
 * Transfer function (LM34 datasheet):
 *   V_OUT = 10 mV/°F × T_F
 *
 * ADC conversion to temperature:
 *   V_OUT (mV) = raw × 3300 / 4095
 *
 *   LM34 sensitivity: 10 mV/°F → 1 mV = 0.1 °F = 1 tenth-°F.
 *   Therefore tenths-of-°F == V_mV numerically (same formula).
 *
 *   T_F_tenths = raw × 3300 / 4095
 *   e.g. raw = 2228 → 1794 → 179.4 °F
 *
 * Integer arithmetic throughout — no FPU dependency.
 *
 * ADC clock: HCLK/4 = 80 MHz/4 = 20 MHz (CKMODE = 0b11 in ADC_CCR).
 * Sampling time: 92.5 ADC cycles → t_samp = 4.625 µs.
 * Total conversion time = t_samp + 12.5 cycles = 105 cycles = 5.25 µs.
 */

/**
 * LM34_Init()
 *   Configure PA0 as analog, enable ADC1, run calibration.
 *   Call once after SystemClock_Config() (which also initialises TIM6).
 *   Allow ≥ 10 ms after power-up before the first LM34_Read() so the
 *   LM34's output settles (LM34 datasheet: t_response ≈ few ms).
 */
void LM34_Init(void);

/**
 * LM34_Read()
 *   Trigger one ADC conversion (blocking, ~5.25 µs).
 *   Returns temperature as tenths of °F.
 *   e.g. 734 = 73.4 °F
 */
int16_t LM34_Read(void);

#endif /* LM34_H */
