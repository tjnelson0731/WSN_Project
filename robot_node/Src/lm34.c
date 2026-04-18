/**
 * lm34.c — LM34 temperature sensor via ADC1, channel 5 (PA0)
 *
 * ADC1 initialization sequence (RM0394 §18.4.6):
 *
 *   Step 1  — Exit deep power-down (DEEPPWD = 0)
 *   Step 2  — Enable internal voltage regulator (ADVREGEN = 1),
 *              wait ≥ 20 µs (t_ADCVREG_STUP per datasheet)
 *   Step 3  — Calibrate (ADCAL = 1 for single-ended), wait for ADCAL = 0
 *   Step 4  — Enable ADC (ADEN = 1), wait for ADRDY = 1
 *   Step 5  — Configure channel + sampling time
 *
 * ADC clock configuration:
 *   ADC1_COMMON->CCR  CKMODE[1:0] = 0b11  →  HCLK / 4 = 20 MHz
 *   (Synchronous mode; no async divider needed; BOOST = 0 for ≤ 20 MHz)
 *
 * Conversion formula — all integer, no floats:
 *
 *   T_F_tenths = raw × 3300 / 4095
 *
 * Why this works:
 *   LM34 sensitivity is 10 mV/°F → 1 mV = 0.1 °F = 1 tenth-°F.
 *   V_mV = raw × 3300 / 4095, and since 1 mV == 1 tenth-°F,
 *   T_F_tenths uses the exact same expression — convenient.
 */

#include "stm32l452xx.h"
#include "lm34.h"
#include "clock.h"

/* PA0 = ADC1 channel 5 */
#define LM34_ADC_CHANNEL   5U

/* ---------------------------------------------------------------------- */
void LM34_Init(void)
{
    /* ---- GPIO --------------------------------------------------------- */
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    __DSB();

    /*
     * PA0 → Analog mode
     *   MODER[1:0] = 11 → Analog mode.
     *   This automatically disconnects the digital input buffer and
     *   connects the pin to the ADC analog input (RM0394 §8.4.1).
     *   No separate analog switch register (ASCR) exists on STM32L452;
     *   MODER = 11 is the only configuration step required.
     *   PUPDR = 00 → no pull, required for accurate ADC readings.
     */
    GPIOA->MODER  |=  (3UL << GPIO_MODER_MODE0_Pos);   /* Analog mode     */
    GPIOA->PUPDR  &= ~(3UL << GPIO_PUPDR_PUPD0_Pos);   /* No pull         */

    /* ---- ADC clock ----------------------------------------------------- */
    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;
    __DSB();

    /*
     * CKMODE[1:0] = 0b11 → synchronous HCLK/4 = 20 MHz.
     * This field lives in the ADC common register (shared by ADC1/ADC2).
     * Must be written while both ADCs are disabled (ADEN = 0), which is
     * the state after any reset.
     */
    ADC1_COMMON->CCR = (ADC1_COMMON->CCR & ~ADC_CCR_CKMODE_Msk)
                     | (3UL << ADC_CCR_CKMODE_Pos);

    /* ---- Step 1: exit deep power-down ---------------------------------- */
    ADC1->CR &= ~ADC_CR_DEEPPWD;

    /* ---- Step 2: enable internal voltage regulator -------------------- */
    ADC1->CR |= ADC_CR_ADVREGEN;
    delay_us(25U);   /* datasheet: t_ADCVREG_STUP = 20 µs; we use 25 µs   */

    /* ---- Step 3: single-ended calibration ----------------------------- */
    /*
     * ADCALDIF = 0 (single-ended, already the reset value).
     * Write ADCAL = 1; hardware clears it when calibration is complete.
     * Do NOT have ADEN set during calibration.
     */
    ADC1->CR &= ~ADC_CR_ADCALDIF;
    ADC1->CR |=  ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL);   /* wait ~116 ADC clock cycles      */

    /* ---- Step 4: enable ADC ------------------------------------------- */
    /*
     * Datasheet note: write ADEN exactly 4 ADC clock cycles after
     * ADCAL cleared.  We clear ADRDY first (write 1 to clear), then
     * set ADEN, and poll ADRDY.
     */
    ADC1->ISR |= ADC_ISR_ADRDY;        /* clear ADRDY (w1c)               */
    ADC1->CR  |= ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRDY));

    /* ---- Step 5: channel and sampling time ----------------------------- */
    /*
     * Single conversion, 12-bit resolution (default after reset):
     *   RES[1:0] = 00 → 12 bits
     *   CONT     = 0  → single conversion mode
     *   DISCEN   = 0  → no discontinuous mode
     *   EXTEN    = 00 → software trigger
     *
     * Sequence: L[3:0] = 0 (1 conversion), SQ1[4:0] = channel 5
     */
    ADC1->CFGR  = 0x80000000;   /* 12-bit, right-aligned, software trigger          */
    ADC1->SQR1  = (LM34_ADC_CHANNEL << ADC_SQR1_SQ1_Pos);

    /*
     * Sampling time for CH5: SMPR1 bits [17:15] = SMP5
     * Encoding 101 = 92.5 ADC cycles.
     *
     * Minimum sampling time calculation (RM0394 §18.4.13):
     *   t_samp ≥ (R_src + R_ADC) × C_ADC × ln(2^(N+2))
     *   With R_src ≈ 1 Ω (LM34 output), R_ADC ≈ 6 kΩ, C_ADC = 5 pF:
     *   t_samp ≥ 6001 × 5e-12 × ln(16384) ≈ 0.28 µs = 5.6 ADC cycles
     *   92.5 cycles = 4.625 µs is very comfortably above this minimum.
     */
    ADC1->SMPR1 = (ADC1->SMPR1 & ~ADC_SMPR1_SMP5_Msk)
                | (5UL << ADC_SMPR1_SMP5_Pos);   /* 101 = 92.5 cycles     */
}

/* ---------------------------------------------------------------------- */
int16_t LM34_Read(void)
{
    /* Start a single conversion */
    ADC1->CR |= ADC_CR_ADSTART;

    /* Wait for End Of Conversion */
    while (!(ADC1->ISR & ADC_ISR_EOC));

    uint16_t raw = (uint16_t)(ADC1->DR & 0x0FFFU);

    /*
     * T_F_tenths = raw × 3300 / 4095
     *   raw_max = 4095 → 3300 (330.0 °F) — above LM34's 300 °F max
     *   raw × 3300 ≤ 4095 × 3300 = 13 513 500 → fits in int32_t
     */
    return (int16_t)(((int32_t)raw * 3300) / 4095);
}
