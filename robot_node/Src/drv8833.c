/**
 * drv8833.c — Differential drive via DRV8833 dual H-bridge
 *
 * Hardware assignments:
 *   Left  motor (Bridge A): AIN1 ← PA6 (TIM3_CH1, AF2)
 *                           AIN2 ← PA7 (TIM3_CH2, AF2)
 *   Right motor (Bridge B): BIN1 ← PB0 (TIM3_CH3, AF2)
 *                           BIN2 ← PB1 (TIM3_CH4, AF2)
 *   STBY (nSLEEP)         : PB4  (GPIO output — HIGH = run, LOW = sleep)
 *
 * TIM3 PWM configuration:
 *   Source clock : PCLK1 = 80 MHz
 *   Prescaler    : PSC = 3   → timer clock = 80 MHz / 4 = 20 MHz
 *   Auto-reload  : ARR = 999 → f_PWM = 20 MHz / 1000 = 20 kHz
 *   CCR range    : 0 – 999   → 0 % – 99.9 % duty
 *                  CCR = 1000 (> ARR) → 100 % (always HIGH in PWM mode 1)
 *
 * PWM Mode 1 output (OC1M = 0110):
 *   CNT < CCR → output HIGH
 *   CNT ≥ CCR → output LOW
 *
 * Fast-decay speed control:
 *   FORWARD  at speed S: CCR_IN1 = S,    CCR_IN2 = 0
 *   BACKWARD at speed S: CCR_IN1 = 0,    CCR_IN2 = S
 *   BRAKE              : CCR_IN1 = 1000, CCR_IN2 = 1000 → both always HIGH
 *                        → DRV8833 xIN1=H xIN2=H → both OUT LOW (slow decay)
 *   COAST              : CCR_IN1 = 0,    CCR_IN2 = 0    → both always LOW
 *                        → DRV8833 xIN1=L xIN2=L → both OUT Hi-Z (freewheel)
 *
 * STBY wake-up: datasheet specifies tWAKE ≤ 1 ms (nSLEEP inactive-high to
 * H-bridge on), so DRV8833_Wake() issues a 1 ms delay after asserting PB4.
 *
 * NOTE: This file uses TIM3 channels 1–4 and conflicts with motor.c
 * (SN754410).  Link only one motor driver at a time — swap motor.c ↔
 * drv8833.c in the Makefile C_SOURCES list.
 */

#include "stm32l452xx.h"
#include "clock.h"
#include "drv8833.h"

/* ---- Pin definitions -------------------------------------------------- */
#define STBY_PIN    4U   /* PB4 — STBY / nSLEEP                            */

#define MAX_SPEED   1000U   /* CCR range 0–1000; CCR > ARR=999 = 100 % duty */

/* ---------------------------------------------------------------------- */
void DRV8833_Init(void)
{
    /* Enable peripheral clocks for GPIOA, GPIOB, and TIM3               */
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
    __DSB();   /* ensure clocks are active before any register access     */


    /* ----  PA6 / PA7 : ALTERNATE FUNCTION 2 (TIM3_CH1 / TIM3_CH2)  --- */

    /*
     * MODER: set PA6 and PA7 to Alternate Function mode
     *   MODERx[1:0] = 10 → Alternate function
     */
    GPIOA->MODER = (GPIOA->MODER
                     & ~(GPIO_MODER_MODE6_Msk | GPIO_MODER_MODE7_Msk))
                 | (2UL << GPIO_MODER_MODE6_Pos)
                 | (2UL << GPIO_MODER_MODE7_Pos);

    /*
     * AFRL: select AF2 for PA6 and PA7
     *   AFSEL6[3:0] = 0010 → AF2 (TIM3_CH1)
     *   AFSEL7[3:0] = 0010 → AF2 (TIM3_CH2)
     */
    GPIOA->AFR[0] = (GPIOA->AFR[0]
                      & ~(GPIO_AFRL_AFSEL6_Msk | GPIO_AFRL_AFSEL7_Msk))
                  | (2UL << GPIO_AFRL_AFSEL6_Pos)
                  | (2UL << GPIO_AFRL_AFSEL7_Pos);

    /*
     * OSPEEDR: high-speed output for PWM signals on PA6, PA7
     *   OSPEEDRx[1:0] = 11 → Very high speed
     */
    GPIOA->OSPEEDR |= (3UL << GPIO_OSPEEDR_OSPEED6_Pos)
                   |  (3UL << GPIO_OSPEEDR_OSPEED7_Pos);


    /* ----  PB0 / PB1 : ALTERNATE FUNCTION 2 (TIM3_CH3 / TIM3_CH4)  --- */

    /*
     * MODER: set PB0 and PB1 to Alternate Function mode
     *   MODERx[1:0] = 10 → Alternate function
     */
    GPIOB->MODER = (GPIOB->MODER
                     & ~(GPIO_MODER_MODE0_Msk | GPIO_MODER_MODE1_Msk))
                 | (2UL << GPIO_MODER_MODE0_Pos)
                 | (2UL << GPIO_MODER_MODE1_Pos);

    /*
     * AFRL: select AF2 for PB0 and PB1
     *   AFSEL0[3:0] = 0010 → AF2 (TIM3_CH3)
     *   AFSEL1[3:0] = 0010 → AF2 (TIM3_CH4)
     */
    GPIOB->AFR[0] = (GPIOB->AFR[0]
                      & ~(GPIO_AFRL_AFSEL0_Msk | GPIO_AFRL_AFSEL1_Msk))
                  | (2UL << GPIO_AFRL_AFSEL0_Pos)
                  | (2UL << GPIO_AFRL_AFSEL1_Pos);

    /*
     * OSPEEDR: high-speed output for PWM signals on PB0, PB1
     *   OSPEEDRx[1:0] = 11 → Very high speed
     */
    GPIOB->OSPEEDR |= (3UL << GPIO_OSPEEDR_OSPEED0_Pos)
                   |  (3UL << GPIO_OSPEEDR_OSPEED1_Pos);


    /* ----  PB4 : GPIO OUTPUT (STBY / nSLEEP)  ------------------------- */

    /*
     * MODER: set PB4 to General Purpose Output mode
     *   MODER4[1:0] = 01 → General purpose output
     */
    GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODE4_Msk)
                 | (1UL << GPIO_MODER_MODE4_Pos);

    /*
     * OTYPER: push-pull output (OT4 = 0).
     * PUPDR:  no pull-up/pull-down (PUPD4[1:0] = 00).
     */
    GPIOB->OTYPER &= ~(1UL << STBY_PIN);
    GPIOB->PUPDR  &= ~GPIO_PUPDR_PUPD4_Msk;

    /*
     * ODR: assert STBY HIGH immediately so the DRV8833 is active.
     * The 1 ms tWAKE delay is handled below after TIM3 is running.
     */
    GPIOB->ODR |= (1UL << STBY_PIN);


    /* ----  TIM3 : 20 kHz PWM ON CH1, CH2, CH3, CH4  ------------------- */

    TIM3->CR1  = 0;     /* disable timer while configuring                */
    TIM3->PSC  = 3U;    /* 80 MHz / (3+1) = 20 MHz timer clock            */
    TIM3->ARR  = 999U;  /* 20 MHz / (999+1) = 20 kHz PWM frequency        */

    /* All four compare registers start at 0 → all outputs LOW (coast)    */
    TIM3->CCR1 = 0U;    /* AIN1 — left forward                            */
    TIM3->CCR2 = 0U;    /* AIN2 — left reverse                            */
    TIM3->CCR3 = 0U;    /* BIN1 — right forward                           */
    TIM3->CCR4 = 0U;    /* BIN2 — right reverse                           */

    /*
     * CCMR1: configure CH1 and CH2 as PWM Mode 1
     *
     *   OC1M[2:0] = 110 → PWM mode 1 (output HIGH while CNT < CCR1)
     *   OC1PE     = 1   → CCR1 preload enable (updates take effect on UEV)
     *   OC2M[2:0] = 110 → same for CH2
     *   OC2PE     = 1   → CCR2 preload enable
     */
    TIM3->CCMR1 = (6UL << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE
                | (6UL << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;

    /*
     * CCMR2: configure CH3 and CH4 as PWM Mode 1
     *
     *   OC3M[2:0] = 110 → PWM mode 1 (output HIGH while CNT < CCR3)
     *   OC3PE     = 1   → CCR3 preload enable
     *   OC4M[2:0] = 110 → same for CH4
     *   OC4PE     = 1   → CCR4 preload enable
     */
    TIM3->CCMR2 = (6UL << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE
                | (6UL << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;

    /*
     * CCER: enable all four channel outputs, active HIGH polarity
     *   CC1E = 1 → CH1 output enabled
     *   CC2E = 1 → CH2 output enabled
     *   CC3E = 1 → CH3 output enabled
     *   CC4E = 1 → CH4 output enabled
     *   CCxP remain 0 → active HIGH
     */
    TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E
               | TIM_CCER_CC3E | TIM_CCER_CC4E;

    /*
     * CR1: enable auto-reload preload so ARR changes take effect only
     * on the next update event, preventing mid-period glitches.
     *   ARPE = 1 → TIMx_ARR register is buffered
     */
    TIM3->CR1  = TIM_CR1_ARPE;

    /* Generate an update event to transfer all preload values (PSC, ARR,
     * CCR1–CCR4) into their active shadow registers before enabling.     */
    TIM3->EGR  = TIM_EGR_UG;
    TIM3->SR   = 0;    /* clear the UIF flag that EGR_UG just set         */

    /* Start the timer — PWM outputs become active                        */
    TIM3->CR1 |= TIM_CR1_CEN;

    /* Wait for tWAKE: DRV8833 datasheet §6.5 specifies up to 1 ms from
     * nSLEEP inactive-high before the H-bridge outputs are ready.        */
    delay_ms(1);
}


/* ---------------------------------------------------------------------- */
void DRV8833_SetLeft(DRV8833_Dir dir, uint16_t speed)
{
    if (speed > MAX_SPEED) speed = (uint16_t)MAX_SPEED;

    switch (dir) {

    case DRV8833_FORWARD:
        /*
         * Fast decay forward: AIN1 = PWM, AIN2 = 0
         *   CCR1 = speed → duty % of 20 kHz applied to AIN1
         *   CCR2 = 0     → output always LOW (CCR ≤ 0 ≤ CNT in Mode 1)
         */
        TIM3->CCR1 = speed;
        TIM3->CCR2 = 0U;
        break;

    case DRV8833_BACKWARD:
        /*
         * Fast decay reverse: AIN1 = 0, AIN2 = PWM
         *   CCR1 = 0     → AIN1 always LOW
         *   CCR2 = speed → duty % of 20 kHz applied to AIN2
         */
        TIM3->CCR1 = 0U;
        TIM3->CCR2 = speed;
        break;

    case DRV8833_BRAKE:
        /*
         * Brake: AIN1 = H, AIN2 = H
         *   CCR > ARR (999) → output always HIGH in PWM Mode 1
         *   DRV8833: xIN1=H xIN2=H → both outputs LOW (slow decay)
         *   Motor windings shorted to GND → active braking
         */
        TIM3->CCR1 = MAX_SPEED;   /* CCR1 = 1000 > ARR = 999 → always HIGH */
        TIM3->CCR2 = MAX_SPEED;
        break;

    case DRV8833_COAST:
    default:
        /*
         * Coast: AIN1 = L, AIN2 = L
         *   CCR = 0 → output always LOW in PWM Mode 1
         *   DRV8833: xIN1=L xIN2=L → both outputs Hi-Z (freewheel)
         */
        TIM3->CCR1 = 0U;
        TIM3->CCR2 = 0U;
        break;
    }
}


/* ---------------------------------------------------------------------- */
void DRV8833_SetRight(DRV8833_Dir dir, uint16_t speed)
{
    if (speed > MAX_SPEED) speed = (uint16_t)MAX_SPEED;

    switch (dir) {

    case DRV8833_FORWARD:
        /*
         * Fast decay forward: BIN1 = PWM, BIN2 = 0
         *   CCR3 = speed → duty % applied to BIN1
         *   CCR4 = 0     → BIN2 always LOW
         */
        TIM3->CCR3 = speed;
        TIM3->CCR4 = 0U;
        break;

    case DRV8833_BACKWARD:
        /*
         * Fast decay reverse: BIN1 = 0, BIN2 = PWM
         *   CCR3 = 0     → BIN1 always LOW
         *   CCR4 = speed → duty % applied to BIN2
         */
        TIM3->CCR3 = 0U;
        TIM3->CCR4 = speed;
        break;

    case DRV8833_BRAKE:
        /*
         * Brake: BIN1 = H, BIN2 = H
         *   CCR > ARR → always HIGH → DRV8833 outputs both LOW (slow decay)
         */
        TIM3->CCR3 = MAX_SPEED;
        TIM3->CCR4 = MAX_SPEED;
        break;

    case DRV8833_COAST:
    default:
        /*
         * Coast: BIN1 = L, BIN2 = L
         *   CCR = 0 → always LOW → DRV8833 outputs Hi-Z
         */
        TIM3->CCR3 = 0U;
        TIM3->CCR4 = 0U;
        break;
    }
}


/* ---------------------------------------------------------------------- */
void DRV8833_Stop(void)
{
    /* Coast both bridges — all IN pins LOW → Hi-Z motor outputs          */
    TIM3->CCR1 = 0U;
    TIM3->CCR2 = 0U;
    TIM3->CCR3 = 0U;
    TIM3->CCR4 = 0U;
}


/* ---------------------------------------------------------------------- */
void DRV8833_Sleep(void)
{
    /* Pull STBY LOW — DRV8833 disables both H-bridges and enters
     * low-power sleep (~1.6 µA quiescent).  All internal logic resets.  */
    GPIOB->ODR &= ~(1UL << STBY_PIN);
}


/* ---------------------------------------------------------------------- */
void DRV8833_Wake(void)
{
    /* Assert STBY HIGH to bring DRV8833 out of sleep.  The datasheet
     * specifies tWAKE ≤ 1 ms before outputs are ready (§6.5).           */
    GPIOB->ODR |= (1UL << STBY_PIN);
    delay_ms(1);
}
