/**
 * clock.c — System clock configuration + TIM6-based delays
 *
 * Target: 80 MHz from HSI16 via PLL on STM32L452RE
 *
 * PLL transfer function:
 *   f_VCO_in  = f_HSI16 / PLLM  =  16 MHz / 1  =  16 MHz
 *   f_VCO_out = f_VCO_in * PLLN =  16 MHz * 10  = 160 MHz
 *   f_PLLR    = f_VCO_out / PLLR = 160 MHz / 2  =  80 MHz  → SYSCLK
 *
 * Constraints (RM0394):
 *   f_VCO_in  ∈ [4, 16] MHz            → 16 MHz ✓
 *   f_VCO_out ∈ [64, 344] MHz          → 160 MHz ✓
 *   f_SYSCLK  ≤ 80 MHz (Range 1)       → 80 MHz ✓
 *   Flash latency: 4 WS for 64–80 MHz  (Table 9, RM0394)
 *
 * Delay timer — TIM6 (basic timer, RM0394 §29):
 *   TIM6 is a 16-bit up-counter on APB1 with no output channels.
 *   It is configured here in one-pulse mode (OPM) for blocking delays.
 *
 *   Timer clock = PCLK1 / (PSC + 1) = 80 MHz / 80 = 1 MHz  → 1 tick = 1 µs
 *   OPM: the counter counts from 0 to ARR, sets UIF, then stops (CEN cleared).
 *   Delay period = (ARR + 1) timer ticks = (ARR + 1) µs.
 *   Maximum single delay_us call: 65535 µs (16-bit ARR limit).
 *   For longer delays, use delay_ms, which loops in 1 ms increments.
 */

#include "clock.h"
#include "stm32l452xx.h"


/* ---------------------------------------------------------------------- */
void SystemClock_Config(void)
{
    /* 1. Voltage scaling: Range 1 allows up to 80 MHz.
     *    Default after reset is already Range 1, but set it explicitly
     *    in case we were in Range 2 (e.g. waking from low-power mode).   */
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
    __DSB();
    PWR->CR1 = (PWR->CR1 & ~PWR_CR1_VOS_Msk) | (1UL << PWR_CR1_VOS_Pos);
    while (PWR->SR2 & PWR_SR2_VOSF);   /* wait for regulator to settle   */

    /* 2. Set flash latency BEFORE increasing the clock.
     *    Also enable instruction + data caches and prefetch buffer.      */
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY_Msk)
               | FLASH_ACR_LATENCY_4WS
               | FLASH_ACR_ICEN
               | FLASH_ACR_DCEN
               | FLASH_ACR_PRFTEN;
    /* Verify the latency was accepted (read-back required per RM0394)    */
    while ((FLASH->ACR & FLASH_ACR_LATENCY_Msk) != FLASH_ACR_LATENCY_4WS);

    /* 3. Enable HSI16 (16 MHz internal RC)                               */
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    /* 4. Disable PLL before reconfiguring (required by hardware)         */
    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY);   /* wait for PLL to unlock         */

    /* 5. Configure PLL
     *    Bits [1:0]   PLLSRC = 10  → HSI16
     *    Bits [7:4]   PLLM   = 0   → /1  (0000 = divide-by-1)
     *    Bits [14:8]  PLLN   = 10  → ×10
     *    Bits [26:25] PLLR   = 00  → /2  (00 = divide-by-2)
     *    Bit  24      PLLREN = 1   → enable PLLR output to SYSCLK        */
    RCC->PLLCFGR =
        (2UL  << RCC_PLLCFGR_PLLSRC_Pos) |   /* HSI16             */
        (0UL  << RCC_PLLCFGR_PLLM_Pos)   |   /* M = 1             */
        (10UL << RCC_PLLCFGR_PLLN_Pos)   |   /* N = 10            */
        (0UL  << RCC_PLLCFGR_PLLR_Pos)   |   /* R = /2            */
        RCC_PLLCFGR_PLLREN;                   /* enable PLLR       */

    /* 6. Enable PLL and wait for lock                                    */
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    /* 7. AHB / APB prescalers = /1 (all buses run at 80 MHz)
     *    HPRE[7:4]   = 0000 → /1
     *    PPRE1[10:8] = 000  → /1
     *    PPRE2[13:11]= 000  → /1                                         */
    RCC->CFGR &= ~( RCC_CFGR_HPRE_Msk
                  | RCC_CFGR_PPRE1_Msk
                  | RCC_CFGR_PPRE2_Msk );

    /* 8. Switch SYSCLK source to PLL
     *    SW[1:0] = 11 → PLL                                              */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL);


    /* ---------------  DELAY TIMER (TIM6)  ----------------------------- */

    /* Enable TIM6 clock on APB1 (RM0394 §6.4.18)                        */
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
    __DSB();   /* ensure the clock enable is visible before register access */

    /* Reset TIM6 to a known state                                        */
    TIM6->CR1  = 0;
    TIM6->CR2  = 0;
    TIM6->DIER = 0;

    /*
     * Set prescaler: PSC = 79
     *   Timer clock = PCLK1 / (PSC + 1) = 80 MHz / 80 = 1 MHz
     *   1 timer tick = 1 µs
     *
     * PSC is a 16-bit preload register. It is loaded into the active
     * prescaler on the next update event (generated below via EGR_UG).
     */
    TIM6->PSC = 79U;

    /*
     * Enable one-pulse mode (OPM):
     *   CR1 bit 3: OPM — One-pulse mode
     *   0: Counter is not stopped at update event
     *   1: Counter stops counting at the next update event (clearing CEN)
     *
     * With OPM, the timer counts from 0 to ARR, sets UIF, then stops
     * automatically. We do not need to manually clear CEN after a delay.
     */
    TIM6->CR1 = TIM_CR1_OPM;

    /*
     * Generate an update event to push the PSC preload value into the
     * active prescaler register. The counter is also reset to 0 and UIF
     * is set as a side-effect — clear it afterwards.
     */
    TIM6->EGR = TIM_EGR_UG;
    TIM6->SR  = 0;   /* clear UIF that the EGR write just set             */
}


/* ---------------------------------------------------------------------- */
/**
 * delay_us — blocking delay of `us` microseconds using TIM6.
 *
 * Implementation:
 *   TIM6 runs at 1 MHz (1 tick = 1 µs), one-pulse mode.
 *   ARR = us - 1  →  period = (ARR + 1) ticks = us µs.
 *
 * The timer counts from 0 to ARR, then sets UIF and stops (CEN cleared).
 * We poll UIF to detect completion.
 *
 * Note: us = 0 returns immediately. Maximum value: 65535 µs.
 *       Interrupts are not disabled; an ISR firing mid-delay will extend it.
 */
void delay_us(uint32_t us)
{
    if (us == 0) return;

    /*
     * ARR — Auto-reload register (16-bit).
     *   The timer counts up to ARR, then fires UIF.
     *   Period = (ARR + 1) timer ticks.
     *   With ARR = us - 1: period = us ticks = us µs at 1 MHz.
     *
     * Because ARPE = 0 (default, no buffering), writing ARR takes effect
     * immediately — no EGR_UG needed for the ARR update itself.
     */
    TIM6->ARR = (uint16_t)(us - 1U);

    /* Reset the counter. Without this, CNT would start from wherever the
     * previous delay left it (which is at ARR after OPM stops).          */
    TIM6->CNT = 0;

    /* Clear any stale UIF from a previous delay before starting          */
    TIM6->SR  = 0;

    /* Start the timer. CEN will be cleared automatically when UIF fires. */
    TIM6->CR1 |= TIM_CR1_CEN;

    /* Poll until the update interrupt flag is set (= delay complete)     */
    while (!(TIM6->SR & TIM_SR_UIF));
}


/* ---------------------------------------------------------------------- */
/**
 * delay_ms — blocking delay of `ms` milliseconds.
 *
 * Implemented as a loop of 1000 µs calls to stay within delay_us's
 * 65535 µs single-shot limit.
 */
void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) {
        delay_us(1000U);
    }
}
