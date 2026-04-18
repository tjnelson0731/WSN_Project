/**
 * system_stm32l4xx.c — minimal stub
 *
 * The CMSIS startup file calls SystemInit() before main().  We keep it
 * empty here because all clock configuration happens explicitly inside
 * SystemClock_Config() in clock.c, so you know exactly what's running.
 *
 * SystemCoreClock is updated to 80 000 000 inside SystemClock_Config().
 * Its initial value here matches the reset default (MSI @ 4 MHz).
 */

#include "stm32l452xx.h"
#include "stdint.h"

void SystemInit(void)
{
    /*
     * FPU access: enable CP10 / CP11 for full access.
     * The Cortex-M4 FPU is gated by the CPACR register; without this the
     * first floating-point instruction causes a UsageFault.
     */
    SCB->CPACR |= (3UL << 20U) | (3UL << 22U);
    __DSB();
    __ISB();

    /* All other initialisation (clock, flash latency, voltage scaling)
     * is done in main() → SystemClock_Config(). */
}
