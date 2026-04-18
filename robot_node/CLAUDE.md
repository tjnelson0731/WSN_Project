# CLAUDE.md — Robot Node Firmware

## Project Overview

Bare-metal C firmware for the **STM32L452RE Nucleo** board.
A wireless sensor robot node with the following subsystems:

| Subsystem        | Hardware                        | Interface      |
|------------------|---------------------------------|----------------|
| Temperature      | LM34 (analog)                   | ADC1, PA0      |
| IMU              | ICM-20948                       | SPI or I2C     |
| Wireless TX      | ESP32-C6 (Zigbee)               | USART2 PA2/PA3 |
| Motor drive      | SN754410 H-bridge (diff. drive) | TIM3 PWM + GPIO|

**Toolchain:** `arm-none-eabi-gcc`, `make`, OpenOCD, VS Code cortex-debug extension.
**Register access:** CMSIS header `stm32l452xx.h` — all peripheral registers are accessed through CMSIS-defined structs.

---

## Hard Rules

### No DWT / No CoreDebug
- Do **not** use `DWT`, `CoreDebug`, `ITM`, or any other ARM core-debug feature.
- These are documented in the ARM TRM / ARMv7-M Architecture RM, **not** in ST's RM0394.
- Use only peripherals found in **RM0394** (the STM32L4 reference manual).
- For microsecond / millisecond delays, use **TIM6** (basic timer on APB1).
  See `Src/clock.c` for the implementation: OPM, 1 MHz clock, poll UIF.

### GPIO Outputs — ODR Only
- Set and clear GPIO output pins using **read-modify-write on ODR**:
  ```c
  GPIOB->ODR |=  (1UL << PIN);   /* set HIGH   */
  GPIOB->ODR &= ~(1UL << PIN);   /* set LOW    */
  ```
- Do **not** use `BSRR` or `BRR`. This is a project convention.
- Note: ODR read-modify-write is not atomic. Do not drive the same pins from both
  an ISR and the main loop without a critical section.

---

## Code Style

Match the style of `blade.c` (a reference file from a previous project).
Match the commenting style already present in this project (or improve on it).

### Section Headers
Use a consistent separator style for logical sections within a function:
```c
/* ----  SECTION NAME  ----------------------------------------------- */
```

### Register-Level Comments
Every register write must have a comment explaining **which bits** are being set
and **what they mean**. Reference RM0394 encoding directly:
```c
/*
 * CCMR1: configure CH1 as PWM Mode 1
 *   OC1M[2:0] = 110 → PWM mode 1 (output HIGH while CNT < CCR1)
 *   OC1PE     = 1   → CCR1 preload enable (updates take effect on UEV)
 */
TIM3->CCMR1 = (6UL << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
```

### General Conventions
- Magic numbers are replaced with named constants or explained inline.
- All `__DSB()` barriers after clock-enable writes are kept (prevents compiler/bus reordering).
- `volatile` is used wherever hardware state can change without software action.
- `uint32_t`, `uint16_t`, etc. from `<stdint.h>` — no bare `int` for register-width values.
- No `printf`, no heap, no stdlib. Integer arithmetic only (no FPU dependency).

---

## Peripheral Notes

### Delays — TIM6 (clock.c)
- Initialised inside `SystemClock_Config()` — no separate init call needed.
- `delay_us(uint32_t us)` — up to 65535 µs.
- `delay_ms(uint32_t ms)` — loops in 1 ms increments, no practical limit.

### System Clock (clock.c)
- HSI16 → PLL → 80 MHz SYSCLK.
- HCLK = PCLK1 = PCLK2 = 80 MHz (all prescalers = /1).
- Flash: 4 wait states, I/D caches enabled, prefetch enabled.
- `SystemCoreClock` = 80000000UL.

### Motors — TIM3 + SN754410 (motor.c)
- Left:  EN = PA6 (TIM3_CH1 AF2), IN_A = PB0, IN_B = PB1
- Right: EN = PA7 (TIM3_CH2 AF2), IN_A = PB4, IN_B = PB5
- PWM: 1 kHz, speed range 0–1000.

### UART — USART2 (uart.c)
- PA2 TX (AF7) → ESP32-C6 RX
- PA3 RX (AF7) ← ESP32-C6 TX
- 115200 8N1, polling (no DMA/IRQ).

### LM34 — ADC1 (lm34.c)
- PA0 → ADC1_IN5, 12-bit, software trigger, 92.5-cycle sample time.
- ADC clock: HCLK/4 = 20 MHz (CKMODE = 0b11 in ADC_CCR).
- Output format: integer tenths-of-degree (e.g. 734 = 73.4 °F).

### ICM-20948 — (not yet implemented)
- To be added. Plan to use SPI or I2C (TBD).

---

## What SCB Is (and Why It's Not in RM0394)

`SCB` (System Control Block) is an ARM Cortex-M4 core register block at `0xE000ED00`.
It is defined in CMSIS `core_cm4.h`, and documented in the *ARM Cortex-M4 TRM* and
*ARMv7-M Architecture Reference Manual* — **not** in ST's RM0394.

RM0394 only covers STM32-specific peripherals (timers, USART, ADC, etc.).
ARM core features — `SCB`, `DWT`, `CoreDebug`, `ITM`, `SysTick`, `NVIC` — are
all ARM IP and live in a separate address space (`0xE0000000`–`0xE00FFFFF`).
