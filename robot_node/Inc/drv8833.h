#ifndef DRV8833_H
#define DRV8833_H

#include <stdint.h>

/**
 * Differential drive — DRV8833 dual H-bridge
 *
 * Pin mapping:
 *
 *   Left motor (Bridge A)
 *     AIN1 ← PA6  TIM3_CH1 AF2  — forward PWM
 *     AIN2 ← PA7  TIM3_CH2 AF2  — reverse PWM
 *
 *   Right motor (Bridge B)
 *     BIN1 ← PB0  TIM3_CH3 AF2  — forward PWM
 *     BIN2 ← PB1  TIM3_CH4 AF2  — reverse PWM
 *
 *   STBY ← PB4  GPIO output     — HIGH = active, LOW = sleep
 *
 * PWM frequency : 20 kHz  (80 MHz / 4 / 1000)
 * Speed range   : 0 – 1000  (0 = coast, 1000 = 100 % duty)
 *
 * Decay mode: fast decay (xIN1 = PWM, xIN2 = 0 for forward;
 *                         xIN1 = 0, xIN2 = PWM for reverse).
 * During the off-time the inactive half of the bridge freewheels
 * through the body diodes.
 *
 * DRV8833 H-bridge truth table (per bridge):
 *   xIN1  xIN2  Function
 *    0     0    Coast  (Hi-Z outputs)
 *    0     1    Reverse
 *    1     0    Forward
 *    1     1    Brake  (both outputs LOW, motor shorted to GND)
 *
 * NOTE: This driver uses all four TIM3 channels (CH1–CH4) and
 * conflicts with motor.c (SN754410) which uses CH1–CH2.
 * Link only one motor driver at a time by swapping motor.c ↔
 * drv8833.c in the Makefile C_SOURCES list.
 */

typedef enum {
    DRV8833_FORWARD  = 0,
    DRV8833_BACKWARD = 1,
    DRV8833_BRAKE    = 2,
    DRV8833_COAST    = 3,
} DRV8833_Dir;

/** Configure GPIO + TIM3 + STBY; call once after SystemClock_Config(). */
void DRV8833_Init(void);

/**
 * DRV8833_SetLeft / DRV8833_SetRight
 *   dir   — DRV8833_FORWARD, DRV8833_BACKWARD, DRV8833_BRAKE, or DRV8833_COAST
 *   speed — 0 to 1000  (clamped if out of range; ignored for BRAKE/COAST)
 */
void DRV8833_SetLeft (DRV8833_Dir dir, uint16_t speed);
void DRV8833_SetRight(DRV8833_Dir dir, uint16_t speed);

/** Immediately coast both motors (all IN pins LOW → Hi-Z outputs). */
void DRV8833_Stop(void);

/** Assert STBY LOW — both bridges disabled, device enters low-power sleep. */
void DRV8833_Sleep(void);

/** Assert STBY HIGH — wake device; blocks 1 ms for tWAKE per datasheet. */
void DRV8833_Wake(void);

#endif /* DRV8833_H */
