#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/**
 * Differential drive — SN754410 dual H-bridge
 *
 * Pin mapping:
 *
 *   Left motor (channels 1+2 of SN754410)
 *     SN754410 pin 1  (1,2EN) ← PA6  TIM3_CH1  PWM — speed
 *     SN754410 pin 2  (1A)    ← PB0  GPIO out  — direction
 *     SN754410 pin 7  (2A)    ← PB1  GPIO out  — direction
 *
 *   Right motor (channels 3+4 of SN754410)
 *     SN754410 pin 9  (3,4EN) ← PA7  TIM3_CH2  PWM — speed
 *     SN754410 pin 10 (3A)    ← PB4  GPIO out  — direction
 *     SN754410 pin 15 (4A)    ← PB5  GPIO out  — direction
 *
 * PWM frequency: 20 kHz  (80 MHz / 4 / 1000)
 * Speed range:   0 – 1000  (0 = coast, 1000 = 100 % duty)
 *
 * Truth table (forward = 1A=H, 2A=L):
 *   FORWARD : IN_A=H, IN_B=L, EN=PWM
 *   BACKWARD: IN_A=L, IN_B=H, EN=PWM
 *   BRAKE   : IN_A=H, IN_B=H, EN=H  (motor shorted → active braking)
 *   STOP    : EN=0              (motor coasts to a stop)
 */

typedef enum {
    MOTOR_FORWARD  = 0,
    MOTOR_BACKWARD = 1,
    MOTOR_BRAKE    = 2,
    MOTOR_STOP     = 3,
} Motor_Dir;

/** Configure GPIO + TIM3; call once after SystemClock_Config(). */
void Motor_Init(void);

/**
 * Motor_SetLeft / Motor_SetRight
 *   dir   — MOTOR_FORWARD, MOTOR_BACKWARD, MOTOR_BRAKE, or MOTOR_STOP
 *   speed — 0 to 1000  (clamped if out of range)
 */
void Motor_SetLeft (Motor_Dir dir, uint16_t speed);
void Motor_SetRight(Motor_Dir dir, uint16_t speed);

/** Immediately coast both motors (EN = 0). */
void Motor_Stop(void);

#endif /* MOTOR_H */
