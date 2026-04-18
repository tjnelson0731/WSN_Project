/**
 * motor.c — Differential drive via SN754410 H-bridge
 *
 * Hardware assignments:
 *   Left  motor: EN ← PA6  (TIM3_CH1, AF2), IN_A ← PB0, IN_B ← PB1
 *   Right motor: EN ← PA7  (TIM3_CH2, AF2), IN_A ← PB4, IN_B ← PB5
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
 * SN754410 direction truth table (per motor):
 *   IN_A  IN_B  EN   Motor state
 *    H     L    PWM  Forward
 *    L     H    PWM  Backward
 *    H     H    H    Active brake (output shorted)
 *    X     X    L    Coast (output disabled)
 *
 * GPIO output convention:
 *   Direction pins (PB0, PB1, PB4, PB5) are driven via ODR read-modify-write.
 *   This is not atomic — avoid calling Motor_Set* from an ISR if the main
 *   loop is also driving these pins concurrently.
 */

#include "stm32l452xx.h"
#include "motor.h"

/* ---- Pin definitions -------------------------------------------------- */
#define L_INA_PIN   0U   /* PB0 */
#define L_INB_PIN   1U   /* PB1 */
#define R_INA_PIN   4U   /* PB4 */
#define R_INB_PIN   5U   /* PB5 */

#define MAX_SPEED   1000U

/* ---------------------------------------------------------------------- */
void Motor_Init(void)
{
    /* Enable peripheral clocks */
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
     * OSPEEDR: high-speed output for PWM signals
     *   OSPEEDRx[1:0] = 11 → Very high speed
     */
    GPIOA->OSPEEDR |= (3UL << GPIO_OSPEEDR_OSPEED6_Pos)
                   |  (3UL << GPIO_OSPEEDR_OSPEED7_Pos);


    /* ----  PB0, PB1, PB4, PB5 : GPIO OUTPUT (direction pins)  --------- */

    /*
     * MODER: set direction pins to General Purpose Output mode
     *   MODERx[1:0] = 01 → General purpose output
     */
    GPIOB->MODER = (GPIOB->MODER
                     & ~(  GPIO_MODER_MODE0_Msk | GPIO_MODER_MODE1_Msk
                          | GPIO_MODER_MODE4_Msk | GPIO_MODER_MODE5_Msk))
                 | (1UL << GPIO_MODER_MODE0_Pos)
                 | (1UL << GPIO_MODER_MODE1_Pos)
                 | (1UL << GPIO_MODER_MODE4_Pos)
                 | (1UL << GPIO_MODER_MODE5_Pos);

    /*
     * OTYPER: push-pull output (OTy = 0) — required for active drive.
     * PUPDR:  no pull-up/pull-down (PUPDy[1:0] = 00).
     */
    GPIOB->OTYPER &= ~((1UL << L_INA_PIN) | (1UL << L_INB_PIN)
                      |(1UL << R_INA_PIN) | (1UL << R_INB_PIN));
    GPIOB->PUPDR  &= ~(  GPIO_PUPDR_PUPD0_Msk | GPIO_PUPDR_PUPD1_Msk
                        | GPIO_PUPDR_PUPD4_Msk | GPIO_PUPDR_PUPD5_Msk);

    /*
     * ODR: drive all direction pins LOW on startup so that the EN PWM
     * signal cannot accidentally energise the motors before Motor_Set
     * is called (coast state: IN_A = L, IN_B = L).
     */
    GPIOB->ODR &= ~((1UL << L_INA_PIN) | (1UL << L_INB_PIN)
                  | (1UL << R_INA_PIN) | (1UL << R_INB_PIN));


    /* ----  TIM3 : 1 kHz PWM ON CH1 AND CH2  --------------------------- */

    TIM3->CR1  = 0;     /* disable timer while configuring                */
    TIM3->PSC  = 3U;    /* 80 MHz / (3+1) = 20 MHz timer clock             */
    TIM3->ARR  = 999U;  /* 20 MHz / (999+1) = 20 kHz PWM frequency         */
    TIM3->CCR1 = 0U;    /* left motor duty = 0 % (stopped)                */
    TIM3->CCR2 = 0U;    /* right motor duty = 0 % (stopped)               */

    /*
     * CCMR1: configure CH1 and CH2 as PWM Mode 1
     *
     *   OC1M[2:0] = 110 → PWM mode 1 (output HIGH while CNT < CCR1)
     *   OC1PE     = 1   → CCR1 preload enable (updates take effect on UEV)
     *   OC2M[2:0] = 110 → same for CH2
     *   OC2PE     = 1   → CCR2 preload enable
     *
     * OC1M[3] (bit 16) and OC2M[3] (bit 24) remain 0 → standard PWM mode 1
     * (not the extended mode available on some channels).
     */
    TIM3->CCMR1 = (6UL << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE
                | (6UL << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;

    /*
     * CCER: enable CH1 and CH2 outputs, active HIGH polarity
     *   CC1E = 1 → CH1 output enabled
     *   CC2E = 1 → CH2 output enabled
     *   CC1P / CC2P remain 0 → active HIGH
     */
    TIM3->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E;

    /*
     * CR1: enable auto-reload preload so ARR changes take effect only
     * on the next update event, preventing mid-period glitches.
     *   ARPE = 1 → TIMx_ARR register is buffered
     */
    TIM3->CR1   = TIM_CR1_ARPE;

    /* Generate an update event to transfer all preload values (PSC, ARR,
     * CCR1, CCR2) into their active shadow registers before enabling.    */
    TIM3->EGR   = TIM_EGR_UG;
    TIM3->SR    = 0;    /* clear the UIF flag that EGR_UG just set        */

    /* Start the timer — PWM outputs become active                        */
    TIM3->CR1  |= TIM_CR1_CEN;
}


/* ---------------------------------------------------------------------- */
void Motor_SetLeft(Motor_Dir dir, uint16_t speed)
{
    if (speed > MAX_SPEED) speed = (uint16_t)MAX_SPEED;

    switch (dir) {

    case MOTOR_FORWARD:
        /* IN_A = H, IN_B = L → forward spin                             */
        GPIOB->ODR |=  (1UL << L_INA_PIN);
        GPIOB->ODR &= ~(1UL << L_INB_PIN);
        TIM3->CCR1  = speed;
        break;

    case MOTOR_BACKWARD:
        /* IN_A = L, IN_B = H → reverse spin                             */
        GPIOB->ODR &= ~(1UL << L_INA_PIN);
        GPIOB->ODR |=  (1UL << L_INB_PIN);
        TIM3->CCR1  = speed;
        break;

    case MOTOR_BRAKE:
        /* IN_A = H, IN_B = H, EN = 100 % → both outputs shorted         */
        GPIOB->ODR |=  (1UL << L_INA_PIN) | (1UL << L_INB_PIN);
        TIM3->CCR1  = MAX_SPEED;
        break;

    case MOTOR_STOP:
    default:
        /* EN = 0 → H-bridge disabled, motor coasts                      */
        TIM3->CCR1  = 0;
        GPIOB->ODR &= ~((1UL << L_INA_PIN) | (1UL << L_INB_PIN));
        break;
    }
}


/* ---------------------------------------------------------------------- */
void Motor_SetRight(Motor_Dir dir, uint16_t speed)
{
    if (speed > MAX_SPEED) speed = (uint16_t)MAX_SPEED;

    switch (dir) {

    case MOTOR_FORWARD:
        GPIOB->ODR |=  (1UL << R_INA_PIN);
        GPIOB->ODR &= ~(1UL << R_INB_PIN);
        TIM3->CCR2  = speed;
        break;

    case MOTOR_BACKWARD:
        GPIOB->ODR &= ~(1UL << R_INA_PIN);
        GPIOB->ODR |=  (1UL << R_INB_PIN);
        TIM3->CCR2  = speed;
        break;

    case MOTOR_BRAKE:
        GPIOB->ODR |=  (1UL << R_INA_PIN) | (1UL << R_INB_PIN);
        TIM3->CCR2  = MAX_SPEED;
        break;

    case MOTOR_STOP:
    default:
        TIM3->CCR2  = 0;
        GPIOB->ODR &= ~((1UL << R_INA_PIN) | (1UL << R_INB_PIN));
        break;
    }
}


/* ---------------------------------------------------------------------- */
void Motor_Stop(void)
{
    /* Disable both EN signals first — motors coast immediately           */
    TIM3->CCR1 = 0;
    TIM3->CCR2 = 0;
    GPIOB->ODR &= ~((1UL << L_INA_PIN) | (1UL << L_INB_PIN)
                  | (1UL << R_INA_PIN) | (1UL << R_INB_PIN));
}
