/**
 * icm20948.c — ICM-20948 + AK09916 driver (I2C1, 400 kHz)
 *
 * Pins:
 *   PB6  I2C1_SCL  AF4  →  ICM-20948 SCL  +  AK09916 ACL  (shared bus)
 *   PB7  I2C1_SDA  AF4  →  ICM-20948 SDA  +  AK09916 ADA  (shared bus)
 *
 * The Teyleten Robot breakout board has a solder bridge that disconnects
 * the AK09916 from the ICM-20948's internal I2C master and routes it to
 * the external ADA/ACL header pins.  Both devices therefore live on the
 * same I2C1 bus at separate 7-bit addresses (0x68 and 0x0C); they are
 * accessed with independent I2C transactions — no ICM-20948 Bank 3 or
 * I2C master configuration is required or used.
 *
 * I2C timing at PCLK1 = 80 MHz — TIMINGR = 0x10422F33:
 *   PRESC  = 1  → t_tick = 25 ns
 *   SCLDEL = 4  → t_SCLDEL = 125 ns  (spec ≥ 100 ns)
 *   SDADEL = 2  → t_SDADEL =  50 ns  (spec ≥   0 ns)
 *   SCLH   = 47 → t_SCLH  = 1200 ns  (spec ≥ 600 ns)
 *   SCLL   = 51 → t_SCLL  = 1300 ns  (spec ≥ 1300 ns)
 *   f_SCL  = 1 / ((48 + 52) × 25 ns) = 400 kHz
 *
 * Transfer model (polling, no DMA/IRQ):
 *   Write: START → addr+W → reg → data → STOP         (AUTOEND=1)
 *   Read:  START → addr+W → reg → TC
 *          → repeated START → addr+R → data[n] → STOP (AUTOEND=1)
 *
 * ICM-20948 register bank model:
 *   REG_BANK_SEL (0x7F) is present at the same address in all four banks.
 *   USER_BANK[5:4] selects the visible bank.  Output registers are in
 *   Bank 0; accel/gyro config registers are in Bank 2.  Bank 0 is always
 *   restored on exit from any function that switches banks.
 *
 * AK09916 register model:
 *   The AK09916 is accessed directly at 0x0C; its registers are flat
 *   (no bank switching).  A single 9-byte burst starting at ST1 (0x10)
 *   covers ST1, HXL, HXH, HYL, HYH, HZL, HZH, TMPS, ST2.  Reading
 *   ST2 is mandatory — it releases the AK09916's internal data register
 *   latch so the next measurement can begin.
 *
 * Initialization sequence:
 *   1.  Configure GPIO (PB6/PB7) and I2C1
 *   2.  ICM-20948: select Bank 0, verify WHO_AM_I
 *   3.  ICM-20948: DEVICE_RESET, wake, CLKSEL=auto-PLL
 *   4.  ICM-20948: enable all axes (PWR_MGMT_2 = 0x00)
 *   5.  ICM-20948: Bank 2 — GYRO_CONFIG_1, ACCEL_CONFIG; restore Bank 0
 *   6.  AK09916: soft reset, wait, continuous mode 2 (20 Hz)
 */

#include "stm32l452xx.h"
#include "icm20948.h"
#include "clock.h"

/* ---- TIMINGR (see file header for derivation) -------------------------- */
#define I2C1_TIMINGR_400KHZ     0x10422F33UL

/* ---------------------------------------------------------------------- */
/*  Generic I2C primitives — parameterised by 7-bit device address         */
/* ---------------------------------------------------------------------- */

/**
 * i2c_write_reg()
 *   Write one byte to register `reg` on the device at `addr`.
 *   Generates: START → addr+W → reg → val → STOP
 */
static void i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val)
{
    /* Wait until bus is idle */
    while (I2C1->ISR & I2C_ISR_BUSY);

    /*
     * CR2:
     *   SADD[7:1]  = addr (7-bit address, written to bits [7:1])
     *   NBYTES     = 2    → register byte + data byte
     *   RD_WRN     = 0    → write
     *   AUTOEND    = 1    → hardware generates STOP after NBYTES
     *   START      = 1    → generate START now
     */
    I2C1->CR2 = ((uint32_t)addr << 1)
              | (2UL << I2C_CR2_NBYTES_Pos)
              | I2C_CR2_AUTOEND
              | I2C_CR2_START;

    while (!(I2C1->ISR & I2C_ISR_TXIS));
    I2C1->TXDR = reg;

    while (!(I2C1->ISR & I2C_ISR_TXIS));
    I2C1->TXDR = val;

    while (!(I2C1->ISR & I2C_ISR_STOPF));
    I2C1->ICR = I2C_ICR_STOPCF;   /* clear STOPF (write-1-to-clear) */
}

/**
 * i2c_read_regs()
 *   Burst-read `len` bytes from `addr` starting at register `reg`.
 *   Generates: START → addr+W → reg → TC
 *              → repeated START → addr+R → buf[0..len-1] → STOP
 */
static void i2c_read_regs(uint8_t addr, uint8_t reg,
                           uint8_t *buf, uint32_t len)
{
    /* Wait until bus is idle */
    while (I2C1->ISR & I2C_ISR_BUSY);

    /* ----  Phase 1: set register pointer  -------------------------------- */
    /*
     * CR2:
     *   NBYTES  = 1    → just the register address byte
     *   AUTOEND = 0    → no STOP; hardware holds SCL after the last byte
     *   START   = 1    → generate START
     */
    I2C1->CR2 = ((uint32_t)addr << 1)
              | (1UL << I2C_CR2_NBYTES_Pos)
              | I2C_CR2_START;

    while (!(I2C1->ISR & I2C_ISR_TXIS));
    I2C1->TXDR = reg;

    /* Wait for TC — SCL stretched, bus held while we arm Phase 2 */
    while (!(I2C1->ISR & I2C_ISR_TC));

    /* ----  Phase 2: repeated START, read len bytes  --------------------- */
    /*
     * CR2:
     *   RD_WRN  = 1    → read
     *   NBYTES  = len
     *   AUTOEND = 1    → NACK last byte and generate STOP automatically
     *   START   = 1    → generate repeated START
     */
    I2C1->CR2 = ((uint32_t)addr << 1)
              | ((uint32_t)len << I2C_CR2_NBYTES_Pos)
              | I2C_CR2_RD_WRN
              | I2C_CR2_AUTOEND
              | I2C_CR2_START;

    for (uint32_t i = 0; i < len; i++) {
        while (!(I2C1->ISR & I2C_ISR_RXNE));
        buf[i] = (uint8_t)I2C1->RXDR;
    }

    while (!(I2C1->ISR & I2C_ISR_STOPF));
    I2C1->ICR = I2C_ICR_STOPCF;
}

/* ---------------------------------------------------------------------- */
/*  ICM-20948 convenience wrappers                                         */
/* ---------------------------------------------------------------------- */

static void icm_write_reg(uint8_t reg, uint8_t val)
{
    i2c_write_reg(ICM20948_ADDR, reg, val);
}

static void icm_read_regs(uint8_t reg, uint8_t *buf, uint32_t len)
{
    i2c_read_regs(ICM20948_ADDR, reg, buf, len);
}

static void icm_select_bank(uint8_t bank)
{
    /*
     * REG_BANK_SEL (0x7F) — USER_BANK[5:4]:
     *   0x00 = bank 0, 0x20 = bank 2
     */
    icm_write_reg(ICM_B0_REG_BANK_SEL, bank);
}

/* ---------------------------------------------------------------------- */
/*  AK09916 convenience wrappers                                           */
/* ---------------------------------------------------------------------- */

static void ak_write_reg(uint8_t reg, uint8_t val)
{
    i2c_write_reg(AK09916_ADDR, reg, val);
}

static void ak_read_regs(uint8_t reg, uint8_t *buf, uint32_t len)
{
    i2c_read_regs(AK09916_ADDR, reg, buf, len);
}

/* ---------------------------------------------------------------------- */
int ICM20948_Init(void)
{
    /* ---- Enable peripheral clocks ------------------------------------- */
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOBEN;   /* GPIOB — PB6, PB7           */
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;   /* I2C1  — APB1 bit 21        */
    __DSB();

    /* ---- PB6 (SCL) and PB7 (SDA): alternate function 4, open-drain ---- */
    /*
     * MODER[13:12] = 10 → PB6 alternate function
     * MODER[15:14] = 10 → PB7 alternate function
     */
    GPIOB->MODER = (GPIOB->MODER
                    & ~(GPIO_MODER_MODE6_Msk | GPIO_MODER_MODE7_Msk))
                 | (2UL << GPIO_MODER_MODE6_Pos)
                 | (2UL << GPIO_MODER_MODE7_Pos);

    /*
     * OTYPER: OT6 = 1, OT7 = 1 → open-drain.
     * Required for I2C so any device on the bus can pull the line low.
     */
    GPIOB->OTYPER |= (1UL << 6) | (1UL << 7);

    /*
     * OSPEEDR: OSPEED6 = 11, OSPEED7 = 11 → high speed.
     * Fast slew rate needed at 400 kHz with line capacitance.
     */
    GPIOB->OSPEEDR |= (3UL << GPIO_OSPEEDR_OSPEED6_Pos)
                   |  (3UL << GPIO_OSPEEDR_OSPEED7_Pos);

    /*
     * PUPDR: PUPD6 = 01, PUPD7 = 01 → internal pull-up.
     * Open-drain lines require a pull-up.  Internal pull-ups (~40 kΩ) are
     * usable for bench testing; add external 4.7 kΩ for production.
     */
    GPIOB->PUPDR = (GPIOB->PUPDR
                    & ~(GPIO_PUPDR_PUPD6_Msk | GPIO_PUPDR_PUPD7_Msk))
                 | (1UL << GPIO_PUPDR_PUPD6_Pos)
                 | (1UL << GPIO_PUPDR_PUPD7_Pos);

    /*
     * AFRL: AFSEL6 = 4, AFSEL7 = 4 → I2C1 (AF4).
     * RM0394 Table 17: AF4 is I2C1 for PB6/PB7 on STM32L452.
     */
    GPIOB->AFR[0] = (GPIOB->AFR[0]
                     & ~(GPIO_AFRL_AFSEL6_Msk | GPIO_AFRL_AFSEL7_Msk))
                  | (4UL << GPIO_AFRL_AFSEL6_Pos)
                  | (4UL << GPIO_AFRL_AFSEL7_Pos);

    /* ---- Configure I2C1 ----------------------------------------------- */
    /*
     * CR1: PE = 0 — I2C1 must be disabled before writing TIMINGR.
     * RM0394 §37.7.1: TIMINGR is write-protected while PE = 1.
     */
    I2C1->CR1 = 0;

    /*
     * TIMINGR = 0x10422F33 — 400 kHz at PCLK1 = 80 MHz.
     * Full derivation in the file header.
     */
    I2C1->TIMINGR = I2C1_TIMINGR_400KHZ;

    /*
     * CR1: PE = 1 → enable I2C1.
     *   ANFOFF = 0 → analog noise filter on (default, recommended)
     *   DNF    = 0 → digital filter off
     *   No IRQ or DMA — polling mode throughout.
     */
    I2C1->CR1 = I2C_CR1_PE;

    /* ---- ICM-20948 initialization ------------------------------------- */

    /* Select Bank 0 explicitly — the device may be in any bank           */
    icm_select_bank(ICM_BANK_SEL_0);

    /* Verify WHO_AM_I — returns 0xEA if the device is present and wired correctly */
    uint8_t who_am_i = 0;
    icm_read_regs(ICM_B0_WHO_AM_I, &who_am_i, 1);
    if (who_am_i != ICM_WHO_AM_I_VAL) {
        return 1;   /* ICM-20948 not found — check PB6/PB7 wiring          */
    }

    /* Reset all ICM-20948 registers to defaults                           */
    /*
     * PWR_MGMT_1:
     *   Bit 7 DEVICE_RESET = 1 → resets all registers; self-clears.
     *   After reset: SLEEP = 1 (device asleep).
     */
    icm_write_reg(ICM_B0_PWR_MGMT_1, ICM_PWR_MGMT_1_DEVICE_RESET);
    delay_ms(100U);   /* datasheet: startup time ≤ 100 ms after reset      */

    /* Wake the device and select the best available clock                 */
    /*
     * PWR_MGMT_1 = 0x01:
     *   Bit 6    SLEEP  = 0   → exit sleep mode
     *   Bits[2:0] CLKSEL = 001 → auto-select PLL if ready, else internal.
     *   Datasheet recommends CLKSEL = 1 for lowest gyroscope noise.
     */
    icm_write_reg(ICM_B0_PWR_MGMT_1, ICM_PWR_MGMT_1_CLKSEL_AUTO);
    delay_ms(10U);   /* allow PLL to lock */

    /* Enable all accelerometer and gyroscope axes                         */
    /*
     * PWR_MGMT_2 = 0x00:
     *   Bits[5:3] DISABLE_ACCEL = 000 → all three accel axes on
     *   Bits[2:0] DISABLE_GYRO  = 000 → all three gyro axes on
     */
    icm_write_reg(ICM_B0_PWR_MGMT_2, 0x00U);

    /* ---- Accel and gyro configuration (Bank 2 registers) -------------- */

    icm_select_bank(ICM_BANK_SEL_2);

    /*
     * GYRO_CONFIG_1 (Bank 2, 0x01):
     *   Bits[5:3] GYRO_DLPFCFG = 011 → ~51.2 Hz 3-dB bandwidth,
     *                                   1.125 kHz output data rate
     *   Bits[2:1] GYRO_FS_SEL  = 01  → ±500 dps, 65.5 LSB/(°/s)
     *   Bit[0]    GYRO_FCHOICE = 1   → route through DLPF
     */
    icm_write_reg(ICM_B2_GYRO_CONFIG_1, ICM_GYRO_CONFIG_1_VAL);

    /*
     * ACCEL_CONFIG (Bank 2, 0x14):
     *   Bits[5:3] ACCEL_DLPFCFG = 011 → ~50.4 Hz 3-dB bandwidth,
     *                                    1.125 kHz output data rate
     *   Bits[2:1] ACCEL_FS_SEL  = 01  → ±4 g, 8192 LSB/g
     *   Bit[0]    ACCEL_FCHOICE = 1   → route through DLPF
     */
    icm_write_reg(ICM_B2_ACCEL_CONFIG, ICM_ACCEL_CONFIG_VAL);

    icm_select_bank(ICM_BANK_SEL_0);

    /* ---- AK09916 initialization (direct I2C at 0x0C) ------------------ */

    /* Soft-reset the AK09916                                              */
    /*
     * AK_CNTL3:
     *   Bit 0 SRST = 1 → reset all AK09916 registers; self-clears.
     *   Datasheet: reset completes within 100 µs.
     */
    ak_write_reg(AK_CNTL3, AK_CNTL3_SRST);
    delay_ms(10U);   /* comfortably longer than the 100 µs reset time      */

    /* Place AK09916 in 20 Hz continuous measurement mode                 */
    /*
     * AK_CNTL2 MODE[4:0] = 0x04 → Continuous Measurement Mode 2 (20 Hz).
     * The AK09916 will continuously update HX/HY/HZ at 20 Hz.
     * ST2 must be read after each sample to arm the next measurement;
     * ICM20948_ReadMag() includes ST2 in its 9-byte burst, handling
     * this automatically.
     */
    ak_write_reg(AK_CNTL2, AK_MODE_CONT_20HZ);

    /* Wait one full measurement period before the first read              */
    delay_ms(50U);   /* 20 Hz period = 50 ms                               */

    return 0;
}

/* ---------------------------------------------------------------------- */
void ICM20948_ReadAccel(ICM20948_Vec3 *a)
{
    /*
     * 6-byte burst from ACCEL_XOUT_H (Bank 0, 0x2D), big-endian:
     *   buf[0..1]: AX   buf[2..3]: AY   buf[4..5]: AZ
     */
    uint8_t buf[6];
    icm_read_regs(ICM_B0_ACCEL_XOUT_H, buf, 6U);

    a->x = (int16_t)((uint16_t)buf[0] << 8 | buf[1]);
    a->y = (int16_t)((uint16_t)buf[2] << 8 | buf[3]);
    a->z = (int16_t)((uint16_t)buf[4] << 8 | buf[5]);
}

/* ---------------------------------------------------------------------- */
void ICM20948_ReadGyro(ICM20948_Vec3 *g)
{
    /*
     * 6-byte burst from GYRO_XOUT_H (Bank 0, 0x33), big-endian:
     *   buf[0..1]: GX   buf[2..3]: GY   buf[4..5]: GZ
     */
    uint8_t buf[6];
    icm_read_regs(ICM_B0_GYRO_XOUT_H, buf, 6U);

    g->x = (int16_t)((uint16_t)buf[0] << 8 | buf[1]);
    g->y = (int16_t)((uint16_t)buf[2] << 8 | buf[3]);
    g->z = (int16_t)((uint16_t)buf[4] << 8 | buf[5]);
}

/* ---------------------------------------------------------------------- */
void ICM20948_ReadTemp(int16_t *t)
{
    /*
     * 2-byte burst from TEMP_OUT_H (Bank 0, 0x39), big-endian.
     * Conversion: T_tenths_C = ((int32_t)(*t) * 10) / 334 + 210
     */
    uint8_t buf[2];
    icm_read_regs(ICM_B0_TEMP_OUT_H, buf, 2U);

    *t = (int16_t)((uint16_t)buf[0] << 8 | buf[1]);
}

/* ---------------------------------------------------------------------- */
int ICM20948_ReadMag(ICM20948_Vec3 *m)
{
    /*
     * 9-byte burst from AK09916 starting at ST1 (0x10).
     * AK09916 auto-increments its register pointer:
     *
     *   buf[0]: ST1   — bit 0 = DRDY (1 = new data ready)
     *   buf[1]: HXL   — X LSB  (little-endian: LSB first)
     *   buf[2]: HXH   — X MSB
     *   buf[3]: HYL
     *   buf[4]: HYH
     *   buf[5]: HZL
     *   buf[6]: HZH
     *   buf[7]: TMPS  — reserved, ignored
     *   buf[8]: ST2   — bit 3 = HOFL (magnetic overflow)
     *                   ST2 MUST be read to release the next measurement.
     *
     * Unlike ICM-20948 accel/gyro, AK09916 output is little-endian:
     * the LSB is at the lower register address.
     */
    uint8_t buf[9];
    ak_read_regs(AK_ST1, buf, 9U);

    /* Recombine little-endian byte pairs → signed 16-bit                 */
    m->x = (int16_t)((uint16_t)buf[2] << 8 | buf[1]);
    m->y = (int16_t)((uint16_t)buf[4] << 8 | buf[3]);
    m->z = (int16_t)((uint16_t)buf[6] << 8 | buf[5]);

    /* Return 1 if the magnetic overflow flag is set in ST2 (bit 3 HOFL)  */
    return (buf[8] & (1U << 3)) ? 1 : 0;
}
