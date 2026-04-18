#ifndef ICM20948_H
#define ICM20948_H

#include <stdint.h>

/**
 * ICM-20948 9-axis IMU — I2C1 driver (400 kHz Fast Mode)
 *
 * ---- Wiring (Teyleten Robot breakout board) ------------------------------
 *
 * The solder bridge on the back of this board disconnects the AK09916
 * magnetometer from the ICM-20948's internal I2C master (AUX_DA / AUX_CL)
 * and instead routes the AK09916's SDA/SCL lines to the ADA and ACL header
 * pins.  As a result, both the ICM-20948 and the AK09916 appear as
 * independent I2C devices on the same host bus at different addresses:
 *
 *   PB6  I2C1_SCL  AF4  →  ICM-20948 SCL pin
 *                        →  AK09916   ACL pin  (wire both to PB6)
 *   PB7  I2C1_SDA  AF4  →  ICM-20948 SDA pin
 *                        →  AK09916   ADA pin  (wire both to PB7)
 *
 *   ICM-20948  7-bit I2C address: 0x68  (AD0 tied LOW)
 *   AK09916    7-bit I2C address: 0x0C  (hard-wired, not configurable)
 *
 * Both lines are open-drain; one set of pull-up resistors on the shared
 * SCL/SDA wires is sufficient (4.7 kΩ to 3.3 V, or rely on the STM32
 * internal pull-ups for bench testing).
 *
 * ---- Sensor configuration -----------------------------------------------
 *
 *   Gyroscope:     ±500 dps full scale, DLPF enabled at ~51 Hz bandwidth.
 *     Sensitivity: 65.5 LSB/(°/s)
 *
 *   Accelerometer: ±4 g full scale, DLPF enabled at ~50 Hz bandwidth.
 *     Sensitivity: 8192 LSB/g
 *
 *   Magnetometer (AK09916): 20 Hz continuous mode.
 *     Sensitivity: 0.15 µT/LSB, range ±4900 µT.
 *     AK09916 output registers are little-endian (LSB first); the driver
 *     corrects this before returning values.
 *
 *   Temperature (ICM-20948 die sensor):
 *     Conversion to tenths of °C (integer arithmetic):
 *       T_tenths_C = ((int32_t)raw * 10) / 334 + 210
 *
 * ---- Register bank model ------------------------------------------------
 *
 *   ICM-20948 has four register banks switched via REG_BANK_SEL (0x7F).
 *   Output registers are in Bank 0; sensor config registers in Bank 2.
 *   The driver always restores Bank 0 after accessing Bank 2.
 *   Bank 3 (I2C master) is not used — the AK09916 is on the host bus.
 */

/* ---- I2C addresses ----------------------------------------------------- */
#define ICM20948_ADDR           0x68U   /* AD0 = 0                          */
#define AK09916_ADDR            0x0CU   /* hard-wired inside the package    */

/* ---- ICM-20948 Bank 0 register addresses ------------------------------- */
#define ICM_B0_WHO_AM_I         0x00U   /* read-only, always 0xEA           */
#define ICM_B0_PWR_MGMT_1       0x06U
#define ICM_B0_PWR_MGMT_2       0x07U
#define ICM_B0_ACCEL_XOUT_H     0x2DU
#define ICM_B0_GYRO_XOUT_H      0x33U
#define ICM_B0_TEMP_OUT_H       0x39U
#define ICM_B0_REG_BANK_SEL     0x7FU   /* same address in every bank       */

/* ---- ICM-20948 Bank 2 register addresses ------------------------------- */
#define ICM_B2_GYRO_CONFIG_1    0x01U
#define ICM_B2_ACCEL_CONFIG     0x14U

/* ---- REG_BANK_SEL encoded values --------------------------------------- */
#define ICM_BANK_SEL_0          0x00U
#define ICM_BANK_SEL_2          0x20U

/* ---- ICM-20948 register bit fields ------------------------------------- */

/** Expected WHO_AM_I return value. */
#define ICM_WHO_AM_I_VAL        0xEAU

/*
 * PWR_MGMT_1 (Bank 0, 0x06):
 *   Bit 7  DEVICE_RESET : write 1 to reset all registers; self-clears
 *   Bit 6  SLEEP        : 1 = sleep mode (default after power-on)
 *   Bits[2:0] CLKSEL    : 001 = auto-select PLL (recommended)
 */
#define ICM_PWR_MGMT_1_DEVICE_RESET     (1U << 7)
#define ICM_PWR_MGMT_1_CLKSEL_AUTO      (1U << 0)

/*
 * GYRO_CONFIG_1 (Bank 2, 0x01):
 *   Bits[5:3] GYRO_DLPFCFG = 011 → ~51.2 Hz 3-dB BW at 1.125 kHz ODR
 *   Bits[2:1] GYRO_FS_SEL  = 01  → ±500 dps  (65.5 LSB/(°/s))
 *   Bit[0]    GYRO_FCHOICE = 1   → enable DLPF
 */
#define ICM_GYRO_CONFIG_1_VAL   ((3U << 3) | (1U << 1) | (1U << 0))

/*
 * ACCEL_CONFIG (Bank 2, 0x14):
 *   Bits[5:3] ACCEL_DLPFCFG = 011 → ~50.4 Hz 3-dB BW at 1.125 kHz ODR
 *   Bits[2:1] ACCEL_FS_SEL  = 01  → ±4 g  (8192 LSB/g)
 *   Bit[0]    ACCEL_FCHOICE = 1   → enable DLPF
 */
#define ICM_ACCEL_CONFIG_VAL    ((3U << 3) | (1U << 1) | (1U << 0))

/* ---- AK09916 register addresses (direct I2C at 0x0C) ------------------ */
#define AK_ST1                  0x10U   /* status 1: bit 0 = DRDY           */
#define AK_CNTL2                0x31U   /* operating mode                   */
#define AK_CNTL3                0x32U   /* soft reset                       */

/*
 * AK_CNTL2 MODE[4:0]:
 *   0x04 → Continuous Measurement Mode 2, 20 Hz ODR.
 */
#define AK_MODE_CONT_20HZ       0x04U

/*
 * AK_CNTL3:
 *   Bit 0 SRST = 1 → soft reset all registers; self-clears.
 */
#define AK_CNTL3_SRST           0x01U

/* ---- Data structure ---------------------------------------------------- */

/**
 * ICM20948_Vec3 — raw 16-bit X/Y/Z sample from one sensor.
 *
 * Conversion to engineering units (integer arithmetic):
 *   Accel (mg):      (int32_t)v.x * 1000 / 8192
 *   Gyro  (0.1 dps): (int32_t)v.x * 10   / 65
 *   Mag   (0.1 µT):  (int32_t)v.x * 15   / 10
 */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} ICM20948_Vec3;

/* ---- Public API -------------------------------------------------------- */

/**
 * ICM20948_Init()
 *   Configure I2C1 on PB6/PB7 at 400 kHz, then initialise both devices
 *   on the shared bus:
 *
 *   ICM-20948:
 *     - Verify WHO_AM_I = 0xEA
 *     - DEVICE_RESET, wake from sleep, CLKSEL = auto-PLL
 *     - Enable all accel + gyro axes
 *     - Gyro:  ±500 dps, 51 Hz DLPF
 *     - Accel: ±4 g,     50 Hz DLPF
 *
 *   AK09916 (via direct I2C at 0x0C — ACL/ADA pins wired to PB6/PB7):
 *     - Soft reset
 *     - Set continuous measurement mode 2 (20 Hz)
 *
 *   Returns 0 on success, 1 if ICM-20948 WHO_AM_I fails.
 *   Call once after SystemClock_Config().
 */
int ICM20948_Init(void);

/**
 * ICM20948_ReadAccel()
 *   6-byte burst read from ACCEL_XOUT_H (0x2D).
 *   Fills *a with raw signed 16-bit values.
 *   Blocking (~150 µs at 400 kHz).
 */
void ICM20948_ReadAccel(ICM20948_Vec3 *a);

/**
 * ICM20948_ReadGyro()
 *   6-byte burst read from GYRO_XOUT_H (0x33).
 *   Fills *g with raw signed 16-bit values.
 *   Blocking (~150 µs at 400 kHz).
 */
void ICM20948_ReadGyro(ICM20948_Vec3 *g);

/**
 * ICM20948_ReadTemp()
 *   2-byte burst read from TEMP_OUT_H (0x39).
 *   Stores the raw value in *t.
 *   Conversion: T_tenths_C = ((int32_t)(*t) * 10) / 334 + 210
 *   Blocking (~50 µs at 400 kHz).
 */
void ICM20948_ReadTemp(int16_t *t);

/**
 * ICM20948_ReadMag()
 *   9-byte burst read from AK09916 starting at ST1 (0x10), covering:
 *     ST1, HXL, HXH, HYL, HYH, HZL, HZH, TMPS, ST2.
 *   Reading ST2 is required by the AK09916 to release the next measurement.
 *   AK09916 data is little-endian; the driver corrects this before filling *m.
 *   Returns 1 if ST2 indicates magnetic overflow (HOFL bit 3), else 0.
 *   Blocking (~225 µs at 400 kHz).
 *
 *   Data refreshes at 20 Hz; calling faster returns the previous sample.
 */
int ICM20948_ReadMag(ICM20948_Vec3 *m);

#endif /* ICM20948_H */
