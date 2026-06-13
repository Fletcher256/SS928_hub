/**
 * @file    bmi270_driver.c
 * @brief   BMI270 IMU driver — implementation
 * @note    Adapter for Bosch BMI270 SensorAPI onto STM32 MyI2C HAL
 *
 * This driver implements the minimum subset of Bosch's bmi270.c needed
 * for raw accel+gyro data acquisition:
 *
 *   1. I2C read/write/delay callbacks (maps to _MyI2C_.c)
 *   2. Configuration file upload (INIT_CTRL + INIT_DATA burst write)
 *   3. Sensor configuration (ODR, range, bandwidth, filter perf)
 *   4. Sensor enable/disable
 *   5. Burst data read and parsing
 *   6. Madgwick AHRS + complementary filter (ported from MPU6050.c)
 *   7. Gyro zero-offset calibration (ported from MPU6050.c)
 *
 * Total Flash footprint: ~8KB (config) + ~3KB (code) ≈ 11KB
 */

#include "bmi270_driver.h"
#include "bmi270_defs.h"
#include "bmi270_config.h"
#include "../_MyI2C_.h"
#include "../USART.h"
#include <math.h>
#include <stdlib.h>
#include "../generic.h"
#if BMI270_USE_Filter
#include "../filter.h"
#endif

/*===========================================================================*/
/*! @name     I2C Bus Object & Convenience Macros                             */
/*===========================================================================*/
static i2cbus_struct bmi270_i2cbus;

#define BMI270_DELAY_MS(ms)                      Delay_ms(ms)
#define BMI270_DELAY_US(us)                      Delay_us(us)
#define BMI270_WRITE_REG(reg, data)              MYI2C_Write_Reg(&bmi270_i2cbus, reg, data)
#define BMI270_READ_REG(reg)                     MYI2C_Read_Reg(&bmi270_i2cbus, reg)
#define BMI270_READ_REG_CONTINUE(reg, len, buf) \
    MYI2C_Read_Reg_Continue(&bmi270_i2cbus, reg, len, buf)
#define BMI270_READ_REG_CONTINUE_STATUS(reg, len, buf) \
    MYI2C_Read_Reg_Continue_Status(&bmi270_i2cbus, reg, len, buf)

/* I2C address (SDO=GND → 0x68, SDO=VDDIO → 0x69) */
#define BMI270_I2C_ADDR  0x69

/*===========================================================================*/
/*! @name     Bosch SensorAPI Callback Layer                                  */
/*===========================================================================*/

/*!
 * @brief I2C write callback — maps Bosch API to MyI2C
 * @note  Bosch API writes: reg_addr (1 byte) → data[0..len-1]
 *        MyI2C expects a combined "register address + data" write.
 *        For len==1 we use the single-register write; for len>1
 *        we write reg_addr first then burst the data bytes individually
 *        since MyI2C doesn't have a true multi-byte write with auto-increment.
 */
static BMI2_INTF_RETURN_TYPE bmi2_i2c_write(uint8_t reg_addr,
                                             const uint8_t *reg_data,
                                             uint32_t len,
                                             void *intf_ptr)
{
    i2cbus_struct *bus = (i2cbus_struct *)intf_ptr;
    (void)bus; /* bus already configured in bmi270_i2cbus */

    if (len == 1) {
        /* Single register write – use existing helper */
        MYI2C_Write_Reg(&bmi270_i2cbus, reg_addr, (uint16_t)reg_data[0]);
    } else {
        /* Multi-byte write with auto-increment.
         * BMI270 supports auto-increment for most registers.
         * We implement this via repeated start: write reg_addr,
         * then write each data byte. The BMI270 auto-increments. */
        uint32_t i;
        for (i = 0; i < len; i++) {
            MYI2C_Write_Reg(&bmi270_i2cbus, (uint8_t)(reg_addr + i), (uint16_t)reg_data[i]);
        }
    }
    return BMI2_OK;
}

/*!
 * @brief I2C read callback — maps Bosch API to MyI2C
 */
static BMI2_INTF_RETURN_TYPE bmi2_i2c_read(uint8_t reg_addr,
                                            uint8_t *reg_data,
                                            uint32_t len,
                                            void *intf_ptr)
{
    i2cbus_struct *bus = (i2cbus_struct *)intf_ptr;
    (void)bus;

    if (len == 1) {
        uint16_t val = MYI2C_Read_Reg(&bmi270_i2cbus, reg_addr);
        reg_data[0] = (uint8_t)(val & 0xFF);
    } else {
        MYI2C_Read_Reg_Continue(&bmi270_i2cbus, reg_addr, (uint16_t)len, reg_data);
    }
    return BMI2_OK;
}

/*!
 * @brief Delay callback — maps Bosch API to our microsecond delay
 */
static void bmi2_delay_us_cb(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    Delay_us(period);
}

/*===========================================================================*/
/*! @name     Internal: BMI270 Register-Level Operations                      */
/*===========================================================================*/

/*!
 * @brief Soft-reset the BMI270
 */
static int8_t bmi2_soft_reset(void)
{
    BMI270_WRITE_REG(BMI2_CMD_REG_ADDR, BMI2_SOFT_RESET_CMD);
    BMI270_DELAY_MS(10);
    return BMI2_OK;
}

/*!
 * @brief Upload configuration file to BMI270
 *
 * This is the core initialization step. The config blob is written
 * to INIT_DATA (0x5E) in 1-byte writes. After writing, we poll
 * INTERNAL_STATUS (0x21) to verify successful load.
 *
 * @return BMI2_OK on success, BMI2_E_CONFIG_LOAD on failure
 */
static int8_t bmi2_upload_config(void)
{
    uint8_t internal_status;
    uint16_t index;
    uint8_t load_attempts;
    uint8_t addr_buf[2];

    /* Step 0: Read current state for diag */
    internal_status = (uint8_t)(BMI270_READ_REG(BMI2_INTERNAL_STATUS_ADDR) & 0xFF);
    USART3_printf("[BMI270] Pre-upload STATUS: 0x%02X\r\n", internal_status);

    /* Step 1: Check if config is already loaded */
    if (internal_status == BMI2_INIT_OK) {
        return BMI2_OK;
    }

    /* Step 2: Disable advanced power save (required before config upload) */
    BMI270_WRITE_REG(BMI2_PWR_CONF_ADDR, 0x00);
    BMI270_DELAY_US(450);

    /* Step 3: Switch to page 0 */
    BMI270_WRITE_REG(BMI2_FEAT_PAGE_ADDR, BMI2_PAGE_0);

    /* Step 4: DISABLE config load before writing data
     *         (official API: set_config_load(DISABLE) first) */
    BMI270_WRITE_REG(BMI2_INIT_CTRL_ADDR, 0x00);
    USART3_printf("[BMI270] INIT_CTRL disabled before upload\r\n");

    /* Step 5: Upload config file using INIT_ADDR_0/1 + INIT_DATA burst.
     *         The INIT_DATA (0x5E) register is a trap address — it does NOT
     *         auto-increment by itself. We must set INIT_ADDR_0 (0x5B) and
     *         INIT_ADDR_1 (0x5C) to select the target word address first,
     *         then write 2 bytes (one word) to INIT_DATA.
     *         After each word write, the internal address auto-increments. */
    USART3_printf("[BMI270] Writing %u bytes via INIT_ADDR_0/1...\r\n", BMI270_CONFIG_SIZE);
    for (index = 0; index < BMI270_CONFIG_SIZE; index += 2) {
        uint16_t word_addr = index / 2;

        /* Set word address: INIT_ADDR_0 = bits[3:0], INIT_ADDR_1 = bits[11:4] */
        addr_buf[0] = (uint8_t)(word_addr & 0x000F);
        addr_buf[1] = (uint8_t)((word_addr >> 4) & 0xFF);

        BMI270_WRITE_REG(BMI2_INIT_ADDR_0, addr_buf[0]);
        BMI270_WRITE_REG(BMI2_INIT_ADDR_1, addr_buf[1]);

        /* Write 2 bytes (one word) to INIT_DATA */
        if ((index + 1) < BMI270_CONFIG_SIZE) {
            /* Full word: write both bytes (burst auto-increments by 1 word) */
            uint8_t word_buf[2];
            word_buf[0] = bmi270_config_file[index];
            word_buf[1] = bmi270_config_file[index + 1];
            MYI2C_Write_Reg_Continue(&bmi270_i2cbus, BMI2_INIT_DATA_ADDR, word_buf, 2);
        } else {
            /* Last odd byte (should not happen, config is even length) */
            BMI270_WRITE_REG(BMI2_INIT_DATA_ADDR, bmi270_config_file[index]);
        }
    }
    USART3_printf("[BMI270] Write complete, enabling config load...\r\n");

    /* Step 6: ENABLE config load — this triggers the firmware to process data */
    BMI270_WRITE_REG(BMI2_INIT_CTRL_ADDR, BMI2_CONF_LOAD_EN_MASK);
    {
        uint8_t ctrl = (uint8_t)(BMI270_READ_REG(BMI2_INIT_CTRL_ADDR) & 0xFF);
        USART3_printf("[BMI270] INIT_CTRL after enable: 0x%02X\r\n", ctrl);
    }

    /* Step 7: Re-enable advanced power save */
    BMI270_WRITE_REG(BMI2_PWR_CONF_ADDR, 0x01);

    /* Step 8: Poll INTERNAL_STATUS for init complete (0x01 = BMI2_INIT_OK) */
    {
        uint8_t prev_status = 0xFF;
        for (load_attempts = 0; load_attempts < 50; load_attempts++) {
            BMI270_DELAY_MS(20);
            internal_status = (uint8_t)(BMI270_READ_REG(BMI2_INTERNAL_STATUS_ADDR) & 0xFF);

            /* Mask to lower nibble for status code comparison */
            if ((internal_status & 0x0F) == BMI2_INIT_OK) {
                USART3_printf("[BMI270] Init OK at attempt %d\r\n", load_attempts);
                break;
            }
            /* Log status changes */
            if (internal_status != prev_status) {
                USART3_printf("[BMI270] STATUS change: 0x%02X at attempt %d\r\n",
                              internal_status, load_attempts);
                prev_status = internal_status;
            }
        }
    }

    if ((internal_status & 0x0F) != BMI2_INIT_OK) {
        USART3_printf("[BMI270] FAIL: STATUS=0x%02X (need 0x01), attempts=%d\r\n",
                      internal_status, load_attempts);
        return BMI2_E_CONFIG_LOAD;
    }

    return BMI2_OK;
}

/*!
 * @brief Configure accelerometer
 *
 * BMI270 ACC_CONF (0x40) layout:
 *   [7]   : acc_filter_perf_mode  (0=ULP, 1=HP)
 *   [6:4] : acc_bwp
 *   [3:0] : acc_odr
 *
 * BMI270 ACC_RANGE (0x41) layout:
 *   [1:0] : acc_range
 */
static int8_t bmi2_set_accel_config(uint8_t odr, uint8_t range, uint8_t bwp, uint8_t filter_perf)
{
    uint8_t acc_conf;
    uint8_t acc_range_reg;

    /* Build ACC_CONF */
    acc_conf = (odr & BMI2_ACC_ODR_MASK);
    acc_conf |= ((bwp << BMI2_ACC_BWP_POS) & BMI2_ACC_BWP_MASK);
    acc_conf |= ((filter_perf << BMI2_ACC_FILTER_PERF_MODE_POS) & BMI2_ACC_FILTER_PERF_MODE_MASK);

    BMI270_WRITE_REG(BMI2_ACC_CONF_ADDR, acc_conf);

    /* ACC_RANGE is at 0x41 */
    acc_range_reg = (range & BMI2_ACC_RANGE_MASK);
    BMI270_WRITE_REG(0x41, acc_range_reg);

    return BMI2_OK;
}

/*!
 * @brief Configure gyroscope
 *
 * BMI270 GYR_CONF (0x42) layout:
 *   [7]   : gyr_filter_perf_mode
 *   [6]   : gyr_noise_perf_mode
 *   [5:4] : gyr_bwp
 *   [3:0] : gyr_odr
 *
 * BMI270 GYR_RANGE (0x43) layout:
 *   [2:0] : gyr_range
 */
static int8_t bmi2_set_gyro_config(uint8_t odr, uint8_t range, uint8_t bwp,
                                    uint8_t noise_perf, uint8_t filter_perf)
{
    uint8_t gyr_conf;
    uint8_t gyr_range_reg;

    /* Build GYR_CONF */
    gyr_conf = (odr & BMI2_GYR_ODR_MASK);
    gyr_conf |= ((bwp << BMI2_GYR_BWP_POS) & BMI2_GYR_BWP_MASK);
    gyr_conf |= ((noise_perf << BMI2_GYR_NOISE_PERF_MODE_POS) & BMI2_GYR_NOISE_PERF_MODE_MASK);
    gyr_conf |= ((filter_perf << BMI2_GYR_FILTER_PERF_MODE_POS) & BMI2_GYR_FILTER_PERF_MODE_MASK);

    BMI270_WRITE_REG(BMI2_GYR_CONF_ADDR, gyr_conf);

    /* GYR_RANGE is at 0x43 */
    gyr_range_reg = (range & BMI2_GYR_RANGE_MASK);
    BMI270_WRITE_REG(0x43, gyr_range_reg);

    return BMI2_OK;
}

/*!
 * @brief Enable/disable accelerometer and gyroscope
 * @param acc_en  : 1 to enable accel, 0 to disable
 * @param gyr_en  : 1 to enable gyro, 0 to disable
 */
static void bmi2_set_power_ctrl(uint8_t acc_en, uint8_t gyr_en)
{
    uint8_t pwr_ctrl = 0;

    if (acc_en) pwr_ctrl |= BMI2_ACC_EN_MASK;
    if (gyr_en) pwr_ctrl |= BMI2_GYR_EN_MASK;

    BMI270_WRITE_REG(BMI2_PWR_CTRL_ADDR, pwr_ctrl);
    BMI270_DELAY_US(450); /* Wait for sensor start-up */
}

/*===========================================================================*/
/*! @name     Gyro Calibration & Filter State (ported from MPU6050.c)         */
/*===========================================================================*/

static float bmi270_dt;                    /*!< Sample time interval (seconds) */
static float bmi270_gyro_scale;           /*!< Gyro LSB to rad/s scale */
static float bmi270_accel_scale;          /*!< Accel LSB to g scale */

/* Calibration reference angles */
static float angle_yaw   = 0.0f;
static float angle_roll  = 0.0f;
static float angle_pitch = 0.0f;

/* Gyro zero offsets (software calibration) */
static int16_t gyro_zero_x = 0;
static int16_t gyro_zero_y = 0;
static int16_t gyro_zero_z = 0;

/*===========================================================================*/
/*! @name     Online Gyro Bias Estimation (方案B)                              */
/*!                                                                           */
/*! Problem:  MEMS gyro bias random-walks over time (Bias Instability).       */
/*!           Static calibration at t=0 cannot compensate t>0 drift.          */
/*!                                                                           */
/*! Solution: When device is stationary, gyro output = bias + noise.          */
/*!           Estimate bias continuously via EMA in the rate domain,          */
/*!           then correct the yaw integration BEFORE accumulation.           */
/*!                                                                           */
/*! Architecture:                                                             */
/*!   raw → SW_offset_sub → PT1_filter → Gz_rate → bias_correct → yaw_inc    */
/*!                                          ↓                                */
/*!                                     [stationary?]─yes→ EMA(bias)          */
/*===========================================================================*/

/* ── Bias estimate state ── */
static float    gyro_bias_z  = 0.0f;    /*!< EMA bias estimate for Z-axis (rad/s) */

/* ── Tunable parameters ── */
static float    gyro_bias_alpha = 0.001f; /*!< EMA coeff: τ=1/(α*f)=1/(0.001*200)=5s */

/* ── Stationary detector state ── */
static uint16_t stationary_cnt = 0;      /*!< Consecutive stationary frames */
static uint8_t  bmi270_vehicle_moving = 1; /*!< Speed gate: 1=moving, 0=stopped (default safe) */

#define STATIONARY_CONFIRM_CNT   100U     /*!< Confirm stationary after 0.5s (100*5ms) */
#define STATIONARY_GYRO_THRESH   3.0f     /*!< Total gyro mag below this → stationary (°/s) */

/*!
 * @brief Set vehicle moving state (called from app layer based on wheel speed)
 * @param moving : 0 = stopped (allow bias update), 1 = moving (freeze bias)
 * @note  Speed-gate prevents EMA from absorbing slow turns into bias estimate.
 *        When the vehicle is moving, even slow gyro signals are real rotation,
 *        not bias drift. Default: moving=1 (bias frozen) for safety.
 */
void BMI270_SetVehicleMoving(uint8_t moving)
{
    bmi270_vehicle_moving = moving;
}

#if BMI270_USE_Filter
PT1Filter_t pt1_filter_x, pt1_filter_y, pt1_filter_z;
PT1Filter_t pt1_filter_gx, pt1_filter_gy, pt1_filter_gz;
#endif

/*!
 * @brief Software gyro zero-offset calibration
 * @param calibration_samples : Number of samples to average (typical: 1000)
 * @return 0 on success, -1 on failure
 *
 * Two-phase calibration:
 *   Phase 1 — Accumulate N samples to compute static bias (gyro_zero_x/y/z)
 *   Phase 2 — Verify with N/5 samples (calibration applied); fail if ANY
 *             axis residual mean >= 2 LSB (= 0.122 °/s at ±2000 dps)
 */
static int8_t BMI270_SoftCalibrate_Z(uint16_t calibration_samples)
{
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    int16_t GyroX, GyroY, GyroZ;
    int16_t cand_zero_x, cand_zero_y, cand_zero_z;
    uint8_t  temp_buffer[6];
    uint16_t i;
    uint16_t verify_samples;

    /* ── Phase 1: Accumulate calibration samples ── */
    for (i = 0; i < calibration_samples; i++) {
        if (BMI270_READ_REG_CONTINUE_STATUS(BMI2_GYR_X_LSB_ADDR, 6, temp_buffer) != 0) {
            return -1;
        }
        GyroX = ((int16_t)(temp_buffer[1] << 8) | temp_buffer[0]);
        GyroY = ((int16_t)(temp_buffer[3] << 8) | temp_buffer[2]);
        GyroZ = ((int16_t)(temp_buffer[5] << 8) | temp_buffer[4]);

        gx_sum += GyroX;
        gy_sum += GyroY;
        gz_sum += GyroZ;
        BMI270_DELAY_MS((uint32_t)(bmi270_dt * 1000.0f));
    }

    /* Compute candidate offsets — do NOT apply yet */
    cand_zero_x = (int16_t)(gx_sum / (int32_t)calibration_samples);
    cand_zero_y = (int16_t)(gy_sum / (int32_t)calibration_samples);
    cand_zero_z = (int16_t)(gz_sum / (int32_t)calibration_samples);

    /* ── Phase 2: Verify with candidate offsets (fresh accumulation) ── */
    verify_samples = calibration_samples / 5;
    if (verify_samples < 50) verify_samples = 50;
    if (verify_samples > 200) verify_samples = 200;

    gx_sum = 0;
    gy_sum = 0;
    gz_sum = 0;

    for (i = 0; i < verify_samples; i++) {
        if (BMI270_READ_REG_CONTINUE_STATUS(BMI2_GYR_X_LSB_ADDR, 6, temp_buffer) != 0) {
            return -1;
        }
        GyroX = ((int16_t)(temp_buffer[1] << 8) | temp_buffer[0]) - cand_zero_x;
        GyroY = ((int16_t)(temp_buffer[3] << 8) | temp_buffer[2]) - cand_zero_y;
        GyroZ = ((int16_t)(temp_buffer[5] << 8) | temp_buffer[4]) - cand_zero_z;

        gx_sum += GyroX;
        gy_sum += GyroY;
        gz_sum += GyroZ;
        BMI270_DELAY_MS((uint32_t)(bmi270_dt * 1000.0f));
    }

    /* Fail if ANY axis residual mean >= 2 LSB */
    if (abs((int)(gx_sum / (int32_t)verify_samples)) >= 2 ||
        abs((int)(gy_sum / (int32_t)verify_samples)) >= 2 ||
        abs((int)(gz_sum / (int32_t)verify_samples)) >= 2) {
        return -1;  /* gyro_zero untouched — caller will retry */
    }

    /* Only commit AFTER verification passes */
    gyro_zero_x = cand_zero_x;
    gyro_zero_y = cand_zero_y;
    gyro_zero_z = cand_zero_z;
    return 0;
}

/*===========================================================================*/
/*! @name     Raw Sensor Data Acquisition                                    */
/*===========================================================================*/

/*!
 * @brief Read raw sensor data from BMI270 via 12-byte burst
 *
 * BMI270 data register layout (0x0C ~ 0x17):
 *   [0] 0x0C: AccX[7:0]
 *   [1] 0x0D: AccX[15:8]
 *   [2] 0x0E: AccY[7:0]
 *   [3] 0x0F: AccY[15:8]
 *   [4] 0x10: AccZ[7:0]
 *   [5] 0x11: AccZ[15:8]
 *   [6] 0x12: GyrX[7:0]
 *   [7] 0x13: GyrX[15:8]
 *   [8] 0x14: GyrY[7:0]
 *   [9] 0x15: GyrY[15:8]
 *   [10] 0x16: GyrZ[7:0]
 *   [11] 0x17: GyrZ[15:8]
 *
 * Note: Temperature is NOT in the burst stream (separate reg 0x22-0x23)
 */
static void BMI270_Get_Raw(BMI270 *this)
{
    static uint8_t temp_buffer[12];

    if (BMI270_READ_REG_CONTINUE_STATUS(BMI2_ACC_X_LSB_ADDR, 12, temp_buffer) != 0) {
        return;
    }

    /* Parse 12-byte burst — BMI270 LSB first, MSB second (little-endian) */
    this->AccX  = ((int16_t)(temp_buffer[1] << 8) | temp_buffer[0]);
    this->AccY  = ((int16_t)(temp_buffer[3] << 8) | temp_buffer[2]);
    this->AccZ  = ((int16_t)(temp_buffer[5] << 8) | temp_buffer[4]);
    this->GyroX = ((int16_t)(temp_buffer[7] << 8) | temp_buffer[6]);
    this->GyroY = ((int16_t)(temp_buffer[9] << 8) | temp_buffer[8]);
    this->GyroZ = ((int16_t)(temp_buffer[11] << 8) | temp_buffer[10]);

    /* Apply software gyro calibration */
    this->GyroX = this->GyroX - gyro_zero_x;
    this->GyroY = this->GyroY - gyro_zero_y;
    this->GyroZ = this->GyroZ - gyro_zero_z;

#if BMI270_USE_Filter
    /* Apply PT1 filters */
    this->AccX  = (int16_t)PT1Filter_Apply(&pt1_filter_x, (float)this->AccX);
    this->AccY  = (int16_t)PT1Filter_Apply(&pt1_filter_y, (float)this->AccY);
    this->AccZ  = (int16_t)PT1Filter_Apply(&pt1_filter_z, (float)this->AccZ);
    this->GyroX = (int16_t)PT1Filter_Apply(&pt1_filter_gx, (float)this->GyroX);
    this->GyroY = (int16_t)PT1Filter_Apply(&pt1_filter_gy, (float)this->GyroY);
    this->GyroZ = (int16_t)PT1Filter_Apply(&pt1_filter_gz, (float)this->GyroZ);
#endif
}

/*!
 * @brief Read raw temperature from BMI270
 * @note  Temperature register: 0x22 (LSB) + 0x23 (MSB)
 *        Formula: temp_celsius = (raw / 512.0) + 23.0
 */
static int16_t BMI270_Get_RawTemp(void)
{
    uint8_t buf[2];
    if (BMI270_READ_REG_CONTINUE_STATUS(BMI2_TEMPERATURE_0_ADDR, 2, buf) != 0) {
        return 0;
    }
    return ((int16_t)(buf[1] << 8) | buf[0]);
}

/*===========================================================================*/
/*! @name     Fast Inverse Square Root (Quake 3 algorithm)                    */
/*===========================================================================*/
static inline float invSqrt(float x)
{
    float halfx = 0.5f * x;
    union { float f; uint32_t u; } conv;
    conv.f = x;
    conv.u = 0x5f3759dfu - (conv.u >> 1);
    float y = conv.f;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/*===========================================================================*/
/*! @name     Public API — Initialization                                    */
/*===========================================================================*/

/**
 * @brief Initialize BMI270 sensor
 *
 * Sequence:
 *  1. Initialize SW-I2C bus on specified GPIO pins
 *  2. Soft-reset BMI270
 *  3. Verify chip ID (0x24)
 *  4. Upload 8192-byte configuration file
 *  5. Configure accel: 200Hz, ±4G, Normal AVG4, Performance mode
 *  6. Configure gyro:  200Hz, ±2000dps, Normal mode, Performance
 *  7. Enable accel + gyro
 *  8. Wait for sensor stabilization
 *  9. Calibrate gyro zero offset (3 attempts)
 *  10. Initialize PT1 filters
 *
 * @param GPIOx : GPIO port for SCL and SDA
 * @param SCl   : SCL pin
 * @param SDA   : SDA pin
 */
void BMI270_init(GPIO_TypeDef *GPIOx, uint16_t SCl, uint16_t SDA)
{
    uint8_t chip_id;
    int8_t  rslt;
    int     i;

    /* 1. Initialize SW-I2C */
    MyI2C_Init(&bmi270_i2cbus,
               GPIOx, SCl,
               GPIOx, SDA,
               BMI270_I2C_ADDR,
               5);  /* delay_time = 5 (standard mode ~100kHz) */

    /* 2. Soft reset */
    bmi2_soft_reset();
    BMI270_DELAY_MS(50);

    /* === DIAGNOSTIC STEP 1: Chip ID === */
    chip_id = (uint8_t)(BMI270_READ_REG(BMI2_CHIP_ID_ADDR) & 0xFF);
    USART3_printf("[BMI270] Chip ID: 0x%02X (expected 0x%02X)\r\n", chip_id, BMI270_CHIP_ID);
    if (chip_id != BMI270_CHIP_ID) {
        //因为SDO拉高/悬空了导致这个端口的电平时高电平,地址是0X69。。若SDO拉低是0X68
        USART3_printf("[BMI270] FAIL: chip not found or wrong ID!,Error ID: 0x%02X\r\n", chip_id);
        return;
    }
    USART3_printf("[BMI270] Chip ID OK\r\n");

    /* === DIAGNOSTIC STEP 2: Config upload === */
    USART3_printf("[BMI270] Uploading config (%u bytes)...\r\n", BMI270_CONFIG_SIZE);
    rslt = bmi2_upload_config();
    if (rslt != BMI2_OK) {
        USART3_printf("[BMI270] FAIL: config upload error %d\r\n", rslt);
        return;
    }
    USART3_printf("[BMI270] Config upload OK\r\n");

    /* 5. Configure accelerometer: 200Hz, ±4G, Normal AVG4, High Performance */
    bmi2_set_accel_config(BMI2_ACC_ODR_200HZ,
                           BMI2_ACC_RANGE_4G,
                           BMI2_ACC_NORMAL_AVG4,
                           BMI2_PERF_OPT_MODE);

    /* 6. Configure gyroscope: 200Hz, ±2000dps, Normal mode, Low Power noise, High Perf filter */
    bmi2_set_gyro_config(BMI2_GYR_ODR_200HZ,
                          BMI2_GYR_RANGE_2000,
                          BMI2_GYR_NORMAL_MODE,
                          BMI2_POWER_OPT_MODE,
                          BMI2_PERF_OPT_MODE);

    /* 7. Enable accel + gyro */
    bmi2_set_power_ctrl(1, 1);

    /* --- Compute scale factors (same formula as MPU6050) --- */
    /* BMI270 16-bit resolution: 2^15 = 32768 LSB per full-scale range */
    /* Accel: ±4G → 4G / 32768 = 1/8192 g/LSB (same as MPU6050 acc_4g) */
    bmi270_accel_scale = 1.0f / 8192.0f;

    /* Gyro: ±2000dps → 2000/32768 = 0.061035... dps/LSB
     *       To radians: * PI/180 = 0.00106526... rad/s/LSB
     *       MPU6050 used: 0.0174533 / 16.4 = 0.00106423...
     *       BMI270: (2000.0 / 32768.0) * (PI / 180.0) */
    bmi270_gyro_scale = (2000.0f / 32768.0f) * 0.0174533f;

    /* Sample rate = 200Hz → dt = 5ms */
    bmi270_dt = 0.005f;

    /* 8. Wait for sensor stabilization (MEMS settling + filter warm-up) */
    BMI270_DELAY_MS(500);

    /* === STEP 9: Disable HW FOC + Gyro calibration ===
     *
     * 9a. Explicitly disable BMI270 on-chip offset compensation.
     *     Soft-reset sets reg 0x77 to 0x00, but config blob may touch it.
     *     Write 0x00 to GYR_OFF_COMP_6 (0x77) for defense-in-depth.
     */
    {
        uint8_t foc_reg;
        BMI270_WRITE_REG(BMI2_GYR_OFF_COMP_6_ADDR, 0x00);
        foc_reg = (uint8_t)(BMI270_READ_REG(BMI2_GYR_OFF_COMP_6_ADDR) & 0xFF);
        USART3_printf("[BMI270] FOC disabled (reg 0x77=0x%02X)\r\n", foc_reg);
    }

    /* 9b. Software static calibration (200 samples ≈ 1s) */
    {
        for (i = 0; i < 3; i++) {
            if (BMI270_SoftCalibrate_Z(200) == 0) {
                break;
            }
        }
        if (i < 3) {
            USART3_printf("[BMI270] SW Gyro cal OK (attempt %d)\r\n", i + 1);
        } else {
            USART3_printf("[BMI270] WARN: SW gyro cal failed, using raw offsets\r\n");
        }
        USART3_printf("[BMI270] SW Gyro offsets: X=%d Y=%d Z=%d\r\n",
                      (int)gyro_zero_x, (int)gyro_zero_y, (int)gyro_zero_z);
    }

    /* 9c. Seed online bias estimator at zero.
     *     SW static offset already removes the constant component from raw data.
     *     EMA starts at 0 and converges upward to the residual time-varying bias. */
    gyro_bias_z = 0.0f;
    USART3_printf("[BMI270] Bias seed: Z=0.0 (SW offset handles static component)\r\n");

    /* 10. Initialize filters */
#if BMI270_USE_Filter
    PT1Filter_InitWithFreq(&pt1_filter_x,  48, 200);
    PT1Filter_InitWithFreq(&pt1_filter_y,  48, 200);
    PT1Filter_InitWithFreq(&pt1_filter_z,  48, 200);
    PT1Filter_InitWithFreq(&pt1_filter_gx, 120, 200);
    PT1Filter_InitWithFreq(&pt1_filter_gy, 120, 200);
    PT1Filter_InitWithFreq(&pt1_filter_gz, 120, 200);
#endif

    /* === DIAGNOSTIC STEP 4: Verify power + data path === */
    {
        uint8_t buf[12];
        uint8_t pwr_ctrl, status_reg, read_rslt;
        int16_t ax, ay, az, gx, gy, gz;

        /* 4a: Read back PWR_CTRL to verify sensors are enabled */
        pwr_ctrl = (uint8_t)(BMI270_READ_REG(BMI2_PWR_CTRL_ADDR) & 0xFF);
        USART3_printf("[BMI270] PWR_CTRL(0x7D)=0x%02X (expect 0x06: acc+gyr en)\r\n", pwr_ctrl);

        /* 4b: Read STATUS for data ready */
        status_reg = (uint8_t)(BMI270_READ_REG(BMI2_STATUS_ADDR) & 0xFF);
        USART3_printf("[BMI270] STATUS(0x03)=0x%02X (bit7=DRDY_ACC,bit6=DRDY_GYR)\r\n", status_reg);

        /* 4c: Try burst read with status check */
        read_rslt = BMI270_READ_REG_CONTINUE_STATUS(BMI2_ACC_X_LSB_ADDR, 12, buf);
        USART3_printf("[BMI270] Burst read result: %d (0=OK, 1=NACK)\r\n", read_rslt);

        /* 4d: Also try single-byte reads */
        {
            uint8_t a0 = (uint8_t)(BMI270_READ_REG(0x0C) & 0xFF);
            uint8_t a1 = (uint8_t)(BMI270_READ_REG(0x0D) & 0xFF);
            uint8_t a2 = (uint8_t)(BMI270_READ_REG(0x0E) & 0xFF);
            USART3_printf("[BMI270] Single reads: 0x0C=0x%02X 0x0D=0x%02X 0x0E=0x%02X\r\n",
                          a0, a1, a2);
        }

        ax = ((int16_t)(buf[1] << 8) | buf[0]);
        ay = ((int16_t)(buf[3] << 8) | buf[2]);
        az = ((int16_t)(buf[5] << 8) | buf[4]);
        gx = ((int16_t)(buf[7] << 8) | buf[6]);
        gy = ((int16_t)(buf[9] << 8) | buf[8]);
        gz = ((int16_t)(buf[11] << 8) | buf[10]);
        USART3_printf("[BMI270] Raw: AccX=%d AccY=%d AccZ=%d | GyrX=%d GyrY=%d GyrZ=%d\r\n",
                      (int)ax, (int)ay, (int)az, (int)gx, (int)gy, (int)gz);
    }

    USART3_printf("[BMI270] Init complete — all checks passed\r\n");
}

/*===========================================================================*/
/*! @name     Public API — Data Acquisition                                  */
/*===========================================================================*/

/**
 * @brief Read temperature from BMI270
 * @param this : Pointer to BMI270 struct
 * @return Temperature in degrees Celsius
 */
float BMI270_GetTemp(BMI270 *this)
{
    this->rawTemp = BMI270_Get_RawTemp();
    /* BMI270 formula: temp = (raw / 512.0) + 23.0 */
    this->temp = (float)this->rawTemp / 512.0f + 23.0f;
    return this->temp;
}

/**
 * @brief Get attitude angles — Complementary Filter + Kalman method
 * @param this : Pointer to BMI270 struct
 * @param dt   : Time delta in seconds (use 0 to auto-use internal dt)
 */
void BMI270_Get_AngleDt(BMI270 *this, float dt)
{
    float Ax, Ay, Az;
    float Gx, Gy, Gz;

    if (dt <= 0.0f) {
        dt = bmi270_dt;
    }

    BMI270_Get_Raw(this);

    /* Convert raw to physical units */
    Ax = (float)this->AccX * bmi270_accel_scale;
    Ay = (float)this->AccY * bmi270_accel_scale;
    Az = (float)this->AccZ * bmi270_accel_scale;

    /* Gyro rates (rad/s) — for stationary detection + bias estimation */
    float Gx_rate = (float)this->GyroX * bmi270_gyro_scale;
    float Gy_rate = (float)this->GyroY * bmi270_gyro_scale;
    float Gz_rate = (float)this->GyroZ * bmi270_gyro_scale;

    /* ── Online Bias Estimation (方案B) ──
     * Stationary → gyro reads only bias+noise → EMA-update bias estimate.
     * Moving     → hold bias, apply correction to prevent integration drift.
     */
    {
        float gx_abs = (Gx_rate >= 0.0f) ? Gx_rate : -Gx_rate;
        float gy_abs = (Gy_rate >= 0.0f) ? Gy_rate : -Gy_rate;
        float gz_abs = (Gz_rate >= 0.0f) ? Gz_rate : -Gz_rate;
        float gyro_mag_dps = (gx_abs + gy_abs + gz_abs) * 57.2958f;

        if (gyro_mag_dps < STATIONARY_GYRO_THRESH) {
            if (stationary_cnt < STATIONARY_CONFIRM_CNT) {
                stationary_cnt++;
            }
        } else {
            stationary_cnt = 0;
        }

        /* Update bias only when BOTH gyro-quiet AND vehicle-stopped.
         * Speed gate prevents slow-turn signal from being absorbed as bias. */
        if (stationary_cnt >= STATIONARY_CONFIRM_CNT && !bmi270_vehicle_moving) {
            /* EMA update: bias ← α·obs + (1-α)·bias,  τ ≈ 1/(α·f) = 5s */
            gyro_bias_z = gyro_bias_alpha * Gz_rate
                        + (1.0f - gyro_bias_alpha) * gyro_bias_z;
        }
        /* else: bias frozen at last estimate */
    }

    /* Angular increments (rad) — bias-corrected for yaw */
    Gx = Gx_rate * dt;
    Gy = Gy_rate * dt;
    Gz = (Gz_rate - gyro_bias_z) * dt;   /* ← bias correction applied HERE */

    /* Dynamic weight based on acceleration magnitude */
    float absAcc = sqrt(Ax * Ax + Ay * Ay + Az * Az);
    float weight;
    if (absAcc > 1.2f) {
        weight = 0.8f;
    } else {
        weight = 0.98f;
    }

    static float Gyroscope_roll  = 0.0f;
    static float Gyroscope_pitch = 0.0f;

    Gyroscope_roll  += Gy;
    Gyroscope_pitch += Gx;

    this->roll  = weight * atan2(Ay, Az) / 3.1415926f * 180.0f + (1.0f - weight) * Gyroscope_roll;
    this->pitch = -(weight * atan2(Ax, Az) / 3.1415926f * 180.0f + (1.0f - weight) * Gyroscope_pitch);
    this->yaw   += Gz * 57.2958f;
}

/**
 * @brief Get attitude angles — Complementary Filter (using internal dt)
 */
void BMI270_Get_Angle(BMI270 *this)
{
    BMI270_Get_AngleDt(this, bmi270_dt);
}

/**
 * @brief Get attitude angles — Madgwick AHRS + Adaptive Gains (no gimbal lock)
 *
 * This is a direct port of MPU6050_Get_Angle_Plus() with identical algorithm.
 * Uses quaternion-based Madgwick filter with:
 *   - Adaptive Kp/Ki gains (high during init, reduced during steady state)
 *   - Integral error correction for gyro bias
 *   - Yaw unwrapping for continuous angle output
 */
void BMI270_Get_Angle_Plus(BMI270 *this)
{
    static uint16_t times = 0;
    static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    static float Kp, Ki;
    static float integralX = 0.0f, integralY = 0.0f, integralZ = 0.0f;

    /* Read raw sensor data */
    BMI270_Get_Raw(this);

    /* Convert to physical units */
    float ax = (float)this->AccX * bmi270_accel_scale;
    float ay = (float)this->AccY * bmi270_accel_scale;
    float az = (float)this->AccZ * bmi270_accel_scale;
    float gx = (float)this->GyroX * bmi270_gyro_scale;
    float gy = (float)this->GyroY * bmi270_gyro_scale;
    float gz = (float)this->GyroZ * bmi270_gyro_scale;

    /* Acceleration magnitude for adaptive gains */
    float accMag = ax * ax + ay * ay + az * az;

    /* Adaptive gain scheduling */
    if (times < 400) {
        times++;
        Kp = 8.0f;
        Ki = 0.002f;
    } else {
        Kp = (accMag > 1.44f || accMag < 0.64f) ? 3.6f : 4.8f;
        Ki = (accMag > 1.44f || accMag < 0.64f) ? 0.001f : 0.0015f;
    }

    /* Attitude correction from accelerometer (when valid) */
    if (accMag > 0.01f) {
        float recipNorm = invSqrt(accMag);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        /* Estimated gravity direction from quaternion */
        float vx = 2.0f * (q1 * q3 - q0 * q2);
        float vy = 2.0f * (q0 * q1 + q2 * q3);
        float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        /* Error = cross product of measured and estimated gravity */
        float ex = ay * vz - az * vy;
        float ey = az * vx - ax * vz;
        float ez = ax * vy - ay * vx;

        /* Integral error correction */
        if (Ki > 0.0f) {
            integralX += ex * bmi270_dt;
            integralY += ey * bmi270_dt;
            integralZ += ez * bmi270_dt;
            gx += Ki * integralX;
            gy += Ki * integralY;
            gz += Ki * integralZ;
        }

        /* Proportional error correction */
        gx += Kp * ex;
        gy += Kp * ey;
        gz += Kp * ez;
    }

    /* Quaternion integration */
    float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qDot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    float qDot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    float qDot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    q0 += qDot0 * bmi270_dt;
    q1 += qDot1 * bmi270_dt;
    q2 += qDot2 * bmi270_dt;
    q3 += qDot3 * bmi270_dt;

    /* Quaternion normalization */
    float norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;

    this->q0 = q0;
    this->q1 = q1;
    this->q2 = q2;
    this->q3 = q3;

    /* Quaternion → Euler angles */
    this->roll  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 57.29578f;
    this->pitch = asinf(2.0f * (q0 * q2 - q3 * q1)) * 57.29578f;
    float current_yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 57.29578f;

    /* Yaw unwrapping for continuous angle output */
    static float unwrapped_yaw = 0.0f;
    static uint8_t first_run = 1;
    if (first_run) {
        unwrapped_yaw = current_yaw;
        first_run = 0;
    } else {
        float diff = current_yaw - unwrapped_yaw;
        if (diff > 180.0f) {
            unwrapped_yaw += diff - 360.0f;
        } else if (diff < -180.0f) {
            unwrapped_yaw += diff + 360.0f;
        } else {
            unwrapped_yaw = current_yaw;
        }
    }

    /* Apply calibration offsets */
    this->roll  -= angle_roll;
    this->pitch -= angle_pitch;
    this->yaw    = unwrapped_yaw - angle_yaw;
}

/**
 * @brief Zero the current angle reference
 * @param this : Pointer to BMI270 struct
 */
void BMI270_Set_Angle0(BMI270 *this)
{
    volatile uint32_t delay;
    for (delay = 0; delay < 1000000; delay++);
    angle_yaw   = this->yaw;
    angle_roll  = this->roll;
    angle_pitch = this->pitch;
}

/**
 * @brief Read BMI270 chip ID
 * @return 0x24 if BMI270 is present, 0xFF on communication error
 */
uint8_t BMI270_ID(void)
{
    uint16_t val = BMI270_READ_REG(BMI2_CHIP_ID_ADDR);
    return (uint8_t)(val & 0xFF);
}
