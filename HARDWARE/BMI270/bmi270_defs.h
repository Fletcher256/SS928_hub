/**
 * Copyright (c) 2025 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file       bmi270_defs.h
 * @brief      Streamlined BMI2 register & type definitions
 * @note       Extracted from Bosch bmi2_defs.h v2.113.0
 *             Only essential types for accel+gyro usage are retained.
 */

#ifndef BMI270_DEFS_H_
#define BMI270_DEFS_H_

/******************************************************************************/
/*! @name       Header includes                                               */
/******************************************************************************/
#include <stdint.h>

/******************************************************************************/
/*! @name       Common macros                                                 */
/******************************************************************************/
#ifndef NULL
#define NULL  ((void *)0)
#endif

#ifndef UINT8_C
#define INT8_C(x)    ((int8_t)(x))
#define UINT8_C(x)   ((uint8_t)(x))
#endif

#ifndef UINT16_C
#define INT16_C(x)   ((int16_t)(x))
#define UINT16_C(x)  ((uint16_t)(x))
#endif

#ifndef UINT32_C
#define INT32_C(x)   ((int32_t)(x))
#define UINT32_C(x)  ((uint32_t)(x))
#endif

#ifndef UINT64_C
#define INT64_C(x)   ((int64_t)(x))
#define UINT64_C(x)  ((uint64_t)(x))
#endif

#define BMI2_DISABLE  UINT8_C(0)
#define BMI2_ENABLE   UINT8_C(1)

/* Bit manipulation macros */
#define BMI2_SET_BITS(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MASK)) |          \
     ((data << bitname##_POS) & bitname##_MASK))

#define BMI2_GET_BITS(reg_data, bitname) \
    ((reg_data & (bitname##_MASK)) >>    \
     (bitname##_POS))

#define BMI2_GET_BITSLICE(regvar, bitname) \
    ((regvar & bitname##_MASK) >> bitname##_POS)

#define BMI2_SET_BITSLICE(regvar, bitname, val) \
    ((regvar & ~bitname##_MASK) |               \
     ((val << bitname##_POS) & bitname##_MASK))

#define BMI2_SET_BITS_POS_0(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MASK)) | (data & bitname##_MASK))

#define BMI2_GET_BITS_POS_0(reg_data, bitname)  (reg_data & (bitname##_MASK))

/******************************************************************************/
/*! @name        Return type and error codes                                  */
/******************************************************************************/
#ifndef BMI2_INTF_RETURN_TYPE
#define BMI2_INTF_RETURN_TYPE  int8_t
#endif

#define BMI2_OK                        INT8_C(0)
#define BMI2_E_NULL_PTR                INT8_C(-1)
#define BMI2_E_COM_FAIL                INT8_C(-2)
#define BMI2_E_DEV_NOT_FOUND           INT8_C(-3)
#define BMI2_E_OUT_OF_RANGE            INT8_C(-4)
#define BMI2_E_ACC_INVALID_CFG         INT8_C(-5)
#define BMI2_E_GYRO_INVALID_CFG        INT8_C(-6)
#define BMI2_E_ACC_GYR_INVALID_CFG     INT8_C(-7)
#define BMI2_E_INVALID_SENSOR          INT8_C(-8)
#define BMI2_E_CONFIG_LOAD             INT8_C(-9)
#define BMI2_E_INVALID_PAGE            INT8_C(-10)
#define BMI2_E_INVALID_INT_PIN         INT8_C(-12)
#define BMI2_E_SET_APS_FAIL            INT8_C(-13)
#define BMI2_E_AUX_INVALID_CFG         INT8_C(-14)
#define BMI2_E_AUX_BUSY                INT8_C(-15)
#define BMI2_E_SELF_TEST_FAIL          INT8_C(-16)
#define BMI2_E_REMAP_ERROR             INT8_C(-17)
#define BMI2_E_GYR_USER_GAIN_UPD_FAIL  INT8_C(-18)
#define BMI2_E_INVALID_INPUT           INT8_C(-20)
#define BMI2_E_INVALID_STATUS          INT8_C(-21)
#define BMI2_E_CRT_ERROR               INT8_C(-22)
#define BMI2_E_DL_ERROR                INT8_C(-25)
#define BMI2_E_PRECON_ERROR            INT8_C(-26)
#define BMI2_E_ABORT_ERROR             INT8_C(-27)
#define BMI2_E_WRITE_CYCLE_ONGOING     INT8_C(-30)
#define BMI2_E_ST_NOT_RUNING           INT8_C(-32)
#define BMI2_E_DATA_RDY_INT_FAILED     INT8_C(-33)
#define BMI2_E_INVALID_FOC_POSITION    INT8_C(-34)

/******************************************************************************/
/*! @name        Register Map                                                 */
/******************************************************************************/
#define BMI2_CHIP_ID_ADDR         UINT8_C(0x00)
#define BMI2_STATUS_ADDR          UINT8_C(0x03)
#define BMI2_ACC_X_LSB_ADDR       UINT8_C(0x0C)
#define BMI2_GYR_X_LSB_ADDR       UINT8_C(0x12)
#define BMI2_SENSORTIME_ADDR      UINT8_C(0x18)
#define BMI2_EVENT_ADDR           UINT8_C(0x1B)
#define BMI2_INT_STATUS_0_ADDR    UINT8_C(0x1C)
#define BMI2_INT_STATUS_1_ADDR    UINT8_C(0x1D)
#define BMI2_INTERNAL_STATUS_ADDR UINT8_C(0x21)
#define BMI2_TEMPERATURE_0_ADDR   UINT8_C(0x22)
#define BMI2_TEMPERATURE_1_ADDR   UINT8_C(0x23)
#define BMI2_FEAT_PAGE_ADDR       UINT8_C(0x2F)
#define BMI2_FEATURES_REG_ADDR    UINT8_C(0x30)
#define BMI2_ACC_CONF_ADDR        UINT8_C(0x40)
#define BMI2_GYR_CONF_ADDR        UINT8_C(0x42)
#define BMI2_FIFO_CONFIG_0_ADDR   UINT8_C(0x48)
#define BMI2_FIFO_CONFIG_1_ADDR   UINT8_C(0x49)
#define BMI2_SATURATION_ADDR      UINT8_C(0x4A)
#define BMI2_AUX_IF_CONF_ADDR     UINT8_C(0x4B)  /*!< AUX interface config (bit0=aux_en) */
#define BMI2_INT1_IO_CTRL_ADDR    UINT8_C(0x53)
#define BMI2_INT2_IO_CTRL_ADDR    UINT8_C(0x54)
#define BMI2_INT_LATCH_ADDR       UINT8_C(0x55)
#define BMI2_INT1_MAP_FEAT_ADDR   UINT8_C(0x56)
#define BMI2_INT2_MAP_FEAT_ADDR   UINT8_C(0x57)
#define BMI2_INT_MAP_DATA_ADDR    UINT8_C(0x58)
#define BMI2_INIT_CTRL_ADDR       UINT8_C(0x59)
#define BMI2_INIT_ADDR_0          UINT8_C(0x5B)
#define BMI2_INIT_ADDR_1          UINT8_C(0x5C)
#define BMI2_INIT_DATA_ADDR       UINT8_C(0x5E)
#define BMI2_INTERNAL_ERR_ADDR    UINT8_C(0x5F)
#define BMI2_IF_CONF_ADDR         UINT8_C(0x6B)
#define BMI2_NV_CONF_ADDR         UINT8_C(0x70)
#define BMI2_PWR_CONF_ADDR        UINT8_C(0x7C)
#define BMI2_PWR_CTRL_ADDR        UINT8_C(0x7D)
#define BMI2_CMD_REG_ADDR         UINT8_C(0x7E)

/*! @name I2C addresses */
#define BMI2_I2C_PRIM_ADDR        UINT8_C(0x68)
#define BMI2_I2C_SEC_ADDR         UINT8_C(0x69)

/*! @name Command register values */
#define BMI2_SOFT_RESET_CMD       UINT8_C(0xB6)
#define BMI2_FIFO_FLUSH_CMD       UINT8_C(0xB0)
#define BMI2_USR_GAIN_CMD         UINT8_C(0x03)

/*! @name Number of data bytes for accel+gyro burst read (6 axes * 2 bytes = 12) */
#define BMI2_ACC_GYR_NUM_BYTES    UINT8_C(12)

/*! @name Total data bytes including sensor time (12 + 3 sensor time + 1 status = 16) */
/* Actually: 0x03 status + 0x04..0x0B aux + 0x0C..0x17 acc+gyr + 0x18..0x1A time = 24 */
/* For our use: burst from 0x0C, 12 bytes covers AccX_L..GyrZ_H */
#define BMI2_ACC_GYR_AUX_SENSORTIME_NUM_BYTES  UINT8_C(24)

/******************************************************************************/
/*! @name        Status and Data Ready masks/positions                        */
/******************************************************************************/
#define BMI2_AUX_BUSY_MASK      UINT8_C(0x04)
#define BMI2_CMD_RDY_MASK       UINT8_C(0x10)
#define BMI2_DRDY_AUX_MASK      UINT8_C(0x20)
#define BMI2_DRDY_GYR_MASK      UINT8_C(0x40)
#define BMI2_DRDY_ACC_MASK      UINT8_C(0x80)

#define BMI2_AUX_BUSY_POS       UINT8_C(0x02)
#define BMI2_CMD_RDY_POS        UINT8_C(0x04)
#define BMI2_DRDY_AUX_POS       UINT8_C(0x05)
#define BMI2_DRDY_GYR_POS       UINT8_C(0x06)
#define BMI2_DRDY_ACC_POS       UINT8_C(0x07)

#define BMI2_DRDY_ACC           UINT8_C(0x80)
#define BMI2_DRDY_GYR           UINT8_C(0x40)

/*! @name Power control masks */
#define BMI2_AUX_EN_MASK        UINT8_C(0x01)
#define BMI2_GYR_EN_MASK        UINT8_C(0x02)
#define BMI2_ACC_EN_MASK        UINT8_C(0x04)
#define BMI2_TEMP_EN_MASK       UINT8_C(0x08)

#define BMI2_GYR_EN_POS         UINT8_C(0x01)
#define BMI2_ACC_EN_POS         UINT8_C(0x02)
#define BMI2_TEMP_EN_POS        UINT8_C(0x03)

/*! @name Init control mask */
#define BMI2_CONF_LOAD_EN_MASK  UINT8_C(0x01)  /*!< bit0: enable config file load */
#define BMI2_AUX_DISABLE_MASK   UINT8_C(0x02)  /*!< bit1: disable AUX init during config load */

/*! @name Config load status */
#define BMI2_CONFIG_LOAD_SUCCESS    UINT8_C(1)

/*! @name Internal status (register 0x21) */
#define BMI2_INIT_OK                UINT8_C(0x01)  /*!< bit0: initialization complete */
#define BMI2_INIT_ERR_AUX           UINT8_C(0x02)  /*!< bit1: AUX I2C init error */
#define BMI2_INIT_ERR_GYR           UINT8_C(0x04)  /*!< bit2: Gyro init error */
#define BMI2_INIT_ERR_ACC           UINT8_C(0x08)  /*!< bit3: Accel init error */

/*! @name Power config mask */
#define BMI2_ADV_POW_EN_MASK    UINT8_C(0x01)

/******************************************************************************/
/*! @name        Pages                                                        */
/******************************************************************************/
#define BMI2_PAGE_0  UINT8_C(0)
#define BMI2_PAGE_1  UINT8_C(1)

/******************************************************************************/
/*! @name        Sensor type identifiers                                      */
/******************************************************************************/
#define BMI2_ACCEL                     UINT8_C(0)
#define BMI2_GYRO                      UINT8_C(1)
#define BMI2_AUX                       UINT8_C(2)
#define BMI2_TEMP                      UINT8_C(32)

/* Sensor selection bitmasks */
#define BMI2_ACCEL_SENS_SEL           ((uint64_t)1)
#define BMI2_GYRO_SENS_SEL            ((uint64_t)1 << BMI2_GYRO)
#define BMI2_TEMP_SENS_SEL            ((uint64_t)1 << BMI2_TEMP)

/* Main sensors */
#define BMI2_MAIN_SENSORS \
    (BMI2_ACCEL_SENS_SEL | BMI2_GYRO_SENS_SEL | BMI2_AUX_SENS_SEL | BMI2_TEMP_SENS_SEL)
#define BMI2_MAIN_SENS_MAX_NUM        UINT8_C(4)

/* Feature sensors (for reference) */
#define BMI2_ANY_MOTION               UINT8_C(4)
#define BMI2_NO_MOTION                UINT8_C(5)
#define BMI2_STEP_DETECTOR            UINT8_C(6)
#define BMI2_STEP_COUNTER             UINT8_C(7)
#define BMI2_STEP_ACTIVITY            UINT8_C(8)
#define BMI2_GYRO_GAIN_UPDATE         UINT8_C(9)
#define BMI2_WRIST_GESTURE            UINT8_C(19)
#define BMI2_WRIST_WEAR_WAKE_UP       UINT8_C(20)
#define BMI2_GYRO_SELF_OFF            UINT8_C(34)

/******************************************************************************/
/*! @name        Accel configuration enums                                    */
/******************************************************************************/
/*! Output Data Rate */
#define BMI2_ACC_ODR_0_78HZ   UINT8_C(0x01)
#define BMI2_ACC_ODR_1_56HZ   UINT8_C(0x02)
#define BMI2_ACC_ODR_3_12HZ   UINT8_C(0x03)
#define BMI2_ACC_ODR_6_25HZ   UINT8_C(0x04)
#define BMI2_ACC_ODR_12_5HZ   UINT8_C(0x05)
#define BMI2_ACC_ODR_25HZ     UINT8_C(0x06)
#define BMI2_ACC_ODR_50HZ     UINT8_C(0x07)
#define BMI2_ACC_ODR_100HZ    UINT8_C(0x08)
#define BMI2_ACC_ODR_200HZ    UINT8_C(0x09)
#define BMI2_ACC_ODR_400HZ    UINT8_C(0x0A)
#define BMI2_ACC_ODR_800HZ    UINT8_C(0x0B)
#define BMI2_ACC_ODR_1600HZ   UINT8_C(0x0C)

/*! Accel Range */
#define BMI2_ACC_RANGE_2G     UINT8_C(0x00)
#define BMI2_ACC_RANGE_4G     UINT8_C(0x01)
#define BMI2_ACC_RANGE_8G     UINT8_C(0x02)
#define BMI2_ACC_RANGE_16G    UINT8_C(0x03)

/*! Accel Bandwidth */
#define BMI2_ACC_OSR4_AVG1    UINT8_C(0x00)
#define BMI2_ACC_OSR2_AVG2    UINT8_C(0x01)
#define BMI2_ACC_NORMAL_AVG4  UINT8_C(0x02)
#define BMI2_ACC_CIC_AVG8     UINT8_C(0x03)
#define BMI2_ACC_RES_AVG16    UINT8_C(0x04)
#define BMI2_ACC_RES_AVG32    UINT8_C(0x05)
#define BMI2_ACC_RES_AVG64    UINT8_C(0x06)
#define BMI2_ACC_RES_AVG128   UINT8_C(0x07)

/*! Accel config masks */
#define BMI2_ACC_RANGE_MASK   UINT8_C(0x03)
#define BMI2_ACC_ODR_MASK     UINT8_C(0x0F)
#define BMI2_ACC_BWP_MASK     UINT8_C(0x70)
#define BMI2_ACC_FILTER_PERF_MODE_MASK  UINT8_C(0x80)

/*! Accel config bit positions */
#define BMI2_ACC_ODR_POS      UINT8_C(0x00)
#define BMI2_ACC_BWP_POS      UINT8_C(0x04)
#define BMI2_ACC_FILTER_PERF_MODE_POS   UINT8_C(0x07)

/******************************************************************************/
/*! @name        Gyro configuration enums                                     */
/******************************************************************************/
/*! Gyro Output Data Rate */
#define BMI2_GYR_ODR_25HZ     UINT8_C(0x06)
#define BMI2_GYR_ODR_50HZ     UINT8_C(0x07)
#define BMI2_GYR_ODR_100HZ    UINT8_C(0x08)
#define BMI2_GYR_ODR_200HZ    UINT8_C(0x09)
#define BMI2_GYR_ODR_400HZ    UINT8_C(0x0A)
#define BMI2_GYR_ODR_800HZ    UINT8_C(0x0B)
#define BMI2_GYR_ODR_1600HZ   UINT8_C(0x0C)
#define BMI2_GYR_ODR_3200HZ   UINT8_C(0x0D)

/*! Gyro Range */
#define BMI2_GYR_RANGE_2000   UINT8_C(0x00)
#define BMI2_GYR_RANGE_1000   UINT8_C(0x01)
#define BMI2_GYR_RANGE_500    UINT8_C(0x02)
#define BMI2_GYR_RANGE_250    UINT8_C(0x03)
#define BMI2_GYR_RANGE_125    UINT8_C(0x04)

/*! Gyro Bandwidth */
#define BMI2_GYR_OSR4_MODE    UINT8_C(0x00)
#define BMI2_GYR_OSR2_MODE    UINT8_C(0x01)
#define BMI2_GYR_NORMAL_MODE  UINT8_C(0x02)
#define BMI2_GYR_CIC1_MODE    UINT8_C(0x03)
#define BMI2_GYR_CIC2_MODE    UINT8_C(0x04)
#define BMI2_GYR_CIC3_MODE    UINT8_C(0x05)

/*! Gyro config masks */
#define BMI2_GYR_RANGE_MASK             UINT8_C(0x07)
#define BMI2_GYR_ODR_MASK               UINT8_C(0x0F)
#define BMI2_GYR_BWP_MASK               UINT8_C(0x30)
#define BMI2_GYR_NOISE_PERF_MODE_MASK   UINT8_C(0x40)
#define BMI2_GYR_FILTER_PERF_MODE_MASK  UINT8_C(0x80)
#define BMI2_GYR_OIS_RANGE_MASK         UINT8_C(0x38)

/*! Gyro config bit positions */
#define BMI2_GYR_ODR_POS                UINT8_C(0x00)
#define BMI2_GYR_BWP_POS                UINT8_C(0x04)
#define BMI2_GYR_NOISE_PERF_MODE_POS    UINT8_C(0x06)
#define BMI2_GYR_FILTER_PERF_MODE_POS   UINT8_C(0x07)

/******************************************************************************/
/*! @name        Performance mode macros                                      */
/******************************************************************************/
#define BMI2_POWER_OPT_MODE   UINT8_C(0)
#define BMI2_PERF_OPT_MODE    UINT8_C(1)

/******************************************************************************/
/*! @name        Range conversion values                                      */
/******************************************************************************/
#define BMI2_ACC_RANGE_2G_VAL   (2.0f)
#define BMI2_ACC_RANGE_4G_VAL   (4.0f)
#define BMI2_ACC_RANGE_8G_VAL   (8.0f)
#define BMI2_ACC_RANGE_16G_VAL  (16.0f)

#define BMI2_GYR_RANGE_125_VAL   (125.0f)
#define BMI2_GYR_RANGE_250_VAL   (250.0f)
#define BMI2_GYR_RANGE_500_VAL   (500.0f)
#define BMI2_GYR_RANGE_1000_VAL  (1000.0f)
#define BMI2_GYR_RANGE_2000_VAL  (2000.0f)

/*! Gravity constant for LSB to m/s² conversion */
#define GRAVITY_EARTH  (9.80665f)

/******************************************************************************/
/*! @name        BMI270 Chip ID                                               */
/******************************************************************************/
#define BMI270_CHIP_ID    UINT8_C(0x24)

/******************************************************************************/
/*! @name        Interrupt pin selection                                      */
/******************************************************************************/
enum bmi2_hw_int_pin {
    BMI2_INT_NONE = 0,
    BMI2_INT1,
    BMI2_INT2,
    BMI2_INT_BOTH,
    BMI2_INT_PIN_MAX
};

/*! Data interrupt types */
#define BMI2_FFULL_INT    UINT8_C(0x01)
#define BMI2_FWM_INT      UINT8_C(0x02)
#define BMI2_DRDY_INT     UINT8_C(0x04)
#define BMI2_ERR_INT      UINT8_C(0x08)

/******************************************************************************/
/*! @name        Interface enum                                               */
/******************************************************************************/
enum bmi2_intf {
    BMI2_SPI_INTF = 0,
    BMI2_I2C_INTF,
    BMI2_I3C_INTF
};

/******************************************************************************/
/*! @name        Sensor configuration error enum                              */
/******************************************************************************/
enum bmi2_sensor_config_error {
    BMI2_NO_ERROR,
    BMI2_ACC_ERROR,
    BMI2_GYR_ERROR,
    BMI2_ACC_GYR_ERROR
};

/******************************************************************************/
/*! @name        Function Pointer Types                                       */
/******************************************************************************/

/*!
 * @brief Bus read function pointer
 * @param[in]  reg_addr   Register address from which data is read
 * @param[out] reg_data   Pointer to data buffer where read data is stored
 * @param[in]  len        Number of bytes of data to be read
 * @param[in]  intf_ptr   Interface pointer (can be our &i2c_bus)
 * @retval BMI2_INTF_RET_SUCCESS on success
 */
typedef BMI2_INTF_RETURN_TYPE (*bmi2_read_fptr_t)(uint8_t reg_addr, uint8_t *reg_data,
                                                   uint32_t len, void *intf_ptr);

/*!
 * @brief Bus write function pointer
 * @param[in] reg_addr    Register address to which the data is written
 * @param[in] reg_data    Pointer to data buffer in which data to be written is stored
 * @param[in] len         Number of bytes of data to be written
 * @param[in] intf_ptr    Interface pointer
 */
typedef BMI2_INTF_RETURN_TYPE (*bmi2_write_fptr_t)(uint8_t reg_addr, const uint8_t *reg_data,
                                                    uint32_t len, void *intf_ptr);

/*!
 * @brief Delay function pointer (microseconds)
 * @param[in] period      Delay in microseconds
 * @param[in] intf_ptr    Interface pointer
 */
typedef void (*bmi2_delay_fptr_t)(uint32_t period, void *intf_ptr);

/******************************************************************************/
/*! @name        Struct Definitions                                           */
/******************************************************************************/

/*! @brief Accelerometer and gyroscope sensor axes data */
struct bmi2_sens_axes_data
{
    int16_t x;  /*!< Data in x-axis */
    int16_t y;  /*!< Data in y-axis */
    int16_t z;  /*!< Data in z-axis */
    uint32_t virt_sens_time;  /*!< Sensor time for virtual frames */
};

/*! @brief BMI2 sensor data (accelerometer + gyroscope) */
struct bmi2_sens_data
{
    struct bmi2_sens_axes_data acc;  /*!< Accelerometer axes data */
    struct bmi2_sens_axes_data gyr;  /*!< Gyroscope axes data */
    uint8_t  aux_data[16];           /*!< Auxiliary sensor data */
    uint32_t sens_time;              /*!< Sensor time */
    uint8_t  status;                 /*!< Status register data */
};

/*! @brief Accelerometer configuration */
struct bmi2_accel_config
{
    uint8_t odr;           /*!< Output data rate in Hz */
    uint8_t bwp;           /*!< Bandwidth parameter */
    uint8_t filter_perf;   /*!< Filter performance mode */
    uint8_t range;         /*!< g-range */
};

/*! @brief Gyroscope configuration */
struct bmi2_gyro_config
{
    uint8_t odr;           /*!< Output data rate in Hz */
    uint8_t bwp;           /*!< Bandwidth parameter */
    uint8_t filter_perf;   /*!< Filter performance mode */
    uint8_t ois_range;     /*!< OIS Range */
    uint8_t range;         /*!< Gyroscope Range */
    uint8_t noise_perf;    /*!< Selects noise performance */
};

/*! @brief Union of all possible sensor configuration types */
union bmi2_sens_config_types
{
    struct bmi2_accel_config acc;   /*!< Accelerometer configuration */
    struct bmi2_gyro_config   gyr;  /*!< Gyroscope configuration */
};

/*! @brief Type of sensor and its configurations */
struct bmi2_sens_config
{
    uint8_t type;                      /*!< Defines the type of sensor */
    union bmi2_sens_config_types cfg;  /*!< Defines various sensor configurations */
};

/*! @brief Feature configuration lookup structure */
struct bmi2_feature_config
{
    uint8_t type;        /*!< Defines the type of sensor */
    uint8_t page;        /*!< Page to where the feature is mapped */
    uint8_t start_addr;  /*!< Address of the feature */
};

/*! @brief Feature interrupt configurations */
struct bmi2_map_int
{
    uint8_t type;         /*!< Defines the type of sensor */
    uint8_t sens_map_int; /*!< Defines the feature interrupt */
};

/*! @brief Axes re-mapping */
struct bmi2_axes_remap
{
    uint8_t x_axis;       /*!< X axis re-map */
    uint8_t x_axis_sign;  /*!< X axis sign */
    uint8_t y_axis;       /*!< Y axis re-map */
    uint8_t y_axis_sign;  /*!< Y axis sign */
    uint8_t z_axis;       /*!< Z axis re-map */
    uint8_t z_axis_sign;  /*!< Z axis sign */
};

/*! @brief BMI2 device structure */
struct bmi2_dev
{
    uint8_t  chip_id;          /*!< Chip id of BMI2 */
    void    *intf_ptr;         /*!< Interface pointer (e.g. &i2c_bus) */
    uint8_t  info;             /*!< To store warnings */
    enum bmi2_intf intf;       /*!< Type of Interface */
    uint8_t  resolution;       /*!< Resolution for FOC */
    uint16_t read_write_len;   /*!< User set read/write length */
    uint8_t  feature_len;      /*!< Feature len */
    uint8_t  load_status;      /*!< Store load status value */

    /*! Pointer to the configuration data buffer address */
    const uint8_t *config_file_ptr;

    uint8_t  page_max;         /*!< Maximum page number */
    uint8_t  input_sens;       /*!< Maximum number of input sensors/features */
    uint8_t  out_sens;         /*!< Maximum number of output sensors/features */

    /*! Array of feature input configuration structure */
    const struct bmi2_feature_config *feat_config;

    /*! Array of feature output configuration structure */
    const struct bmi2_feature_config *feat_output;

    /*! Structure to maintain a copy of the re-mapped axis */
    struct bmi2_axes_remap remap;

    /*! Flag to hold enable status of sensors */
    uint64_t sens_en_stat;

    /*! Read function pointer */
    bmi2_read_fptr_t read;

    /*! Write function pointer */
    bmi2_write_fptr_t write;

    /*! Delay function pointer */
    bmi2_delay_fptr_t delay_us;

    /*! Gyroscope cross sensitivity value */
    int16_t gyr_cross_sens_zx;

    uint8_t  gyro_en : 1;       /*!< Gyro enable status flag */
    uint8_t  aps_status;        /*!< Advance power saving mode status */
    uint16_t variant_feature;   /*!< Variant specific features flag */
    uint16_t config_size;       /*!< Size of config file */

    /*! Array of feature interrupts configuration structure */
    struct bmi2_map_int *map_int;

    uint8_t  sens_int_map;      /*!< Maximum number of interrupts */
    uint8_t  dummy_byte;        /*!< For switching from I2C to SPI */
};

#endif /* BMI270_DEFS_H_ */
