/**
 * @file    bmi270_driver.h
 * @brief   BMI270 IMU driver — public API
 * @note    Drop-in replacement for MPU6050.h
 *
 * This driver wraps Bosch's BMI270 SensorAPI initialization flow and
 * provides the same interface as the existing MPU6050 driver (structure
 * layout, function signatures), so the application layer needs minimal
 * changes to switch from MPU6050 to BMI270.
 *
 * Key differences from MPU6050:
 *  - BMI270 requires an 8192-byte config blob upload on every power-up
 *  - Data register start is 0x0C (vs 0x3B for MPU6050)
 *  - 12-byte burst read (no temp in stream) vs 14-byte for MPU6050
 *  - Temperature read from separate register 0x22-0x23
 */

#ifndef BMI270_DRIVER_H_
#define BMI270_DRIVER_H_

#include "stm32f10x.h"

/*===========================================================================*/
/*! @name Filter enable flag (matches MPU6050 convention) */
#define BMI270_USE_Filter  1

/*===========================================================================*/
/*! @name Sensor data structure (layout compatible with MPU6050 struct) */
typedef struct BMI270 {
    /* Raw data */
    int16_t AccX;      /*!< X-axis accelerometer raw data */
    int16_t AccY;      /*!< Y-axis accelerometer raw data */
    int16_t AccZ;      /*!< Z-axis accelerometer raw data */
    int16_t GyroX;     /*!< X-axis gyroscope raw data */
    int16_t GyroY;     /*!< Y-axis gyroscope raw data */
    int16_t GyroZ;     /*!< Z-axis gyroscope raw data */
    int16_t rawTemp;   /*!< Temperature raw data */

    /* Euler angles (degrees) */
    float yaw;         /*!< Yaw angle */
    float roll;        /*!< Roll angle */
    float pitch;       /*!< Pitch angle */
    float temp;        /*!< Actual temperature (Celsius) */

    /* Quaternion attitude */
    float q0;           /*!< Quaternion w */
    float q1;           /*!< Quaternion x */
    float q2;           /*!< Quaternion y */
    float q3;           /*!< Quaternion z */
} BMI270;

/*===========================================================================*/
/*! @name     Core API (matches MPU6050.h interface)                          */
/*===========================================================================*/

/*!
 * @brief  Initialize BMI270 sensor
 * @param  GPIOx : GPIO port for SCL/SDA (must be same port)
 * @param  SCl   : SCL pin
 * @param  SDA   : SDA pin
 * @note   Example: BMI270_init(GPIOB, GPIO_Pin_0, GPIO_Pin_1);
 */
void BMI270_init(GPIO_TypeDef *GPIOx, uint16_t SCl, uint16_t SDA);

/*!
 * @brief  Get attitude angles (Madgwick AHRS + adaptive gains + no gimbal lock)
 * @param  this : Pointer to BMI270 struct
 */
void BMI270_Get_Angle_Plus(BMI270 *this);

/*!
 * @brief  Get attitude angles (complementary filter + Kalman filter)
 * @param  this : Pointer to BMI270 struct
 */
void BMI270_Get_Angle(BMI270 *this);

/*!
 * @brief  Zero the current attitude reference
 * @param  this : Pointer to BMI270 struct
 * @note   Call after sensor data has stabilized
 */
void BMI270_Set_Angle0(BMI270 *this);

/*===========================================================================*/
/*! @name     Extended API                                                    */
/*===========================================================================*/

/*!
 * @brief  Read BMI270 chip ID
 * @return CHIP_ID (should be 0x24), or 0xFF on communication failure
 */
uint8_t BMI270_ID(void);

/*!
 * @brief  Read temperature from BMI270
 * @param  this : Pointer to BMI270 struct
 * @return Temperature in degrees Celsius
 */
float BMI270_GetTemp(BMI270 *this);

/*!
 * @brief  Get attitude with variable dt (complementary filter)
 * @param  this : Pointer to BMI270 struct
 * @param  dt   : Time delta in seconds
 */
void BMI270_Get_AngleDt(BMI270 *this, float dt);

/*===========================================================================*/
/*! @name     Calibration API                                                 */
/*===========================================================================*/

/*!
 * @brief  Set vehicle moving state for online bias estimation gating
 * @param  moving : 0 = stopped (allow EMA bias update), 1 = moving (freeze bias)
 * @note   Call from app layer based on wheel speed before BMI270_Get_AngleDt().
 *         Prevents slow-turn gyro signal from being absorbed into bias estimate.
 */
void BMI270_SetVehicleMoving(uint8_t moving);

#endif /* BMI270_DRIVER_H_ */
