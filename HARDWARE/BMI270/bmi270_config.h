/**
 * @file    bmi270_config.h
 * @brief   BMI270 configuration file declarations
 * @note    Extracted from Bosch BMI270 SensorAPI v2.86.1
 *
 * This 8192-byte configuration blob is Bosch-proprietary sensor firmware.
 * It must be uploaded to BMI270 internal RAM via INIT_DATA (0x5E) on every
 * power-up before the sensor can produce valid data.
 *
 * The config enables:
 *  - Accelerometer + Gyroscope in performance mode
 *  - Gyroscope cross-axis sensitivity compensation
 *  - Component Re-Trim (CRT) support
 */

#ifndef BMI270_CONFIG_H_
#define BMI270_CONFIG_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================*/
/*! @name Configuration file size in bytes */
#define BMI270_CONFIG_SIZE  8192U

/*===========================================================================*/
/*! @name Global array that stores the configuration file of BMI270 */
extern const uint8_t bmi270_config_file[8192];

#ifdef __cplusplus
}
#endif

#endif /* BMI270_CONFIG_H_ */
