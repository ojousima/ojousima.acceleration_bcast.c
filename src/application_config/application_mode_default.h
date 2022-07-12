#ifndef APPLICATION_MODE_DEFAULT_H
#define APPLICATION_MODE_DEFAULT_H

/**
 * @file application_mode_default.h
 * @author Otso Jousimaa <otso@ojousima.net>
 * @date 2020-06-12
 * @copyright Ruuvi Innovations Ltd, License BSD-3-Clause
 *
 * Default configuration for Ruuvi Firmware.
 */

/**
 * @brief name for Device Information Service.
 */

#ifndef APP_FW_VARIANT
#define APP_FW_VARIANT "+default"
#endif

/** @brief Communicate sensor data at this interval. 1285 matches Apple guideline. */
#ifndef APP_BLE_INTERVAL_MS
#   define APP_BLE_INTERVAL_MS (1285U)
#endif

/** @brief repeat same data N times in advertisement, reduce sensor reads. */
#ifndef APP_NUM_REPEATS
#   define APP_NUM_REPEATS 5
#endif

/** @brief Refresh sensor data at this interval.*/
#ifndef APP_HEARTBEAT_INTERVAL_MS
#   define APP_HEARTBEAT_INTERVAL_MS (60U * 1000U)
#endif

/** @brief Resample battery voltage at this interval */
#ifndef APP_BATTERY_SAMPLE_MS
#   define APP_BATTERY_SAMPLE_MS (60ULL*1000ULL)
#endif

/** @brief Time to long press of a button. */
#ifndef APP_BUTTON_LONG_PRESS_TIME_MS
#   define APP_BUTTON_LONG_PRESS_TIME_MS (5000U)
#endif

/** @brief Configuration mode disabled on this timeout. */
#ifndef APP_CONFIG_ENABLED_TIME_MS
#   define APP_CONFIG_ENABLED_TIME_MS (1U * 60U * 1000U)
#endif

#ifndef APP_MOTION_THRESHOLD
#   define APP_MOTION_THRESHOLD (0.064F)
#endif

#ifndef APP_LOCKED_AT_BOOT
#   define APP_LOCKED_AT_BOOT (true)
#endif

/** @brief High-performance mode for accelerometer */
#ifndef APP_ACC_HIPERF_DSP_FUNC
#   define APP_ACC_HIPERF_DSP_FUNC RD_SENSOR_DSP_HIGH_PASS //HIGH_PASS, LOW_PASS, _LAST
#endif
 // 0, 1, 2, 3. 0 is lightest, 3 is strongest. Affects high and low pass, exact effect is not well defined
#ifndef APP_SENSOR_LIS2DH12_DSP_PARAM
#   define APP_ACC_HIPERF_DSP_PARAM (1U) 
#endif
#ifndef APP_ACC_HIPERF_MODE
#   define APP_ACC_HIPERF_MODE RD_SENSOR_CFG_CONTINUOUS
#endif
#ifndef APP_ACC_HIPERF_RESOLUTION
#   define APP_ACC_HIPERF_RESOLUTION (10U) //!< bits, 8, 10, 12 allowed
#endif
// Values: 1-200, Hz, rounded up to what sensor supports
// RD_SENSOR_CFG_CUSTOM_1: 400 Hz on every resolution
// RD_SENSOR_CFG_CUSTOM_2: Allowed only on 8 bit resolution, 1620 Hz
// RD_SENSOR_CFG_CUSTOM_3: 12, 10 bits: 1344 Hz, 8 bit 5376 Hz
#ifndef APP_ACC_HIPERF_SAMPLERATE
#   define APP_ACC_HIPERF_SAMPLERATE (RD_SENSOR_CFG_CUSTOM_3) 
#endif
#ifndef APP_ACC_HIPERF_SCALE
#   define APP_ACC_HIPERF_SCALE (2U) //!< G, 2, 4, 8, 16 allowed.
#endif
#endif
