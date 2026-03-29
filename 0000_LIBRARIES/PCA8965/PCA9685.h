/*
 * PCA9685.h
 *
 * PCA9685 16-channel PWM driver library for STM32 HAL
 * Designed for servo control at 50Hz
 */

#ifndef PCA9685_H
#define PCA9685_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ── Register Addresses ───────────────────────────────────────────────────── */
#define PCA9685_MODE1               0x00
#define PCA9685_MODE2               0x01
#define PCA9685_PRE_SCALE           0xFE
#define PCA9685_LED0_ON_L           0x06

/* ── MODE1 Bit Positions ──────────────────────────────────────────────────── */
#define PCA9685_MODE1_SLEEP_BIT     4
#define PCA9685_MODE1_AI_BIT        5
#define PCA9685_MODE1_RESTART_BIT   7

/* ── Default Configuration ────────────────────────────────────────────────── */
#define PCA9685_DEFAULT_ADDR        0x40
#define PCA9685_INTERNAL_OSC_HZ     25000000
#define PCA9685_RESOLUTION          4096
#define PCA9685_PERIOD_MS           20      /* 50Hz → 20ms period            */

/* ── Servo Pulse Defaults (milliseconds) ──────────────────────────────────── */
#define PCA9685_SERVO_MIN_MS        0.5f    /* 0°   → 0.5ms pulse width      */
#define PCA9685_SERVO_MAX_MS        2.5f    /* 180° → 2.5ms pulse width      */

/* ── Config Struct ────────────────────────────────────────────────────────── */
typedef struct {
    I2C_HandleTypeDef  *hi2c;
    uint8_t             address;            /* 7-bit address, shifted in use  */
    uint16_t            frequency;          /* PWM frequency in Hz            */
    uint16_t            servo_min_ticks;    /* precomputed from servo_min_ms  */
    uint16_t            servo_max_ticks;    /* precomputed from servo_max_ms  */
} PCA9685_t;

/* ── Public Function Prototypes ───────────────────────────────────────────── */

/**
 * @brief  Weak default address provider. Override in application to change address.
 * @retval 7-bit I2C address (not shifted)
 */
uint8_t PCA9685_GetAddress(void);

/**
 * @brief  Initialise the PCA9685 and configure PWM frequency.
 * @param  pca           Pointer to PCA9685_t config struct
 * @param  hi2c          Pointer to HAL I2C handle
 * @param  frequency     PWM frequency in Hz (e.g. 50 for servo)
 * @param  servo_min_ms  Pulse width in ms corresponding to 0°
 * @param  servo_max_ms  Pulse width in ms corresponding to 180°
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef PCA9685_Init(PCA9685_t *pca, I2C_HandleTypeDef *hi2c,
                                uint16_t frequency,
                                float servo_min_ms, float servo_max_ms);

/**
 * @brief  Set raw PWM on/off ticks for a channel.
 * @param  pca      Pointer to PCA9685_t config struct
 * @param  channel  PWM channel (0–15)
 * @param  on_tick  Tick count at which signal goes HIGH (typically 0)
 * @param  off_tick Tick count at which signal goes LOW
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef PCA9685_SetPWM(PCA9685_t *pca, uint8_t channel,
                                  uint16_t on_tick, uint16_t off_tick);

/**
 * @brief  Set servo angle on a channel (0°–180°).
 * @param  pca      Pointer to PCA9685_t config struct
 * @param  channel  PWM channel (0–15)
 * @param  angle    Desired angle in degrees (0.0f – 180.0f)
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef PCA9685_SetServoAngle(PCA9685_t *pca, uint8_t channel,
                                         float angle);

#endif /* PCA9685_H */
