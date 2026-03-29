/*
 * PCA9685.c
 *
 * PCA9685 16-channel PWM driver library for STM32 HAL
 * Designed for servo control at 50Hz
 */

#include "PCA9685.h"
#include <math.h>

/* ── Internal Helper ──────────────────────────────────────────────────────── */

/**
 * @brief  Read-modify-write a single bit in a PCA9685 register.
 * @param  pca    Pointer to PCA9685_t config struct
 * @param  reg    Register address
 * @param  bit    Bit position (0–7)
 * @param  value  0 to clear, 1 to set
 * @retval HAL_StatusTypeDef
 */
static HAL_StatusTypeDef PCA9685_SetBit(PCA9685_t *pca, uint8_t reg,
                                         uint8_t bit, uint8_t value)
{
    HAL_StatusTypeDef status;
    uint8_t reg_val;

    status = HAL_I2C_Mem_Read(pca->hi2c, (uint16_t)(pca->address << 1),
                               reg, 1, &reg_val, 1, 10);
    if (status != HAL_OK) return status;

    if (value == 0) reg_val &= ~(1 << bit);
    else            reg_val |=  (1 << bit);

    status = HAL_I2C_Mem_Write(pca->hi2c, (uint16_t)(pca->address << 1),
                                reg, 1, &reg_val, 1, 10);
    return status;
}

/* ── Weak Address Provider ────────────────────────────────────────────────── */

__weak uint8_t PCA9685_GetAddress(void)
{
    return PCA9685_DEFAULT_ADDR;
}

/* ── Public API ───────────────────────────────────────────────────────────── */

HAL_StatusTypeDef PCA9685_Init(PCA9685_t *pca, I2C_HandleTypeDef *hi2c,
                                uint16_t frequency,
                                float servo_min_ms, float servo_max_ms)
{
    HAL_StatusTypeDef status;

    /* Populate struct ───────────────────────────────────────────────────── */
    pca->hi2c      = hi2c;
    pca->address   = PCA9685_GetAddress();
    pca->frequency = frequency;

    /* Precompute servo tick limits — float used here only, stored as uint16 */
    pca->servo_min_ticks = (uint16_t)((servo_min_ms / PCA9685_PERIOD_MS)
                                       * PCA9685_RESOLUTION);
    pca->servo_max_ticks = (uint16_t)((servo_max_ms / PCA9685_PERIOD_MS)
                                       * PCA9685_RESOLUTION);

    /* Step 1: Set MODE2 — totem-pole output drive (OUTDRV bit, datasheet p.16) */
    uint8_t mode2 = 0x04;
    status = HAL_I2C_Mem_Write(pca->hi2c, (uint16_t)(pca->address << 1),
                                PCA9685_MODE2, 1, &mode2, 1, 10);
    if (status != HAL_OK) return status;

    /* Step 2: Enter sleep mode to allow prescaler write ─────────────────── */
    status = PCA9685_SetBit(pca, PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 1);
    if (status != HAL_OK) return status;

    /* Step 3: Write prescaler (datasheet formula: round(osc / (4096*f)) - 1) */
    uint8_t prescale = (uint8_t)(round((float)PCA9685_INTERNAL_OSC_HZ
                                        / (PCA9685_RESOLUTION * frequency)) - 1);

    status = HAL_I2C_Mem_Write(pca->hi2c, (uint16_t)(pca->address << 1),
                                PCA9685_PRE_SCALE, 1, &prescale, 1, 10);
    if (status != HAL_OK) return status;

    /* Step 4: Wake up (clear sleep bit) ─────────────────────────────────── */
    status = PCA9685_SetBit(pca, PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 0);
    if (status != HAL_OK) return status;

    /* Step 5: Enable auto-increment ─────────────────────────────────────── */
    status = PCA9685_SetBit(pca, PCA9685_MODE1, PCA9685_MODE1_AI_BIT, 1);
    if (status != HAL_OK) return status;

    /* Step 6: Trigger restart ───────────────────────────────────────────── */
    status = PCA9685_SetBit(pca, PCA9685_MODE1, PCA9685_MODE1_RESTART_BIT, 1);
    return status;
}

HAL_StatusTypeDef PCA9685_SetPWM(PCA9685_t *pca, uint8_t channel,
                                  uint16_t on_tick, uint16_t off_tick)
{
    uint8_t reg;
    uint8_t pwm[4];

    reg    = PCA9685_LED0_ON_L + (4 * channel);
    pwm[0] = (uint8_t)(on_tick  & 0xFF);
    pwm[1] = (uint8_t)(on_tick  >> 8);
    pwm[2] = (uint8_t)(off_tick & 0xFF);
    pwm[3] = (uint8_t)(off_tick >> 8);

    return HAL_I2C_Mem_Write(pca->hi2c, (uint16_t)(pca->address << 1),
                              reg, 1, pwm, 4, 10);
}

HAL_StatusTypeDef PCA9685_SetServoAngle(PCA9685_t *pca, uint8_t channel,
                                         float angle)
{
    uint16_t off_tick;

    /* Linear interpolation between precomputed min and max ticks ────────── */
    off_tick = (uint16_t)(pca->servo_min_ticks
               + (angle * (pca->servo_max_ticks - pca->servo_min_ticks)
                  / 180.0f));

    return PCA9685_SetPWM(pca, channel, 0, off_tick);
}
