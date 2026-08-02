/*
 * ultrasonic.c
 *
 *  Created on: Jul 6, 2026
 *      Author: DELL
 */

#include "ultrasonic.h"

typedef struct {
    GPIO_TypeDef *trig_port;
    uint16_t      trig_pin;
    GPIO_TypeDef *echo_port;
    uint16_t      echo_pin;
} HCSR04_Sensor_t;

static const HCSR04_Sensor_t SENSORS[HCSR04_COUNT] = {
    [HCSR04_FRONT] = { FRONT_TRIG_PORT, FRONT_TRIG_PIN,
                        FRONT_ECHO_PORT, FRONT_ECHO_PIN },
    [HCSR04_LEFT]  = { LEFT_TRIG_PORT,  LEFT_TRIG_PIN,
                        LEFT_ECHO_PORT,  LEFT_ECHO_PIN  },
    [HCSR04_RIGHT] = { RIGHT_TRIG_PORT, RIGHT_TRIG_PIN,
                        RIGHT_ECHO_PORT, RIGHT_ECHO_PIN },
};

static HCSR04_Result_t results[HCSR04_COUNT];
/* ── Median-of-3 filter history ─────────────────────────────────────── */
#define HCSR04_HISTORY_LEN 3
static float hist[HCSR04_COUNT][HCSR04_HISTORY_LEN];
static uint8_t hist_idx[HCSR04_COUNT] = {0};
static uint8_t hist_filled[HCSR04_COUNT] = {0};
/* ── FIX 1: Cast to uint16_t for safe 16-bit rollover ─────────────────── */
static inline uint16_t TIM2_GetUS(void)
{
    return (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
}

static void Delay_US(uint16_t us)
{
    uint16_t start = TIM2_GetUS();
    while ((uint16_t)(TIM2_GetUS() - start) < us);
}

void HCSR04_Init(void)
{
    for (int i = 0; i < HCSR04_COUNT; i++) {
        results[i].distance_cm = HCSR04_MAX_CM;
        results[i].valid       = 0;
        hist_idx[i]    = 0;
        hist_filled[i] = 0;
        for (int j = 0; j < HCSR04_HISTORY_LEN; j++) {
            hist[i][j] = HCSR04_MAX_CM;
        }
        HAL_GPIO_WritePin(SENSORS[i].trig_port,
                          SENSORS[i].trig_pin,
                          GPIO_PIN_RESET);
    }
}
static float median3(float a, float b, float c)
{
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b;
}

HCSR04_Result_t HCSR04_Read(HCSR04_ID id)
{
    HCSR04_Result_t res  = { HCSR04_MAX_CM, 0 };
    const HCSR04_Sensor_t *s = &SENSORS[id];

    /* ── Step 1: Send 10µs TRIG pulse ─────────────────────────────── */
    HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_RESET);
    Delay_US(2);                              /* ensure clean LOW start */
    HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_SET);
    Delay_US(10);                             /* 10µs HIGH              */
    HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_RESET);

    /* ── Step 2: Wait for ECHO HIGH ────────────────────────────────── */
    uint16_t t_start = TIM2_GetUS();
    while (HAL_GPIO_ReadPin(s->echo_port, s->echo_pin) == GPIO_PIN_RESET) {
        if ((uint16_t)(TIM2_GetUS() - t_start) > 30000U) {
            results[id] = res;               /* timeout — return invalid */
            return res;
        }
    }
    uint16_t echo_rise = TIM2_GetUS();

    /* ── Step 3: Wait for ECHO LOW ─────────────────────────────────── */
    while (HAL_GPIO_ReadPin(s->echo_port, s->echo_pin) == GPIO_PIN_SET) {
        if ((uint16_t)(TIM2_GetUS() - echo_rise) > 30000U) {
            results[id] = res;
            return res;
        }
    }
    uint16_t echo_fall = TIM2_GetUS();

    /* ── Step 4: Calculate distance ────────────────────────────────── */
    uint16_t pulse_us  = echo_fall - echo_rise;
    float    dist_cm   = (pulse_us * SOUND_SPEED_CM_PER_US) / 2.0f;
    dist_cm = CLAMP(dist_cm, HCSR04_MIN_CM, HCSR04_MAX_CM);
    /* ── Step 5: Median-of-3 filter to reject single-sample glitches ─ */
        hist[id][hist_idx[id]] = dist_cm;
        hist_idx[id] = (hist_idx[id] + 1) % HCSR04_HISTORY_LEN;
        if (hist_filled[id] < HCSR04_HISTORY_LEN) hist_filled[id]++;

        float filtered_cm;
        if (hist_filled[id] < HCSR04_HISTORY_LEN) {
            /* Not enough history yet — pass raw value through */
            filtered_cm = dist_cm;
        } else {
            filtered_cm = median3(hist[id][0], hist[id][1], hist[id][2]);
        }


    res.distance_cm =filtered_cm;
    res.valid       = 1;
    results[id]     = res;
    return res;
}

void HCSR04_ReadAll(void)
{
    for (int i = 0; i < HCSR04_COUNT; i++) {
        HCSR04_Read((HCSR04_ID)i);
        HAL_Delay(60);
    }
}

float   HCSR04_GetDistance(HCSR04_ID id) { return results[id].distance_cm; }
uint8_t HCSR04_IsValid    (HCSR04_ID id) { return results[id].valid;        }
