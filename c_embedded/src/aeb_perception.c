/**
 * @file aeb_perception.c
 * @brief Perception plausibility checks for the AEB system.
 *
 * Validates raw sensor inputs (range, speed, rate-of-change) and manages
 * a fault counter that latches after 3 consecutive invalid readings.
 *
 * MISRA C:2012 compliant: fixed-width types, no dynamic allocation,
 * no recursion, all variables initialised at declaration.
 */

#include "aeb_types.h"
#include "aeb_config.h"
#include "aeb_perception.h"
#include <math.h>

/* ---- Rate-of-change limits (per cycle, not per second) ------------------ */
#define DISTANCE_ROC_MAX    10.0F   /**< Max distance change per cycle [m].  */
#define SPEED_ROC_MAX        2.0F   /**< Max speed change per cycle [m/s].   */

/* ---- Speed plausibility window ------------------------------------------ */
#define SPEED_PLAUS_MAX     50.0F   /**< Maximum plausible speed [m/s].      */

/* ---- Module-static state ------------------------------------------------ */
static PerceptionData_t s_data = {
    0.0F,   /* distance        */
    0.0F,   /* v_ego           */
    0.0F,   /* v_target        */
    0.0F,   /* v_rel           */
    0U,     /* brake_pedal     */
    0.0F,   /* steering_angle  */
    0U,     /* fault           */
    1.0F    /* confidence      */
};

static float32_t s_prev_distance  = 0.0F;
static float32_t s_prev_v_ego     = 0.0F;
static float32_t s_prev_v_target  = 0.0F;
static uint8_t   s_fault_counter  = 0U;
static uint8_t   s_first_cycle    = 1U;

/* ---- Helper: clamp float to [lo, hi] ----------------------------------- */
static float32_t clampf(float32_t val, float32_t lo, float32_t hi)
{
    float32_t out = val;
    if (out < lo) { out = lo; }
    if (out > hi) { out = hi; }
    return out;
}

/* ---- Public API --------------------------------------------------------- */

void perception_init(void)
{
    s_data.distance       = 0.0F;
    s_data.v_ego          = 0.0F;
    s_data.v_target       = 0.0F;
    s_data.v_rel          = 0.0F;
    s_data.brake_pedal    = 0U;
    s_data.steering_angle = 0.0F;
    s_data.fault          = 0U;
    s_data.confidence     = 1.0F;

    s_prev_distance  = 0.0F;
    s_prev_v_ego     = 0.0F;
    s_prev_v_target  = 0.0F;
    s_fault_counter  = 0U;
    s_first_cycle    = 1U;
}

void perception_update(float32_t distance_raw,
                       float32_t v_ego_raw,
                       float32_t v_target_raw,
                       uint8_t   brake_pedal,
                       float32_t steering_angle)
{
    uint8_t cycle_fault = 0U;

    /* ---- Range plausibility: [0.5, 300] m ------------------------------- */
    if ((distance_raw < RANGE_MIN) || (distance_raw > RANGE_MAX))
    {
        cycle_fault = 1U;
    }

    /* ---- Speed plausibility: [0, 50] m/s -------------------------------- */
    if ((v_ego_raw < 0.0F) || (v_ego_raw > SPEED_PLAUS_MAX))
    {
        cycle_fault = 1U;
    }
    if ((v_target_raw < 0.0F) || (v_target_raw > SPEED_PLAUS_MAX))
    {
        cycle_fault = 1U;
    }

    /* ---- Rate-of-change checks (skip first cycle) ----------------------- */
    if (s_first_cycle == 0U)
    {
        if (fabsf(distance_raw - s_prev_distance) > DISTANCE_ROC_MAX)
        {
            cycle_fault = 1U;
        }
        if (fabsf(v_ego_raw - s_prev_v_ego) > SPEED_ROC_MAX)
        {
            cycle_fault = 1U;
        }
        if (fabsf(v_target_raw - s_prev_v_target) > SPEED_ROC_MAX)
        {
            cycle_fault = 1U;
        }
    }

    /* ---- 3-cycle consecutive fault detection ----------------------------- */
    if (cycle_fault != 0U)
    {
        if (s_fault_counter < 255U)
        {
            s_fault_counter++;
        }
    }
    else
    {
        s_fault_counter = 0U;
    }

    /* ---- Populate fused output structure --------------------------------- */
    s_data.distance       = clampf(distance_raw, RANGE_MIN, RANGE_MAX);
    s_data.v_ego          = clampf(v_ego_raw, 0.0F, SPEED_PLAUS_MAX);
    s_data.v_target       = clampf(v_target_raw, 0.0F, SPEED_PLAUS_MAX);
    s_data.v_rel          = s_data.v_ego - s_data.v_target;
    s_data.brake_pedal    = brake_pedal;
    s_data.steering_angle = steering_angle;

    /* ---- Latch fault after SENSOR_FAULT_CYCLES consecutive failures ----- */
    if (s_fault_counter >= (uint8_t)SENSOR_FAULT_CYCLES)
    {
        s_data.fault      = 1U;
        s_data.confidence = 0.0F;
    }
    else
    {
        s_data.fault      = 0U;
        s_data.confidence = 1.0F;
    }

    /* ---- Store previous values for next cycle --------------------------- */
    s_prev_distance = distance_raw;
    s_prev_v_ego    = v_ego_raw;
    s_prev_v_target = v_target_raw;
    s_first_cycle   = 0U;
}

const PerceptionData_t* perception_get_data(void)
{
    return &s_data;
}
