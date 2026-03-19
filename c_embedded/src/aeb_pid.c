/**
 * @file aeb_pid.c
 * @brief PI brake-pressure controller for the AEB system.
 *
 * Kp = 4.0, Ki = 0.05
 * Anti-windup: integrator clamped to [0, 50]
 * Output clamped to [0, 100] %
 * Jerk limiting on output rate of change
 *
 * MISRA C:2012 compliant: fixed-width types, no dynamic allocation,
 * no recursion, all variables initialised at declaration.
 */

#include "aeb_types.h"
#include "aeb_config.h"
#include "aeb_pid.h"
#include <math.h>

/* ---- Anti-windup integrator limits -------------------------------------- */
#define INTEGRATOR_MIN  0.0F
#define INTEGRATOR_MAX  50.0F

/* ---- Module-static state ----------------------------------------------- */
static float32_t s_integral    = 0.0F;
static float32_t s_prev_output = 0.0F;

/* ---- Helper: clamp float to [lo, hi] ----------------------------------- */
static float32_t clampf(float32_t val, float32_t lo, float32_t hi)
{
    float32_t out = val;
    if (out < lo) { out = lo; }
    if (out > hi) { out = hi; }
    return out;
}

/* ---- Public API --------------------------------------------------------- */

void pid_init(void)
{
    s_integral    = 0.0F;
    s_prev_output = 0.0F;
}

void pid_reset(void)
{
    s_integral    = 0.0F;
    s_prev_output = 0.0F;
}

float32_t pid_compute(float32_t target_decel,
                      float32_t actual_decel,
                      float32_t dt)
{
    float32_t error     = 0.0F;
    float32_t p_term    = 0.0F;
    float32_t output    = 0.0F;
    float32_t delta     = 0.0F;
    float32_t max_delta = 0.0F;

    /* ---- Guard: no deceleration requested -> zero output --------------- */
    if (target_decel <= 0.0F)
    {
        s_integral    = 0.0F;
        s_prev_output = 0.0F;
        return 0.0F;
    }

    /* ---- Guard: zero or negative dt ------------------------------------ */
    if (dt <= 0.0F)
    {
        return s_prev_output;
    }

    /* ---- Error: positive when more braking is needed ------------------- */
    error = target_decel - actual_decel;

    /* ---- Proportional term --------------------------------------------- */
    p_term = PID_KP * error;

    /* ---- Integral term with anti-windup clamping [0, 50] --------------- */
    s_integral += PID_KI * error * dt;
    s_integral  = clampf(s_integral, INTEGRATOR_MIN, INTEGRATOR_MAX);

    /* ---- Raw PI output ------------------------------------------------- */
    output = p_term + s_integral;

    /* ---- Output saturation [0, 100] % ---------------------------------- */
    output = clampf(output, PID_OUTPUT_MIN, PID_OUTPUT_MAX);

    /* ---- Jerk limiting: cap output rate of change ---------------------- */
    /* Convert MAX_JERK [m/s^3] to %/s via scaling factor */
    max_delta = (MAX_JERK * (PID_OUTPUT_MAX / BRAKE_MAX_DECEL)) * dt;

    delta = output - s_prev_output;

    if (delta > max_delta)
    {
        output = s_prev_output + max_delta;
    }
    else if (delta < -max_delta)
    {
        output = s_prev_output - max_delta;
    }
    else
    {
        /* No rate limiting needed */
    }

    /* ---- Final clamp after jerk limiting ------------------------------- */
    output = clampf(output, PID_OUTPUT_MIN, PID_OUTPUT_MAX);

    /* ---- Store for next cycle ------------------------------------------ */
    s_prev_output = output;

    return output;
}
