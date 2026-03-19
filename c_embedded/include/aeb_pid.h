/**
 * @file aeb_pid.h
 * @brief PID controller interface for AEB brake-pressure control.
 */

#ifndef AEB_PID_H
#define AEB_PID_H

#include "aeb_types.h"

/**
 * @brief Initialise the PID controller (zero gains accumulator).
 */
void pid_init(void);

/**
 * @brief Compute one PID cycle.
 *
 * @param[in] target_decel Desired deceleration [m/s^2].
 * @param[in] actual_decel Measured deceleration [m/s^2].
 * @param[in] dt           Time step [s].
 *
 * @return Brake command output [0 .. PID_OUTPUT_MAX] (%).
 */
float32_t pid_compute(float32_t target_decel,
                      float32_t actual_decel,
                      float32_t dt);

/**
 * @brief Reset the PID integrator and internal state.
 */
void pid_reset(void);

#endif /* AEB_PID_H */
