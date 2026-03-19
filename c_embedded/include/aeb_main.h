/**
 * @file aeb_main.h
 * @brief Top-level AEB application interface.
 *
 * Orchestrates the 10 ms cyclic call that chains perception,
 * TTC, FSM, PID and alert modules.
 */

#ifndef AEB_MAIN_H
#define AEB_MAIN_H

#include "aeb_types.h"

/**
 * @brief Initialise all AEB sub-modules.
 */
void aeb_init(void);

/**
 * @brief Execute one 10 ms AEB cycle.
 *
 * @param[in] distance_raw   Raw distance measurement [m].
 * @param[in] v_ego_raw      Raw ego-vehicle speed [m/s].
 * @param[in] v_target_raw   Raw target-object speed [m/s].
 * @param[in] brake_pedal    Driver brake-pedal flag (0/1).
 * @param[in] steering_angle Steering-wheel angle [deg].
 * @param[in] actual_decel   Measured vehicle deceleration [m/s^2].
 */
void aeb_cycle_10ms(float32_t distance_raw,
                    float32_t v_ego_raw,
                    float32_t v_target_raw,
                    uint8_t   brake_pedal,
                    float32_t steering_angle,
                    float32_t actual_decel);

/**
 * @brief Retrieve the current brake command.
 *
 * @return Brake command [0 .. 100] (%).
 */
float32_t aeb_get_brake_cmd(void);

/**
 * @brief Retrieve the current AEB state.
 *
 * @return Current AEB_State_t value.
 */
AEB_State_t aeb_get_state(void);

#endif /* AEB_MAIN_H */
