/**
 * @file aeb_perception.h
 * @brief Perception module interface for the AEB system.
 *
 * Provides sensor-data validation, fault detection and fused
 * perception output consumed by the TTC and FSM modules.
 */

#ifndef AEB_PERCEPTION_H
#define AEB_PERCEPTION_H

#include "aeb_types.h"

/**
 * @brief Initialise the perception module (reset internal state).
 */
void perception_init(void);

/**
 * @brief Process raw sensor inputs and update fused perception data.
 *
 * @param[in] distance_raw   Raw distance measurement [m].
 * @param[in] v_ego_raw      Raw ego-vehicle speed [m/s].
 * @param[in] v_target_raw   Raw target-object speed [m/s].
 * @param[in] brake_pedal    Driver brake-pedal flag (0/1).
 * @param[in] steering_angle Steering-wheel angle [deg].
 */
void perception_update(float32_t distance_raw,
                       float32_t v_ego_raw,
                       float32_t v_target_raw,
                       uint8_t   brake_pedal,
                       float32_t steering_angle);

/**
 * @brief Retrieve the latest fused perception data.
 *
 * @return Pointer to the internal (static) PerceptionData_t structure.
 */
const PerceptionData_t* perception_get_data(void);

#endif /* AEB_PERCEPTION_H */
