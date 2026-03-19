/**
 * @file aeb_ttc.h
 * @brief Time-to-collision computation interface for the AEB system.
 */

#ifndef AEB_TTC_H
#define AEB_TTC_H

#include "aeb_types.h"

/**
 * @brief Initialise the TTC module (reset internal state).
 */
void ttc_init(void);

/**
 * @brief Compute time-to-collision and braking distance.
 *
 * @param[in] distance  Distance to target [m].
 * @param[in] v_ego     Ego-vehicle speed [m/s].
 * @param[in] v_target  Target-object speed [m/s].
 *
 * @return TTCResult_t containing ttc, d_brake and is_closing flag.
 */
TTCResult_t ttc_compute(float32_t distance,
                        float32_t v_ego,
                        float32_t v_target);

#endif /* AEB_TTC_H */
