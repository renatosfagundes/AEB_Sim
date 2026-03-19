/**
 * @file aeb_alert.h
 * @brief Alert management interface for the AEB system.
 *
 * Controls visual and audible driver warnings based on the
 * current FSM state.
 */

#ifndef AEB_ALERT_H
#define AEB_ALERT_H

#include "aeb_types.h"

/**
 * @brief Initialise the alert module (all alerts off).
 */
void alert_init(void);

/**
 * @brief Update alert outputs based on the current AEB state.
 *
 * @param[in] state Current AEB FSM state.
 */
void alert_update(AEB_State_t state);

/**
 * @brief Query the visual alert status.
 *
 * @return 1 if visual alert is active, 0 otherwise.
 */
uint8_t alert_get_visual(void);

/**
 * @brief Query the audible alert status.
 *
 * @return 1 if audible alert is active, 0 otherwise.
 */
uint8_t alert_get_audible(void);

#endif /* AEB_ALERT_H */
