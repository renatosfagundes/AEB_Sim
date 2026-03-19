/**
 * @file aeb_fsm.h
 * @brief Finite-state-machine interface for the AEB system.
 *
 * Implements the state transitions (STANDBY -> WARNING -> BRAKE_Lx ->
 * POST_BRAKE) based on TTC thresholds and perception data.
 */

#ifndef AEB_FSM_H
#define AEB_FSM_H

#include "aeb_types.h"

/**
 * @brief Initialise the FSM (set state to AEB_STANDBY, clear timers).
 */
void fsm_init(void);

/**
 * @brief Execute one FSM cycle.
 *
 * @param[in] ttc        Pointer to the latest TTC result.
 * @param[in] perception Pointer to the latest perception data.
 * @param[in] dt         Time step [s].
 *
 * @return FSMOutput_t with the current state, decel demand and alerts.
 */
FSMOutput_t fsm_update(const TTCResult_t      *ttc,
                       const PerceptionData_t *perception,
                       float32_t               dt);

/**
 * @brief Query the current FSM state without running an update.
 *
 * @return Current AEB_State_t value.
 */
AEB_State_t fsm_get_state(void);

#endif /* AEB_FSM_H */
