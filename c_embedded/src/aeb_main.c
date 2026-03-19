/**
 * @file aeb_main.c
 * @brief Top-level 10 ms cycle integrator for the AEB system.
 *
 * Orchestrates all sub-modules in the correct order:
 *   perception -> TTC -> FSM -> PID (if braking) -> alert
 *
 * MISRA C:2012 compliant: fixed-width types, no dynamic allocation,
 * no recursion, all variables initialised at declaration.
 */

#include "aeb_types.h"
#include "aeb_config.h"
#include "aeb_perception.h"
#include "aeb_ttc.h"
#include "aeb_fsm.h"
#include "aeb_pid.h"
#include "aeb_alert.h"
#include "aeb_main.h"

/* ---- Module-static state ----------------------------------------------- */
static float32_t s_brake_cmd = 0.0F;

/* ---- Public API -------------------------------------------------------- */

void aeb_init(void)
{
    perception_init();
    ttc_init();
    fsm_init();
    pid_init();
    alert_init();

    s_brake_cmd = 0.0F;
}

void aeb_cycle_10ms(float32_t distance_raw,
                    float32_t v_ego_raw,
                    float32_t v_target_raw,
                    uint8_t   brake_pedal,
                    float32_t steering_angle,
                    float32_t actual_decel)
{
    const PerceptionData_t *pd      = (const PerceptionData_t *)0;
    TTCResult_t             ttc_res = { TTC_MAX, 0.0F, 0U };
    FSMOutput_t             fsm_out = { AEB_STANDBY, 0.0F, 0U, 0U, 0U };
    float32_t               dt      = SIM_DT_CONTROLLER;

    /* 1. Perception: validate and fuse raw sensor data ------------------- */
    perception_update(distance_raw, v_ego_raw, v_target_raw,
                      brake_pedal, steering_angle);
    pd = perception_get_data();

    /* 2. TTC: compute time-to-collision and braking distance ------------- */
    ttc_res = ttc_compute(pd->distance, pd->v_ego, pd->v_target);

    /* 3. FSM: determine state, deceleration demand, alert flags ---------- */
    fsm_out = fsm_update(&ttc_res, pd, dt);

    /* 4. PID: compute brake command only when braking is active ---------- */
    if (fsm_out.brake_active != 0U)
    {
        s_brake_cmd = pid_compute(fsm_out.target_decel, actual_decel, dt);
    }
    else
    {
        pid_reset();
        s_brake_cmd = 0.0F;
    }

    /* 5. Alert: update visual / audible warnings ------------------------- */
    alert_update(fsm_out.state);
}

float32_t aeb_get_brake_cmd(void)
{
    return s_brake_cmd;
}

AEB_State_t aeb_get_state(void)
{
    return fsm_get_state();
}
