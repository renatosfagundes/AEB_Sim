/**
 * @file aeb_fsm.c
 * @brief 7-state finite-state machine for the AEB system.
 *
 * States: OFF, STANDBY, WARNING, BRAKE_L1, BRAKE_L2, BRAKE_L3, POST_BRAKE
 *
 * Transition rules:
 *   - TTC thresholds: 4.0 / 3.0 / 2.2 / 1.8 s with TTC_HYSTERESIS band
 *   - d_brake >= distance triggers escalation alongside TTC
 *   - Hysteresis 200 ms hold before de-escalation
 *   - Warning minimum 800 ms before brake escalation
 *   - POST_BRAKE hold 2 s then return to STANDBY
 *   - Fault -> OFF
 *   - Driver override (brake pedal or steering) -> STANDBY
 *
 * MISRA C:2012 compliant: fixed-width types, no dynamic allocation,
 * no recursion, all variables initialised at declaration.
 */

#include "aeb_types.h"
#include "aeb_config.h"
#include "aeb_fsm.h"
#include <math.h>

/* ---- Module-static state ----------------------------------------------- */
static AEB_State_t s_state              = AEB_STANDBY;
static float32_t   s_state_timer        = 0.0F;
static float32_t   s_warning_accum      = 0.0F;
static float32_t   s_debounce_timer     = 0.0F;

/* ---- Forward declaration ----------------------------------------------- */
static FSMOutput_t build_output(AEB_State_t state);

/* ---- Helper: detect driver override ------------------------------------ */
static uint8_t driver_override(const PerceptionData_t *p)
{
    uint8_t ovr = 0U;
    if (p->brake_pedal != 0U)
    {
        ovr = 1U;
    }
    if (fabsf(p->steering_angle) > STEERING_OVERRIDE_DEG)
    {
        ovr = 1U;
    }
    return ovr;
}

/* ---- Helper: evaluate desired threat level from TTC + d_brake ---------- */
static AEB_State_t evaluate_threat(const TTCResult_t      *ttc,
                                   const PerceptionData_t *perception)
{
    AEB_State_t target = AEB_STANDBY;

    if (ttc->is_closing == 0U)
    {
        target = AEB_STANDBY;
    }
    else if ((ttc->ttc <= TTC_BRAKE_L3) || (ttc->d_brake >= perception->distance))
    {
        target = AEB_BRAKE_L3;
    }
    else if (ttc->ttc <= TTC_BRAKE_L2)
    {
        target = AEB_BRAKE_L2;
    }
    else if (ttc->ttc <= TTC_BRAKE_L1)
    {
        target = AEB_BRAKE_L1;
    }
    else if (ttc->ttc <= TTC_WARNING)
    {
        target = AEB_WARNING;
    }
    else
    {
        target = AEB_STANDBY;
    }

    /* ---- Distance-based braking floor ----------------------------------- */
    /* Prevent TTC-based de-escalation while the vehicle is still closing.
     * When is_closing == 0 (v_rel <= 0), the floor is skipped so a moving
     * target that has been matched in speed causes natural braking release —
     * the ego will not brake to a standstill unnecessarily. */
    if (ttc->is_closing != 0U)
    {
        if ((perception->distance <= D_BRAKE_L3) && (target < AEB_BRAKE_L3))
        {
            target = AEB_BRAKE_L3;
        }
        else if ((perception->distance <= D_BRAKE_L2) && (target < AEB_BRAKE_L2))
        {
            target = AEB_BRAKE_L2;
        }
        else if ((perception->distance <= D_BRAKE_L1) && (target < AEB_BRAKE_L1))
        {
            target = AEB_BRAKE_L1;
        }
        else
        {
            /* Distance above all floors — TTC result stands */
        }
    }

    return target;
}

/* ---- Helper: transition with timer reset ------------------------------- */
static void transition_to(AEB_State_t next)
{
    if (next != s_state)
    {
        s_state       = next;
        s_state_timer = 0.0F;
    }
}

/* ---- Public API -------------------------------------------------------- */

void fsm_init(void)
{
    s_state          = AEB_STANDBY;
    s_state_timer    = 0.0F;
    s_warning_accum  = 0.0F;
    s_debounce_timer = 0.0F;
}

AEB_State_t fsm_get_state(void)
{
    return s_state;
}

FSMOutput_t fsm_update(const TTCResult_t      *ttc,
                       const PerceptionData_t *perception,
                       float32_t               dt)
{
    AEB_State_t desired = AEB_STANDBY;

    s_state_timer += dt;

    /* ================================================================== */
    /* Global overrides (checked in every state)                          */
    /* ================================================================== */

    /* ---- Fault -> OFF -------------------------------------------------- */
    if (perception->fault != 0U)
    {
        s_state          = AEB_OFF;
        s_state_timer    = 0.0F;
        s_warning_accum  = 0.0F;
        s_debounce_timer = 0.0F;
        return build_output(s_state);
    }

    /* ---- Ego speed outside valid window -> STANDBY --------------------- */
    if ((perception->v_ego < V_EGO_MIN) || (perception->v_ego > V_EGO_MAX))
    {
        /* Exception 1: vehicle stopped while braking -> POST_BRAKE */
        if ((perception->v_ego < 0.01F) &&
            ((s_state == AEB_BRAKE_L1) ||
             (s_state == AEB_BRAKE_L2) ||
             (s_state == AEB_BRAKE_L3)))
        {
            transition_to(AEB_POST_BRAKE);
            return build_output(s_state);
        }

        /* Exception 2: close-range still closing — maintain minimum brake level.
         * The distance floor in evaluate_threat cannot be reached because this
         * block returns early; replicate the floor logic here.
         * Moving targets release naturally when v_rel drops to zero. */
        if ((perception->distance <= D_BRAKE_L1) && (perception->v_rel > 0.0F))
        {
            AEB_State_t min_close = AEB_BRAKE_L1;
            if (perception->distance <= D_BRAKE_L3)
            {
                min_close = AEB_BRAKE_L3;
            }
            else if (perception->distance <= D_BRAKE_L2)
            {
                min_close = AEB_BRAKE_L2;
            }
            else
            {
                /* D_BRAKE_L2 < distance <= D_BRAKE_L1: L1 floor */
            }
            if (s_state < min_close)
            {
                transition_to(min_close);
            }
            return build_output(s_state);
        }

        s_state          = AEB_STANDBY;
        s_state_timer    = 0.0F;
        s_warning_accum  = 0.0F;
        s_debounce_timer = 0.0F;
        return build_output(s_state);
    }

    /* ---- Driver override -> STANDBY ------------------------------------ */
    if (driver_override(perception) != 0U)
    {
        s_state          = AEB_STANDBY;
        s_state_timer    = 0.0F;
        s_warning_accum  = 0.0F;
        s_debounce_timer = 0.0F;
        return build_output(s_state);
    }

    /* ================================================================== */
    /* Evaluate desired threat level                                       */
    /* ================================================================== */
    desired = evaluate_threat(ttc, perception);

    /* ================================================================== */
    /* State-specific transition logic                                     */
    /* ================================================================== */
    switch (s_state)
    {
        case AEB_OFF:
            /* Fault already cleared (handled above), resume monitoring */
            s_state          = AEB_STANDBY;
            s_state_timer    = 0.0F;
            s_warning_accum  = 0.0F;
            break;

        case AEB_STANDBY:
            s_warning_accum  = 0.0F;
            s_debounce_timer = 0.0F;

            /* Any threat >= WARNING enters WARNING first */
            if ((desired == AEB_WARNING)  ||
                (desired == AEB_BRAKE_L1) ||
                (desired == AEB_BRAKE_L2) ||
                (desired == AEB_BRAKE_L3))
            {
                transition_to(AEB_WARNING);
            }
            break;

        case AEB_WARNING:
            s_warning_accum += dt;

            if (desired == AEB_STANDBY)
            {
                /* De-escalation with 200 ms hysteresis */
                s_debounce_timer += dt;
                if (s_debounce_timer >= HYSTERESIS_TIME)
                {
                    transition_to(AEB_STANDBY);
                    s_warning_accum  = 0.0F;
                    s_debounce_timer = 0.0F;
                }
            }
            else
            {
                s_debounce_timer = 0.0F;

                /* Escalate to braking only after minimum 800 ms warning */
                if (s_warning_accum >= WARNING_TO_BRAKE_MIN)
                {
                    if (desired == AEB_BRAKE_L3)
                    {
                        transition_to(AEB_BRAKE_L3);
                    }
                    else if (desired == AEB_BRAKE_L2)
                    {
                        transition_to(AEB_BRAKE_L2);
                    }
                    else if (desired == AEB_BRAKE_L1)
                    {
                        transition_to(AEB_BRAKE_L1);
                    }
                    else
                    {
                        /* Stay in WARNING */
                    }
                }
            }
            break;

        case AEB_BRAKE_L1:
            /* Vehicle stopped -> POST_BRAKE */
            if (perception->v_ego < 0.01F)
            {
                transition_to(AEB_POST_BRAKE);
            }
            /* Immediate escalation */
            else if ((desired == AEB_BRAKE_L2) || (desired == AEB_BRAKE_L3))
            {
                s_debounce_timer = 0.0F;
                transition_to(desired);
            }
            /* De-escalation with hysteresis */
            else if ((desired == AEB_STANDBY) || (desired == AEB_WARNING))
            {
                s_debounce_timer += dt;
                if (s_debounce_timer >= HYSTERESIS_TIME)
                {
                    transition_to(AEB_WARNING);
                    s_debounce_timer = 0.0F;
                    s_warning_accum  = 0.0F;
                }
            }
            else
            {
                s_debounce_timer = 0.0F;
            }
            break;

        case AEB_BRAKE_L2:
            if (perception->v_ego < 0.01F)
            {
                transition_to(AEB_POST_BRAKE);
            }
            /* Immediate escalation to L3 */
            else if (desired == AEB_BRAKE_L3)
            {
                s_debounce_timer = 0.0F;
                transition_to(AEB_BRAKE_L3);
            }
            /* De-escalation with hysteresis */
            else if ((desired == AEB_STANDBY) || (desired == AEB_WARNING) ||
                     (desired == AEB_BRAKE_L1))
            {
                s_debounce_timer += dt;
                if (s_debounce_timer >= HYSTERESIS_TIME)
                {
                    transition_to(AEB_BRAKE_L1);
                    s_debounce_timer = 0.0F;
                }
            }
            else
            {
                s_debounce_timer = 0.0F;
            }
            break;

        case AEB_BRAKE_L3:
            if (perception->v_ego < 0.01F)
            {
                transition_to(AEB_POST_BRAKE);
            }
            else if (desired != AEB_BRAKE_L3)
            {
                /* De-escalation with hysteresis */
                s_debounce_timer += dt;
                if (s_debounce_timer >= HYSTERESIS_TIME)
                {
                    if ((desired == AEB_BRAKE_L2) || (desired == AEB_BRAKE_L1))
                    {
                        transition_to(desired);
                    }
                    else
                    {
                        transition_to(AEB_POST_BRAKE);
                    }
                    s_debounce_timer = 0.0F;
                }
            }
            else
            {
                s_debounce_timer = 0.0F;
            }
            break;

        case AEB_POST_BRAKE:
            /* Hold for 2 s then return to STANDBY */
            if (s_state_timer >= POST_BRAKE_HOLD)
            {
                s_state          = AEB_STANDBY;
                s_state_timer    = 0.0F;
                s_warning_accum  = 0.0F;
                s_debounce_timer = 0.0F;
            }
            break;

        default:
            /* Defensive: unexpected state -> STANDBY */
            s_state          = AEB_STANDBY;
            s_state_timer    = 0.0F;
            s_warning_accum  = 0.0F;
            break;
    }

    return build_output(s_state);
}

/* ---- Build the FSMOutput_t from the given state ------------------------ */
static FSMOutput_t build_output(AEB_State_t state)
{
    FSMOutput_t out = { AEB_STANDBY, 0.0F, 0U, 0U, 0U };
    out.state = state;

    switch (state)
    {
        case AEB_WARNING:
            out.target_decel  = 0.0F;
            out.alert_visual  = 1U;
            out.alert_audible = 1U;
            out.brake_active  = 0U;
            break;

        case AEB_BRAKE_L1:
            out.target_decel  = DECEL_L1;
            out.alert_visual  = 1U;
            out.alert_audible = 1U;
            out.brake_active  = 1U;
            break;

        case AEB_BRAKE_L2:
            out.target_decel  = DECEL_L2;
            out.alert_visual  = 1U;
            out.alert_audible = 1U;
            out.brake_active  = 1U;
            break;

        case AEB_BRAKE_L3:
            out.target_decel  = DECEL_L3;
            out.alert_visual  = 1U;
            out.alert_audible = 1U;
            out.brake_active  = 1U;
            break;

        case AEB_POST_BRAKE:
            out.target_decel  = DECEL_L3;  /* FR-BRK-005: hold > 50% for 2 s after stop */
            out.alert_visual  = 1U;
            out.alert_audible = 0U;
            out.brake_active  = 1U;
            break;

        case AEB_OFF:      /* fall-through */
        case AEB_STANDBY:  /* fall-through */
        default:
            out.target_decel  = 0.0F;
            out.alert_visual  = 0U;
            out.alert_audible = 0U;
            out.brake_active  = 0U;
            break;
    }

    return out;
}
