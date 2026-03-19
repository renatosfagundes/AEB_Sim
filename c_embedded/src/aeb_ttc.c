/**
 * @file aeb_ttc.c
 * @brief Time-to-collision and braking-distance computation.
 *
 * TTC = distance / v_rel   when v_rel > 0.5 m/s
 * d_brake = v_ego^2 / (2 * 6.0)   (max decel = DECEL_L3 = 6 m/s^2)
 *
 * MISRA C:2012 compliant: fixed-width types, no dynamic allocation,
 * no recursion, all variables initialised at declaration.
 */

#include "aeb_types.h"
#include "aeb_config.h"
#include "aeb_ttc.h"

/* ---- Module-static state ----------------------------------------------- */
static TTCResult_t s_result = {
    0.0F,   /* ttc        */
    0.0F,   /* d_brake    */
    0U      /* is_closing */
};

/* ------------------------------------------------------------------------ */
void ttc_init(void)
{
    s_result.ttc        = TTC_MAX;
    s_result.d_brake    = 0.0F;
    s_result.is_closing = 0U;
}

/* ------------------------------------------------------------------------ */
TTCResult_t ttc_compute(float32_t distance,
                        float32_t v_ego,
                        float32_t v_target)
{
    TTCResult_t result  = { TTC_MAX, 0.0F, 0U };
    float32_t   v_rel   = v_ego - v_target;

    /* ---- Is-closing flag: positive relative speed means closing --------- */
    if (v_rel > 0.0F)
    {
        result.is_closing = 1U;
    }
    else
    {
        result.is_closing = 0U;
    }

    /* ---- TTC = d / v_rel  (only when v_rel > V_REL_MIN = 0.5 m/s) ----- */
    if (v_rel > V_REL_MIN)
    {
        result.ttc = distance / v_rel;

        if (result.ttc < 0.0F)
        {
            result.ttc = 0.0F;
        }
        if (result.ttc > TTC_MAX)
        {
            result.ttc = TTC_MAX;
        }
    }
    else
    {
        result.ttc = TTC_MAX;
    }

    /* ---- Braking distance: d_brake = v^2 / (2 * 6)  ------------------- */
    result.d_brake = (v_ego * v_ego) / (2.0F * DECEL_L3);

    /* ---- Store for module query ---------------------------------------- */
    s_result = result;

    return result;
}
