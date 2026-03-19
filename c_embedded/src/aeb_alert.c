/**
 * @file aeb_alert.c
 * @brief Alert management for the AEB system.
 *
 * Sets visual and audible alert flags based on the current FSM state:
 *   - WARNING / BRAKE_L1..L3: visual + audible ON
 *   - POST_BRAKE:             visual ON, audible OFF
 *   - OFF / STANDBY:          all OFF
 *
 * MISRA C:2012 compliant: fixed-width types, no dynamic allocation,
 * no recursion, all variables initialised at declaration.
 */

#include "aeb_types.h"
#include "aeb_alert.h"

/* ---- Module-static state ----------------------------------------------- */
static uint8_t s_visual  = 0U;
static uint8_t s_audible = 0U;

/* ---- Public API -------------------------------------------------------- */

void alert_init(void)
{
    s_visual  = 0U;
    s_audible = 0U;
}

void alert_update(AEB_State_t state)
{
    switch (state)
    {
        case AEB_WARNING:   /* fall-through */
        case AEB_BRAKE_L1:  /* fall-through */
        case AEB_BRAKE_L2:  /* fall-through */
        case AEB_BRAKE_L3:
            s_visual  = 1U;
            s_audible = 1U;
            break;

        case AEB_POST_BRAKE:
            s_visual  = 1U;
            s_audible = 0U;
            break;

        case AEB_OFF:       /* fall-through */
        case AEB_STANDBY:   /* fall-through */
        default:
            s_visual  = 0U;
            s_audible = 0U;
            break;
    }
}

uint8_t alert_get_visual(void)
{
    return s_visual;
}

uint8_t alert_get_audible(void)
{
    return s_audible;
}
