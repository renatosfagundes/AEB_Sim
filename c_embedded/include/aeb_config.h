/**
 * @file aeb_config.h
 * @brief Calibration parameters for the AEB system.
 *
 * All tuneable constants are collected here so that calibration changes
 * require recompilation of this single header only.
 */

#ifndef AEB_CONFIG_H
#define AEB_CONFIG_H

/* ---- Simulation / scheduler -------------------------------------------- */
#define SIM_DT_CONTROLLER       0.01f   /**< Controller cycle time [s].     */

/* ---- Vehicle parameters ------------------------------------------------ */
#define VEHICLE_MASS            1500.0f /**< Vehicle mass [kg].              */

/* ---- Brake actuator model ---------------------------------------------- */
#define BRAKE_TAU               0.05f   /**< Brake time constant [s].       */
#define BRAKE_DEAD_TIME         0.03f   /**< Brake dead time [s].           */
#define BRAKE_MAX_DECEL         10.0f   /**< Maximum deceleration [m/s^2].  */

/* ---- TTC thresholds ---------------------------------------------------- */
#define TTC_WARNING             4.0f    /**< TTC for warning [s].           */
#define TTC_BRAKE_L1            3.0f    /**< TTC for brake level 1 [s].     */
#define TTC_BRAKE_L2            2.2f    /**< TTC for brake level 2 [s].     */
#define TTC_BRAKE_L3            1.8f    /**< TTC for brake level 3 [s].     */

/* ---- Deceleration demand per level ------------------------------------- */
#define DECEL_L1                2.0f    /**< Decel for level 1 [m/s^2].     */
#define DECEL_L2                4.0f    /**< Decel for level 2 [m/s^2].     */
#define DECEL_L3                6.0f    /**< Decel for level 3 [m/s^2].     */

/* ---- Ego speed validity window ----------------------------------------- */
#define V_EGO_MIN               1.39f   /**< Min ego speed for AEB [m/s] (5 km/h).  */
#define V_EGO_MAX               16.67f  /**< Max ego speed for AEB [m/s] (60 km/h). */

/* ---- Distance braking floor -------------------------------------------- */
/* Prevents TTC-based de-escalation while still closing on a target.
 * Applied only when is_closing == 1, so moving targets release naturally
 * when v_rel drops to zero — the ego will not continue braking to a stop. */
#define D_BRAKE_L1              20.0f   /**< Distance floor for L1 braking [m]. */
#define D_BRAKE_L2              10.0f   /**< Distance floor for L2 braking [m]. */
#define D_BRAKE_L3               5.0f   /**< Distance floor for L3 braking [m]. */

/* ---- Hysteresis -------------------------------------------------------- */
#define HYSTERESIS_TIME         0.2f    /**< State hold time [s].           */
#define TTC_HYSTERESIS          0.15f   /**< TTC hysteresis band [s].       */

/* ---- Driver override --------------------------------------------------- */
#define STEERING_OVERRIDE_DEG   5.0f    /**< Steering override angle [deg]. */

/* ---- Post-brake / warning timing --------------------------------------- */
#define POST_BRAKE_HOLD         2.0f    /**< Post-brake hold time [s].      */
#define WARNING_TO_BRAKE_MIN    0.8f    /**< Min warning before brake [s].  */

/* ---- PID controller ---------------------------------------------------- */
#define PID_KP                  10.0f   /**< Proportional gain.             */
#define PID_KI                  0.05f   /**< Integral gain.                 */
#define PID_OUTPUT_MAX          100.0f  /**< PID output upper limit [%].    */
#define PID_OUTPUT_MIN          0.0f    /**< PID output lower limit [%].    */

/* ---- Jerk limit -------------------------------------------------------- */
/* Real vehicle comfort target: 10 m/s^3.
 * At 10ms cycle with 100% / 10 m/s^2 scaling:
 *   max_delta = MAX_JERK * (100/10) * 0.01 = MAX_JERK * 0.1  [% / cycle]
 * 10  m/s^3 →  1 %/cycle → 60 % in 600 ms (too slow for AEB)
 * 100 m/s^3 → 10 %/cycle → 60 % in  60 ms (suitable for emergency braking) */
#define MAX_JERK                100.0f  /**< Maximum jerk [m/s^3].          */

/* ---- Sensor fault detection -------------------------------------------- */
#define SENSOR_FAULT_CYCLES     3       /**< Consecutive fault cycles.      */

/* ---- Sensor range validity --------------------------------------------- */
#define RANGE_MIN               0.5f    /**< Minimum valid range [m].       */
#define RANGE_MAX               300.0f  /**< Maximum valid range [m].       */

/* ---- TTC computation limits -------------------------------------------- */
#define V_REL_MIN               0.5f    /**< Min relative speed [m/s].      */
#define TTC_MAX                 10.0f   /**< TTC saturation value [s].      */

#endif /* AEB_CONFIG_H */
