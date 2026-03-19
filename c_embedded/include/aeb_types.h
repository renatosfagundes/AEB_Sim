/**
 * @file aeb_types.h
 * @brief Common data types and enumerations for the AEB system.
 */

#ifndef AEB_TYPES_H
#define AEB_TYPES_H

#include <stdint.h>

/** @brief 32-bit IEEE-754 floating-point type. */
typedef float float32_t;

/**
 * @brief AEB finite-state-machine states.
 */
typedef enum
{
    AEB_OFF        = 0,   /**< System disabled.                       */
    AEB_STANDBY    = 1,   /**< Monitoring, no threat detected.        */
    AEB_WARNING    = 2,   /**< Collision warning issued to driver.    */
    AEB_BRAKE_L1   = 3,   /**< Autonomous braking level 1 (light).   */
    AEB_BRAKE_L2   = 4,   /**< Autonomous braking level 2 (medium).  */
    AEB_BRAKE_L3   = 5,   /**< Autonomous braking level 3 (full).    */
    AEB_POST_BRAKE = 6    /**< Post-braking hold / recovery phase.   */
} AEB_State_t;

/**
 * @brief Fused perception data provided to the AEB algorithm each cycle.
 */
typedef struct
{
    float32_t distance;        /**< Distance to target [m].              */
    float32_t v_ego;           /**< Ego-vehicle speed [m/s].             */
    float32_t v_target;        /**< Target-object speed [m/s].           */
    float32_t v_rel;           /**< Relative speed (v_ego - v_target) [m/s]. */
    uint8_t   brake_pedal;     /**< Driver brake-pedal flag (0/1).       */
    float32_t steering_angle;  /**< Steering-wheel angle [deg].          */
    uint8_t   fault;           /**< Sensor fault flag (0 = OK).          */
    float32_t confidence;      /**< Perception confidence [0.0 .. 1.0].  */
} PerceptionData_t;

/**
 * @brief Time-to-collision computation result.
 */
typedef struct
{
    float32_t ttc;        /**< Time to collision [s].                    */
    float32_t d_brake;    /**< Required braking distance [m].            */
    uint8_t   is_closing; /**< 1 if ego is closing on target, else 0.   */
} TTCResult_t;

/**
 * @brief Output of the AEB finite-state machine for a single cycle.
 */
typedef struct
{
    AEB_State_t state;         /**< Current FSM state.                   */
    float32_t   target_decel;  /**< Requested deceleration [m/s^2].     */
    uint8_t     alert_visual;  /**< Visual alert active flag (0/1).     */
    uint8_t     alert_audible; /**< Audible alert active flag (0/1).    */
    uint8_t     brake_active;  /**< Brake actuator active flag (0/1).   */
} FSMOutput_t;

#endif /* AEB_TYPES_H */
