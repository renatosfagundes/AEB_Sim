/**
 * @file test_aeb.c
 * @brief Basic unit tests for AEB system (SIL back-to-back validation)
 *
 * Tests cover:
 *   - TTC computation (FR-DEC-001)
 *   - Braking distance (FR-DEC-002)
 *   - FSM state transitions (FR-FSM-001..005)
 *   - PID controller response (FR-BRK-002)
 *   - Full 10ms cycle integration
 */
#include <stdio.h>
#include <math.h>
#include <assert.h>

#include "aeb_types.h"
#include "aeb_config.h"
#include "aeb_ttc.h"
#include "aeb_fsm.h"
#include "aeb_pid.h"
#include "aeb_main.h"

#define ASSERT_FLOAT_EQ(a, b, tol) \
    do { \
        if (fabsf((a) - (b)) > (tol)) { \
            printf("FAIL: %s:%d: %.4f != %.4f (tol=%.4f)\n", \
                   __FILE__, __LINE__, (double)(a), (double)(b), (double)(tol)); \
            failures++; \
        } \
    } while(0)

#define ASSERT_EQ(a, b) \
    do { \
        if ((a) != (b)) { \
            printf("FAIL: %s:%d: %d != %d\n", __FILE__, __LINE__, (int)(a), (int)(b)); \
            failures++; \
        } \
    } while(0)

static int failures = 0;

/* ---- TTC Tests ---- */
static void test_ttc_basic(void)
{
    printf("  test_ttc_basic... ");
    ttc_init();

    /* v_ego=40km/h=11.11m/s, v_target=0, d=50m -> TTC = 50/11.11 = 4.5s */
    TTCResult_t r = ttc_compute(50.0f, 11.11f, 0.0f);
    ASSERT_FLOAT_EQ(r.ttc, 4.5f, 0.1f);
    ASSERT_EQ(r.is_closing, 1U);

    /* d_brake = 11.11^2 / (2*6) = 10.29m */
    ASSERT_FLOAT_EQ(r.d_brake, 10.29f, 0.5f);

    printf("OK\n");
}

static void test_ttc_not_closing(void)
{
    printf("  test_ttc_not_closing... ");
    ttc_init();

    /* v_ego < v_target -> not closing */
    TTCResult_t r = ttc_compute(50.0f, 5.0f, 10.0f);
    ASSERT_EQ(r.is_closing, 0U);
    ASSERT_FLOAT_EQ(r.ttc, TTC_MAX, 0.01f);

    printf("OK\n");
}

/* ---- FSM Tests ---- */
static void test_fsm_standby_to_warning(void)
{
    printf("  test_fsm_standby_to_warning... ");
    fsm_init();

    /* Simulate TTC = 3.5s (should be WARNING) */
    TTCResult_t ttc = {3.5f, 10.0f, 1U};
    PerceptionData_t per = {50.0f, 11.11f, 0.0f, 11.11f, 0U, 0.0f, 0U, 1.0f};

    FSMOutput_t out = fsm_update(&ttc, &per, SIM_DT_CONTROLLER);
    ASSERT_EQ(out.state, AEB_WARNING);
    ASSERT_EQ(out.alert_visual, 1U);
    ASSERT_EQ(out.alert_audible, 1U);
    ASSERT_FLOAT_EQ(out.target_decel, 0.0f, 0.01f);

    printf("OK\n");
}

static void test_fsm_warning_to_brake(void)
{
    printf("  test_fsm_warning_to_brake... ");
    fsm_init();

    TTCResult_t ttc = {3.5f, 10.0f, 1U};
    PerceptionData_t per = {50.0f, 11.11f, 0.0f, 11.11f, 0U, 0.0f, 0U, 1.0f};

    /* Stay in WARNING for 800ms+ */
    int cycles_800ms = 80; /* 80 * 10ms = 800ms */
    FSMOutput_t out;
    for (int i = 0; i < cycles_800ms + 5; i++) {
        out = fsm_update(&ttc, &per, SIM_DT_CONTROLLER);
    }
    ASSERT_EQ(out.state, AEB_WARNING);

    /* Now drop TTC to 2.5s -> should go to BRAKE_L1 */
    ttc.ttc = 2.5f;
    out = fsm_update(&ttc, &per, SIM_DT_CONTROLLER);
    ASSERT_EQ(out.state, AEB_BRAKE_L1);
    ASSERT_FLOAT_EQ(out.target_decel, DECEL_BRAKE_L1, 0.01f);
    ASSERT_EQ(out.brake_active, 1U);

    printf("OK\n");
}

static void test_fsm_fault_goes_off(void)
{
    printf("  test_fsm_fault_goes_off... ");
    fsm_init();

    TTCResult_t ttc = {2.0f, 10.0f, 1U};
    PerceptionData_t per = {50.0f, 11.11f, 0.0f, 11.11f, 0U, 0.0f, 1U, 0.0f};

    /* Fault -> OFF */
    FSMOutput_t out = fsm_update(&ttc, &per, SIM_DT_CONTROLLER);
    ASSERT_EQ(out.state, AEB_OFF);

    printf("OK\n");
}

static void test_fsm_driver_override(void)
{
    printf("  test_fsm_driver_override... ");
    fsm_init();

    TTCResult_t ttc = {3.5f, 10.0f, 1U};
    PerceptionData_t per = {50.0f, 11.11f, 0.0f, 11.11f, 0U, 0.0f, 0U, 1.0f};

    /* Enter WARNING */
    FSMOutput_t out = fsm_update(&ttc, &per, SIM_DT_CONTROLLER);
    ASSERT_EQ(out.state, AEB_WARNING);

    /* Driver presses brake */
    per.brake_pedal = 1U;
    out = fsm_update(&ttc, &per, SIM_DT_CONTROLLER);
    ASSERT_EQ(out.state, AEB_STANDBY);

    printf("OK\n");
}

/* ---- PID Tests ---- */
static void test_pid_basic(void)
{
    printf("  test_pid_basic... ");
    pid_init();

    /* Target = 4 m/s^2, actual = 0 -> big positive error -> high output */
    float32_t cmd = pid_compute(4.0f, 0.0f, SIM_DT_CONTROLLER);
    /* Expect: P = 4*4 = 16, I = 0.05*4*0.01 = 0.002 -> ~16% */
    ASSERT_FLOAT_EQ(cmd, 16.0f, 2.0f);

    printf("OK\n");
}

/* ---- Full Cycle Test ---- */
static void test_full_cycle(void)
{
    printf("  test_full_cycle... ");
    aeb_init();

    /* Simulate: d=50m, v_ego=40km/h, v_target=0 -> TTC=4.5s -> WARNING */
    aeb_cycle_10ms(50.0f, 11.11f, 0.0f, 0U, 0.0f, 0.0f);
    ASSERT_EQ(aeb_get_state(), AEB_WARNING);
    ASSERT_FLOAT_EQ(aeb_get_brake_cmd(), 0.0f, 0.01f);

    printf("OK\n");
}

int main(void)
{
    printf("=== AEB Unit Tests ===\n");

    printf("\nTTC Calculator:\n");
    test_ttc_basic();
    test_ttc_not_closing();

    printf("\nFSM:\n");
    test_fsm_standby_to_warning();
    test_fsm_warning_to_brake();
    test_fsm_fault_goes_off();
    test_fsm_driver_override();

    printf("\nPID Controller:\n");
    test_pid_basic();

    printf("\nFull Cycle:\n");
    test_full_cycle();

    printf("\n=== Results: %d failure(s) ===\n", failures);
    return (failures > 0) ? 1 : 0;
}
