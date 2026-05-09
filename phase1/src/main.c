#include <stdint.h>
#include "imu_source.h"
#include "dummy_generator.h"
#include "fusion_filter.h"
#include "classifier.h"
#include "alert.h"
#include "telemetry.h"
#include "hal_time.h"

/* Main loop period in milliseconds. 50 ms = 20 Hz, plenty for posture
 * detection and gentle on the buzzer cadence. */
#define MAIN_LOOP_PERIOD_MS  50u

/* Same period expressed in seconds for the filter. */
#define MAIN_LOOP_DT_SEC     (MAIN_LOOP_PERIOD_MS / 1000.0f)

/* Number of iterations to run before exiting. Set to 0 to run forever
 * on the device; non-zero so the host build terminates cleanly under
 * the test script. The Makefile's host target overrides this via
 * -DPHASE1_MAIN_ITERATIONS=N. */
#ifndef PHASE1_MAIN_ITERATIONS
#  define PHASE1_MAIN_ITERATIONS  0u
#endif

/* Cycle through dummy scenarios so the test harness sees variety.
 * Pure host-build convenience; on the device this is dead code. */
static void rotate_scenario(uint32_t iter) {
    static const DummyScenario cycle[DUMMY_SCENARIO_COUNT] = {
        SCENARIO_GOOD, SCENARIO_MILD_SLOUCH, SCENARIO_FULL_SLOUCH
    };
    /* Switch every 20 ticks (1 second at 50 ms loop). */
    dummy_set_scenario(cycle[(iter / 20u) % DUMMY_SCENARIO_COUNT]);
}

int main(void) {
    /* --- Init ---------------------------------------------------------- */
    imu_source_init();
    fusion_filter_reset();
    alert_init();
    telemetry_init();

    /* --- Loop ---------------------------------------------------------- */
    /* Volatile copy of the macro stops the compiler from constant-folding
     * the loop-exit comparison when PHASE1_MAIN_ITERATIONS == 0u, which
     * would otherwise trip -Wtype-limits under -Werror. */
    const volatile uint32_t max_iter = PHASE1_MAIN_ITERATIONS;
    uint32_t iter = 0u;
    for (;;) {
        rotate_scenario(iter);

        ImuRaw lower;
        ImuRaw upper;
        if (imu_source_read(&lower, &upper)) {
            float angle = fusion_filter_update(&lower, &upper, MAIN_LOOP_DT_SEC);
            PostureState state = classifier_classify(angle);
            alert_update(state);
            telemetry_send(angle, state);
        }

        sleep_ms(MAIN_LOOP_PERIOD_MS);

        iter++;
        if (max_iter != 0u && iter >= max_iter) {
            break;
        }
    }

    return 0;
}
