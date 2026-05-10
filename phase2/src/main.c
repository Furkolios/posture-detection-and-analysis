#include <stdint.h>
#include "imu_source.h"
#include "dummy_generator.h"
#include "fusion_filter.h"
#include "classifier.h"
#include "alert.h"
#include "telemetry.h"
#include "hal_time.h"

/* Main loop period in milliseconds. 50 ms = 20 Hz. */
#define MAIN_LOOP_PERIOD_MS  50u
#define MAIN_LOOP_DT_SEC     (MAIN_LOOP_PERIOD_MS / 1000.0f)

/* Number of Madgwick steps to settle through during startup
 * calibration. At 50 ms loop, 40 ticks = 2 seconds — enough for
 * the filter to converge from identity to gravity-aligned. */
#define CALIBRATION_TICKS    40u

/* Iteration cap for host builds. Set to 0 to run forever (device). */
#ifndef PHASE2_MAIN_ITERATIONS
#  define PHASE2_MAIN_ITERATIONS  0u
#endif

/* Cycle through dummy scenarios so the host validator sees variety.
 * Dead code on the device build (the dummy generator isn't linked). */
static void rotate_scenario(uint32_t iter) {
    static const DummyScenario cycle[DUMMY_SCENARIO_COUNT] = {
        SCENARIO_GOOD, SCENARIO_MILD_SLOUCH, SCENARIO_FULL_SLOUCH
    };
    /* Switch every 20 ticks (1 second at 50 ms loop). */
    dummy_set_scenario(cycle[(iter / 20u) % DUMMY_SCENARIO_COUNT]);
}

/* Run the filter against current sensor input for a few ticks, then
 * snapshot the resulting pitch/roll as the user's neutral baseline.
 * Caller is responsible for telling the user to "stand still" first
 * — for now we just do it at boot.
 *
 * We DO emit telemetry during calibration (with state = GOOD and the
 * filter's still-converging angles) so downstream consumers see a
 * continuous packet stream from the moment the device powers on. */
static void run_calibration(void) {
    fusion_filter_reset();
    dummy_set_scenario(SCENARIO_GOOD);

    for (uint32_t i = 0u; i < CALIBRATION_TICKS; ++i) {
        ImuRaw lower;
        ImuRaw upper;
        if (imu_source_read(&lower, &upper)) {
            float pitch = fusion_filter_update(&lower, &upper, MAIN_LOOP_DT_SEC);
            float roll  = 0.0f;
            fusion_filter_get_angles(&pitch, &roll);
#ifdef USE_TELEMETRY_V1
            telemetry_send(pitch, POSTURE_GOOD);
#else
            telemetry_send_v2(pitch, roll, POSTURE_GOOD);
#endif
        }
        sleep_ms(MAIN_LOOP_PERIOD_MS);
    }
    float pitch_neutral = 0.0f;
    float roll_neutral  = 0.0f;
    fusion_filter_get_angles(&pitch_neutral, &roll_neutral);
    classifier_calibrate_full(pitch_neutral, roll_neutral);
}

int main(void) {
    /* --- Init ---------------------------------------------------------- */
    imu_source_init();
    alert_init();
    telemetry_init();
    run_calibration();

    /* --- Loop ---------------------------------------------------------- */
    const volatile uint32_t max_iter = PHASE2_MAIN_ITERATIONS;
    uint32_t iter = 0u;
    for (;;) {
        rotate_scenario(iter);

        ImuRaw lower;
        ImuRaw upper;
        if (imu_source_read(&lower, &upper)) {
            float pitch = fusion_filter_update(&lower, &upper, MAIN_LOOP_DT_SEC);
            float roll  = 0.0f;
            fusion_filter_get_angles(&pitch, &roll);
            PostureState state = classifier_classify_full(pitch, roll);
            alert_update(state);
#ifdef USE_TELEMETRY_V1
            /* Legacy emitter for Phase 1 pytest compatibility. Phase 1
             * decoders only know GOOD/MILD/FULL, so we collapse the
             * three Phase 2 additions onto the closest V1 state. */
            PostureState legacy_state = state;
            if (legacy_state == POSTURE_LEAN_FORWARD)
                legacy_state = POSTURE_MILD_SLOUCH;
            else if (legacy_state == POSTURE_LEAN_BACK ||
                     legacy_state == POSTURE_LATERAL_TILT)
                legacy_state = POSTURE_GOOD;
            telemetry_send(pitch, legacy_state);
#else
            telemetry_send_v2(pitch, roll, state);
#endif
        }

        sleep_ms(MAIN_LOOP_PERIOD_MS);

        iter++;
        if (max_iter != 0u && iter >= max_iter) {
            break;
        }
    }

    return 0;
}
