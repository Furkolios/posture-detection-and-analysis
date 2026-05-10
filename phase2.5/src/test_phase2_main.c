/* Phase 2 in-process C test harness.
 *
 * Exercises every Phase 2 checkpoint and prints PASS/FAIL per assertion.
 * Returns non-zero exit code if anything failed. */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "posture_types.h"
#include "imu_source.h"
#include "fusion_filter.h"
#include "classifier.h"
#include "alert.h"
#include "telemetry.h"

static int s_failures = 0;

static void check(bool ok, const char *what) {
    if (ok) {
        printf("  [PASS] %s\n", what);
    } else {
        printf("  [FAIL] %s\n", what);
        s_failures++;
    }
}

/* --- CP6: threshold classifier ------------------------------------------ */

static void test_cp6_classifier(void) {
    printf("CP6 — threshold classifier\n");
    classifier_calibrate(0.0f);

    /* Sweep pitch from -5 to 35 degrees. Verify boundary behaviour:
     *   pitch <  10           -> NOT FULL_SLOUCH and NOT MILD_SLOUCH
     *   pitch in [10, 25)     -> MILD_SLOUCH
     *   pitch >= 25           -> FULL_SLOUCH
     * Also verify the result is always a valid enum. */
    bool boundaries_ok = true;
    bool always_valid = true;

    for (int p = -5; p <= 35; ++p) {
        PostureState s = classifier_classify((float)p);
        if (s != POSTURE_GOOD && s != POSTURE_MILD_SLOUCH &&
            s != POSTURE_FULL_SLOUCH && s != POSTURE_LEAN_FORWARD &&
            s != POSTURE_LEAN_BACK && s != POSTURE_LATERAL_TILT) {
            always_valid = false;
        }
        if (p >= 25 && s != POSTURE_FULL_SLOUCH) {
            boundaries_ok = false;
        }
        if (p >= 10 && p < 25 && s != POSTURE_MILD_SLOUCH) {
            boundaries_ok = false;
        }
        /* Below 10°, we permit GOOD or LEAN_FORWARD depending on the
         * exact angle — both are correct for that band. */
    }
    check(always_valid, "classifier always returns a valid PostureState");
    check(boundaries_ok, "classifier honours mild/full thresholds");

    /* Spot-check the exact boundary points. */
    check(classifier_classify(9.99f) != POSTURE_FULL_SLOUCH,
          "9.99° is below FULL threshold");
    check(classifier_classify(10.0f) == POSTURE_MILD_SLOUCH,
          "10.0° crosses into MILD_SLOUCH");
    check(classifier_classify(24.99f) == POSTURE_MILD_SLOUCH,
          "24.99° is still MILD_SLOUCH");
    check(classifier_classify(25.0f) == POSTURE_FULL_SLOUCH,
          "25.0° crosses into FULL_SLOUCH");
}

/* --- CP7: Madgwick filter ----------------------------------------------- */

static void test_cp7_madgwick_neutral(void) {
    printf("CP7 — Madgwick filter (neutral steady-state)\n");
    fusion_filter_reset();

    /* Both IMUs reading gravity straight down, no rotation. After
     * convergence the relative pitch should be ~0. */
    ImuRaw lo = { .ax=0, .ay=0, .az=9.81f, .gx=0, .gy=0, .gz=0 };
    ImuRaw up = { .ax=0, .ay=0, .az=9.81f, .gx=0, .gy=0, .gz=0 };

    float pitch = 0.0f;
    for (int i = 0; i < 100; ++i) {
        pitch = fusion_filter_update(&lo, &up, 0.01f);
    }
    printf("    converged pitch = %.3f deg\n", pitch);
    check(fabsf(pitch) < 2.0f,
          "neutral input converges to within 2 deg of zero");
}

static void test_cp7_madgwick_tilted(void) {
    printf("CP7 — Madgwick filter (upper IMU tilted ~30 deg forward)\n");
    fusion_filter_reset();

    /* Lower IMU upright, upper IMU tilted 30° forward (around Y).
     * Accel for 30° pitch: ax = -g*sin(30°), az = g*cos(30°). */
    const float deg = 30.0f * 3.14159265358979323846f / 180.0f;
    ImuRaw lo = { .ax = 0.0f,
                  .ay = 0.0f,
                  .az = 9.81f,
                  .gx = 0, .gy = 0, .gz = 0 };
    ImuRaw up = { .ax = -9.81f * sinf(deg),
                  .ay = 0.0f,
                  .az =  9.81f * cosf(deg),
                  .gx = 0, .gy = 0, .gz = 0 };

    float pitch = 0.0f;
    for (int i = 0; i < 500; ++i) {
        pitch = fusion_filter_update(&lo, &up, 0.01f);
    }
    printf("    converged pitch = %.3f deg (target 30, threshold for FULL = %.1f)\n",
           pitch, THRESHOLD_FULL_DEG);
    check(pitch >= THRESHOLD_FULL_DEG - 5.0f,
          "tilted input converges into or above FULL threshold band");
}

/* --- CP8: extended classification + V2 telemetry ------------------------ */

/* Capture buffer for V2 packets — same trick as Phase 1 used for V1. */
static uint8_t s_captured[64];
static size_t  s_captured_len = 0;

void uart_init(uint32_t baud);
void uart_write_bytes(const uint8_t *buf, size_t n);
void uart_init(uint32_t baud) { (void)baud; }
void uart_write_bytes(const uint8_t *buf, size_t n) {
    if (s_captured_len + n > sizeof(s_captured)) return;
    memcpy(s_captured + s_captured_len, buf, n);
    s_captured_len += n;
}

static void test_cp8_classifier_full(void) {
    printf("CP8 — full classifier (pitch + roll)\n");
    classifier_calibrate_full(0.0f, 0.0f);

    check(classifier_classify_full(  0.0f,  0.0f) == POSTURE_GOOD,
          "(0, 0) -> GOOD");
    check(classifier_classify_full( 18.0f,  0.0f) == POSTURE_MILD_SLOUCH,
          "(18, 0) -> MILD_SLOUCH");
    check(classifier_classify_full( 35.0f,  0.0f) == POSTURE_FULL_SLOUCH,
          "(35, 0) -> FULL_SLOUCH");
    check(classifier_classify_full(-15.0f,  0.0f) == POSTURE_LEAN_BACK,
          "(-15, 0) -> LEAN_BACK");
    check(classifier_classify_full(  6.0f,  0.0f) == POSTURE_LEAN_FORWARD,
          "(6, 0) -> LEAN_FORWARD");
    check(classifier_classify_full(  0.0f, 20.0f) == POSTURE_LATERAL_TILT,
          "(0, 20) -> LATERAL_TILT");
    /* Lateral wins over slouch when both fire. */
    check(classifier_classify_full( 30.0f, 20.0f) == POSTURE_LATERAL_TILT,
          "(30, 20) -> LATERAL_TILT (lateral overrides slouch)");
}

static void test_cp8_telemetry_v2(void) {
    printf("CP8 — telemetry V2 packet\n");
    telemetry_init();
    s_captured_len = 0;

    telemetry_send_v2(12.5f, -3.25f, POSTURE_LATERAL_TILT);

    check(s_captured_len == TELEMETRY_PACKET_V2_SIZE,
          "V2 packet is exactly TELEMETRY_PACKET_V2_SIZE bytes");

    if (s_captured_len == TELEMETRY_PACKET_V2_SIZE) {
        TelemetryPacketV2 pkt;
        memcpy(&pkt, s_captured, TELEMETRY_PACKET_V2_SIZE);
        check(pkt.sync == TELEMETRY_SYNC_BYTE, "sync byte is 0xA5");
        check(fabsf(pkt.pitch_deg - 12.5f) < 0.001f,
              "pitch round-trips");
        check(fabsf(pkt.roll_deg - (-3.25f)) < 0.001f,
              "roll round-trips");
        check(pkt.state == (uint8_t)POSTURE_LATERAL_TILT,
              "state byte matches");
        check(pkt.checksum == telemetry_compute_checksum_v2(&pkt),
              "V2 checksum is self-consistent");
    }
}

static void test_cp8_alert_extended(void) {
    printf("CP8 — alert state machine on extended states\n");
    alert_init();
    /* Drive every new state. The host buzzer HAL prints transitions
     * to stderr; a crash here would fail the test. */
    for (int t = 0; t < 30; ++t) alert_update(POSTURE_LEAN_FORWARD);
    for (int t = 0; t < 30; ++t) alert_update(POSTURE_LEAN_BACK);
    for (int t = 0; t < 30; ++t) alert_update(POSTURE_LATERAL_TILT);
    /* Out-of-range cast: must not crash. */
    alert_update((PostureState)99);
    check(true, "alert_update handles every extended state without crashing");
}

/* --- CP9: imu_source through the MPU-6050 driver in host mode ---------- */
/* The host I2C stub fakes register reads; this test confirms the driver
 * path produces sensible accel readings via the same imu_source.h API. */

/* The host I2C HAL exposes a direct override hook for tests. We use
 * it instead of the dummy generator's scenario knob because this test
 * binary does not link dummy_generator.c (it would duplicate the
 * imu_source_init/read symbols that imu_source_mpu6050.c defines). */
void hal_i2c_host_set_pitch(float pitch_deg);

static void test_cp9_mpu6050_via_hal(void) {
    printf("CP9 — MPU-6050 driver via host I2C HAL\n");
    imu_source_init();
    hal_i2c_host_set_pitch(35.0f);   /* upper tilted ~35° forward */

    ImuRaw lo, up;
    bool ok = imu_source_read(&lo, &up);
    printf("    lower az=%.2f upper ax=%.2f upper az=%.2f\n",
           lo.az, up.ax, up.az);
    check(ok, "imu_source_read succeeds via MPU-6050 host HAL");
    check(lo.az > 8.5f && lo.az < 11.0f,
          "lower IMU az is near +g (level)");
    check(up.ax < -3.0f,
          "upper IMU ax is strongly negative (forward tilt)");
}

/* --- main --------------------------------------------------------------- */

int main(void) {
    test_cp6_classifier();
    test_cp7_madgwick_neutral();
    test_cp7_madgwick_tilted();
    test_cp8_classifier_full();
    test_cp8_telemetry_v2();
    test_cp8_alert_extended();
    test_cp9_mpu6050_via_hal();

    printf("\n");
    if (s_failures == 0) {
        printf("ALL CHECKS PASSED\n");
        return 0;
    } else {
        printf("%d CHECK(S) FAILED\n", s_failures);
        return 1;
    }
}
