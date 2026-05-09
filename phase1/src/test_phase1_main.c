/* Phase 1 in-process C test harness.
 *
 * Exercises each module independently and prints pass/fail for each
 * group of checks. Returns a non-zero exit code if anything failed.
 *
 * Build with the Makefile's `make test` target. */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "posture_types.h"
#include "imu_source.h"
#include "dummy_generator.h"
#include "fusion_filter.h"
#include "classifier.h"
#include "alert.h"
#include "telemetry.h"

#define CLASSIFIER_TRIALS         100
#define FILTER_SETTLE_ITERATIONS  200
#define FILTER_DT_SEC             0.05f
#define FILTER_TOLERANCE_DEG      5.0f

static int s_failures = 0;

static void check(bool ok, const char *what) {
    if (ok) {
        printf("  [PASS] %s\n", what);
    } else {
        printf("  [FAIL] %s\n", what);
        s_failures++;
    }
}

/* --- CP1: dummy generator -------------------------------------------- */

static void test_dummy_generator(void) {
    printf("CP1 — dummy generator\n");
    imu_source_init();

    const DummyScenario scenarios[] = {
        SCENARIO_GOOD, SCENARIO_MILD_SLOUCH, SCENARIO_FULL_SLOUCH
    };
    const char *names[] = { "GOOD", "MILD_SLOUCH", "FULL_SLOUCH" };

    for (size_t i = 0; i < 3; ++i) {
        dummy_set_scenario(scenarios[i]);
        ImuRaw lo;
        ImuRaw up;
        bool ok = imu_source_read(&lo, &up);
        printf("    scenario=%s target_angle=%.1f deg "
               "lower.az=%.2f upper.ax=%.2f upper.az=%.2f\n",
               names[i], dummy_get_target_angle(), lo.az, up.ax, up.az);
        check(ok, "imu_source_read returns true");
        /* az should be near +g for the level lower IMU. */
        check(lo.az > 8.5f && lo.az < 11.0f,
              "lower IMU az is near +g");
    }
}

/* --- CP2: complementary filter --------------------------------------- */

static void test_fusion_filter(void) {
    printf("CP2 — fusion filter\n");

    const DummyScenario scenarios[] = {
        SCENARIO_GOOD, SCENARIO_MILD_SLOUCH, SCENARIO_FULL_SLOUCH
    };

    for (size_t i = 0; i < 3; ++i) {
        imu_source_init();        /* re-seed RNG */
        fusion_filter_reset();
        dummy_set_scenario(scenarios[i]);

        float fused = 0.0f;
        for (int k = 0; k < FILTER_SETTLE_ITERATIONS; ++k) {
            ImuRaw lo;
            ImuRaw up;
            imu_source_read(&lo, &up);
            fused = fusion_filter_update(&lo, &up, FILTER_DT_SEC);
        }
        float target = dummy_get_target_angle();
        float err = fabsf(fused - target);
        printf("    target=%.1f deg fused=%.1f deg err=%.2f deg\n",
               target, fused, err);
        check(err < FILTER_TOLERANCE_DEG,
              "fused angle within tolerance of scenario target");
    }
}

/* --- CP3: stub classifier -------------------------------------------- */

static void test_classifier(void) {
    printf("CP3 — stub classifier\n");
    bool all_valid = true;
    int counts[POSTURE_STATE_COUNT] = {0, 0, 0};
    for (int i = 0; i < CLASSIFIER_TRIALS; ++i) {
        PostureState s = classifier_classify((float)i);
        if (s != POSTURE_GOOD &&
            s != POSTURE_MILD_SLOUCH &&
            s != POSTURE_FULL_SLOUCH) {
            all_valid = false;
            break;
        }
        counts[s]++;
    }
    printf("    distribution over %d trials: GOOD=%d MILD=%d FULL=%d\n",
           CLASSIFIER_TRIALS, counts[0], counts[1], counts[2]);
    check(all_valid, "all classifier outputs are valid enum members");
}

/* --- CP4: alert state machine ---------------------------------------- */

static void test_alert(void) {
    printf("CP4 — alert state machine\n");
    alert_init();
    /* Just exercise each branch; the host buzzer HAL prints transitions
     * to stderr so a human can sanity-check, and a crash here would
     * fail the test. */
    for (int t = 0; t < 30; ++t) alert_update(POSTURE_GOOD);
    for (int t = 0; t < 30; ++t) alert_update(POSTURE_MILD_SLOUCH);
    for (int t = 0; t < 30; ++t) alert_update(POSTURE_FULL_SLOUCH);
    /* Defensive: out-of-range cast must not crash. */
    alert_update((PostureState)42);
    check(true, "alert_update did not crash on any input");
}

/* --- CP5: telemetry --------------------------------------------------- */

/* The host UART HAL writes to stdout, which we don't want polluted
 * during the harness run. Override write here by linking against a
 * tiny capture buffer instead — done by compiling this file with a
 * special define and supplying our own uart_write_bytes. See Makefile
 * test target. */

static uint8_t s_captured[256];
static size_t  s_captured_len = 0;

/* Replacement for the host HAL's uart_write_bytes. We compile this
 * file with -DPHASE1_TEST_OVERRIDES_UART so the linker picks ours
 * instead of the one in hal_uart_host.c. */
void uart_write_bytes(const uint8_t *buf, size_t n);
void uart_init(uint32_t baud);

void uart_init(uint32_t baud) { (void)baud; }
void uart_write_bytes(const uint8_t *buf, size_t n) {
    if (s_captured_len + n > sizeof(s_captured)) {
        return;  /* drop overflow; not expected in this harness */
    }
    memcpy(s_captured + s_captured_len, buf, n);
    s_captured_len += n;
}

static void test_telemetry(void) {
    printf("CP5 — telemetry\n");
    telemetry_init();
    s_captured_len = 0;

    telemetry_send(12.5f, POSTURE_MILD_SLOUCH);

    check(s_captured_len == TELEMETRY_PACKET_SIZE,
          "telemetry_send wrote exactly TELEMETRY_PACKET_SIZE bytes");

    if (s_captured_len == TELEMETRY_PACKET_SIZE) {
        TelemetryPacket pkt;
        memcpy(&pkt, s_captured, TELEMETRY_PACKET_SIZE);
        check(pkt.sync == TELEMETRY_SYNC_BYTE, "sync byte is 0xA5");
        check(fabsf(pkt.angle_deg - 12.5f) < 0.001f,
              "angle round-trips through the wire format");
        check(pkt.state == (uint8_t)POSTURE_MILD_SLOUCH,
              "state byte matches the value we sent");
        check(pkt.checksum == telemetry_compute_checksum(&pkt),
              "checksum is self-consistent");
    }
}

/* --- main ------------------------------------------------------------ */

int main(void) {
    test_dummy_generator();
    test_fusion_filter();
    test_classifier();
    test_alert();
    test_telemetry();

    printf("\n");
    if (s_failures == 0) {
        printf("ALL CHECKS PASSED\n");
        return 0;
    } else {
        printf("%d CHECK(S) FAILED\n", s_failures);
        return 1;
    }
}
