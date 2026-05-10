#include <stdlib.h>
#include <math.h>
#include "dummy_generator.h"

/* --- Tunable scenario angles (degrees) ---------------------------------- *
 * These define what "good", "mild slouch", and "full slouch" look like for
 * the dummy data path. The real device replaces this whole module in
 * Phase 2 — these numbers don't ship to production. */
#define SCENARIO_GOOD_ANGLE_DEG          0.0f
#define SCENARIO_MILD_SLOUCH_ANGLE_DEG  18.0f
#define SCENARIO_FULL_SLOUCH_ANGLE_DEG  35.0f

/* Magnitude of small uniform noise added to each raw sample, so the
 * complementary filter sees something realistic rather than a constant. */
#define DUMMY_ACCEL_NOISE_MS2   0.05f
#define DUMMY_GYRO_NOISE_DPS    0.50f

/* Standard gravity, used to synthesise accelerometer readings consistent
 * with a static tilt. */
#define GRAVITY_MS2             9.81f

/* Conversion factor between degrees and radians. */
#define DEG_TO_RAD              (3.14159265358979323846f / 180.0f)

/* --- Module state -------------------------------------------------------- */
static DummyScenario s_scenario = SCENARIO_GOOD;

/* Map a scenario enum to its target angle in degrees. */
static float scenario_to_angle(DummyScenario s) {
    switch (s) {
        case SCENARIO_GOOD:        return SCENARIO_GOOD_ANGLE_DEG;
        case SCENARIO_MILD_SLOUCH: return SCENARIO_MILD_SLOUCH_ANGLE_DEG;
        case SCENARIO_FULL_SLOUCH: return SCENARIO_FULL_SLOUCH_ANGLE_DEG;
        default:                   return SCENARIO_GOOD_ANGLE_DEG;
    }
}

/* Uniform noise in [-magnitude, +magnitude]. rand() is fine here — this
 * is fake data for development, not crypto. */
static float uniform_noise(float magnitude) {
    float r = (float)rand() / (float)RAND_MAX;        /* [0, 1] */
    return (r * 2.0f - 1.0f) * magnitude;             /* [-m, +m] */
}

/* Synthesise a static-tilt IMU sample at the given pitch.
 * Accelerometer reads gravity rotated by the pitch angle; gyro reads ~0
 * (plus noise) because the sensor is not moving. */
static void synth_sample_at_pitch(float pitch_deg, ImuRaw *out) {
    float pitch_rad = pitch_deg * DEG_TO_RAD;
    out->ax = -GRAVITY_MS2 * sinf(pitch_rad) + uniform_noise(DUMMY_ACCEL_NOISE_MS2);
    out->ay =  uniform_noise(DUMMY_ACCEL_NOISE_MS2);
    out->az =  GRAVITY_MS2 * cosf(pitch_rad) + uniform_noise(DUMMY_ACCEL_NOISE_MS2);
    out->gx = uniform_noise(DUMMY_GYRO_NOISE_DPS);
    out->gy = uniform_noise(DUMMY_GYRO_NOISE_DPS);
    out->gz = uniform_noise(DUMMY_GYRO_NOISE_DPS);
}

/* --- Public API ---------------------------------------------------------- */

void dummy_set_scenario(DummyScenario scenario) {
    s_scenario = scenario;
}

float dummy_get_target_angle(void) {
    return scenario_to_angle(s_scenario);
}

/* --- imu_source.h implementation ----------------------------------------- */

void imu_source_init(void) {
    /* Deterministic seed so test runs are reproducible. */
    srand(1);
    s_scenario = SCENARIO_GOOD;
}

bool imu_source_read(ImuRaw *out_lower, ImuRaw *out_upper) {
    if (out_lower == NULL || out_upper == NULL) {
        return false;
    }
    /* Lower IMU sits at the hip — model it as level. */
    synth_sample_at_pitch(0.0f, out_lower);
    /* Upper IMU sits on the upper back — model it tilted by the
     * scenario's target angle. */
    synth_sample_at_pitch(scenario_to_angle(s_scenario), out_upper);
    return true;
}
