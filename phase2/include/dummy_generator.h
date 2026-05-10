#ifndef DUMMY_GENERATOR_H
#define DUMMY_GENERATOR_H

#include "imu_source.h"

/* Scenario presets for the dummy IMU generator. The generator produces
 * accel/gyro values that, after fusion, land in the corresponding
 * angle band. This is the only knob the test harness needs. */
typedef enum {
    SCENARIO_GOOD         = 0,  /* near 0 degrees relative pitch */
    SCENARIO_MILD_SLOUCH  = 1,  /* around 18 degrees */
    SCENARIO_FULL_SLOUCH  = 2   /* around 35 degrees */
} DummyScenario;

#define DUMMY_SCENARIO_COUNT 3

/* Set the active scenario. Subsequent calls to imu_source_read() will
 * return samples consistent with the selected posture. */
void dummy_set_scenario(DummyScenario scenario);

/* Get the angle (in degrees) the current scenario is targeting.
 * Useful for assertions in the test harness. */
float dummy_get_target_angle(void);

#endif /* DUMMY_GENERATOR_H */
