#ifndef CLASSIFIER_H
#define CLASSIFIER_H

#include "posture_types.h"

/* --- Public threshold constants ---------------------------------------- *
 * Empirically chosen for the wearable's geometry. Adjust here only —
 * never hardcoded inside classifier.c. */

/* Pitch deviation (deg) above which we consider the user not-good. */
#define THRESHOLD_MILD_DEG    10.0f

/* Pitch deviation (deg) above which we consider the user fully slouched. */
#define THRESHOLD_FULL_DEG    25.0f

/* Pitch deviation (deg) below neutral that counts as leaning back. */
#define THRESHOLD_LEAN_BACK_DEG  -10.0f

/* Roll deviation magnitude (deg) above which lateral tilt fires.
 * Lateral tilt overrides any pitch verdict — see classifier.c for the
 * precedence rule and the rationale. */
#define THRESHOLD_LATERAL_DEG  15.0f

/* Capture the user's neutral pitch as the calibration baseline.
 * Convenience wrapper for `classifier_calibrate_full(neutral_angle, 0)`. */
void classifier_calibrate(float neutral_angle_deg);

/* Capture the user's neutral pitch and roll as calibration baselines.
 * Subsequent classifications are deviations from this pair. */
void classifier_calibrate_full(float neutral_pitch_deg,
                               float neutral_roll_deg);

/* Classify a fused relative-pitch angle into a PostureState.
 *
 * Single-axis call: roll is treated as zero, so this function can never
 * return POSTURE_LATERAL_TILT. Phase 1 callers keep working unchanged.
 * Use classifier_classify_full() if both axes are available. */
PostureState classifier_classify(float angle_deg);

/* Classify based on both pitch and roll deviations. Recommended for
 * Phase 2 callers — produces the full 6-state output including
 * lean-forward / lean-back / lateral-tilt. */
PostureState classifier_classify_full(float pitch_deg, float roll_deg);

#endif /* CLASSIFIER_H */
