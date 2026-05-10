#include <math.h>
#include "classifier.h"

/* --- Module state -------------------------------------------------------- *
 * The neutral baseline captured during calibration. All classification
 * is done in deviations from this pair. Both default to zero so that an
 * uncalibrated classifier still produces sensible results against a
 * dummy generator that emits zero-centred data. */
static float s_neutral_pitch_deg = 0.0f;
static float s_neutral_roll_deg  = 0.0f;

void classifier_calibrate(float neutral_angle_deg) {
    classifier_calibrate_full(neutral_angle_deg, 0.0f);
}

void classifier_calibrate_full(float neutral_pitch_deg,
                               float neutral_roll_deg) {
    s_neutral_pitch_deg = neutral_pitch_deg;
    s_neutral_roll_deg  = neutral_roll_deg;
}

PostureState classifier_classify(float angle_deg) {
    /* Single-axis path: roll fixed to zero, so lateral-tilt cannot fire
     * through this entry point. */
    return classifier_classify_full(angle_deg, s_neutral_roll_deg);
}

/* Classification rules, in priority order:
 *
 *   1. If |roll deviation| >= THRESHOLD_LATERAL_DEG, the user is
 *      tipping sideways — that's clinically distinct from forward
 *      slouch and we surface it explicitly even if pitch is also off.
 *      Lateral tilt wins.
 *
 *   2. Otherwise, judge by signed pitch deviation:
 *        - pitch >= THRESHOLD_FULL_DEG       -> POSTURE_FULL_SLOUCH
 *        - pitch >= THRESHOLD_MILD_DEG       -> POSTURE_MILD_SLOUCH
 *        - pitch <= THRESHOLD_LEAN_BACK_DEG  -> POSTURE_LEAN_BACK
 *        - small forward bow but not slouch  -> POSTURE_LEAN_FORWARD
 *          (covers the "I'm leaning to read" case the proposal calls
 *           out as a Phase 3 ML target — for now, a separate state is
 *           the best we can do with rules)
 *        - else                               -> POSTURE_GOOD
 *
 * Note: LEAN_FORWARD currently overlaps with the lower end of MILD,
 * so we keep MILD_SLOUCH as the more severe label and reserve
 * LEAN_FORWARD for a small forward bow that doesn't yet cross
 * the mild threshold but is still off-neutral. */

/* Forward bow has to be at least this far from neutral to register
 * as a deliberate forward lean (smaller than MILD). Hardcoded as a
 * fraction of THRESHOLD_MILD_DEG so the two move together. */
#define LEAN_FORWARD_MIN_DEG  (THRESHOLD_MILD_DEG * 0.5f)

PostureState classifier_classify_full(float pitch_deg, float roll_deg) {
    float pitch_dev = pitch_deg - s_neutral_pitch_deg;
    float roll_dev  = roll_deg  - s_neutral_roll_deg;

    if (fabsf(roll_dev) >= THRESHOLD_LATERAL_DEG) {
        return POSTURE_LATERAL_TILT;
    }

    if (pitch_dev >= THRESHOLD_FULL_DEG) {
        return POSTURE_FULL_SLOUCH;
    }
    if (pitch_dev >= THRESHOLD_MILD_DEG) {
        return POSTURE_MILD_SLOUCH;
    }
    if (pitch_dev <= THRESHOLD_LEAN_BACK_DEG) {
        return POSTURE_LEAN_BACK;
    }
    if (pitch_dev >= LEAN_FORWARD_MIN_DEG) {
        return POSTURE_LEAN_FORWARD;
    }
    return POSTURE_GOOD;
}
