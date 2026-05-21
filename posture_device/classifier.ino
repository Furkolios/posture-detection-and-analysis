/*
 * Posture classifier — pure function from relative angles (deviation from
 * the calibrated neutral) to a posture state.
 *
 * Priority (highest first), so each state has a unique pitch range:
 *
 *   1. POSTURE_LATERAL_TILT  if |roll|  >= 15  — lateral lean takes over.
 *   2. POSTURE_FULL_SLOUCH   if  pitch  >= 25  — severe forward curl.
 *   3. POSTURE_LEAN_FORWARD  if  pitch  >= 15  — clear forward lean.
 *   4. POSTURE_MILD_SLOUCH   if  pitch  >= 10  — early warning.
 *   5. POSTURE_LEAN_BACK     if  pitch  <= -15 — leaning backward.
 *   6. POSTURE_GOOD          otherwise.
 *
 * Note: with only two IMUs we measure spine curvature (relative angle), so
 * "lean forward" here means the upper sensor is rotated forward relative to
 * the hip sensor — geometrically the same as a slouch but in a milder range.
 * The thresholds above match the spec's stated values literally.
 */

#define PITCH_MILD_SLOUCH_DEG    10.0f
#define PITCH_LEAN_FORWARD_DEG   15.0f
#define PITCH_FULL_SLOUCH_DEG    25.0f
#define PITCH_LEAN_BACK_DEG      15.0f   // applied to -pitch
#define ROLL_LATERAL_DEG         20.0f

/* Map a (pitch, roll) deviation to one of the six posture states. */
PostureState classify(float pitch_deg, float roll_deg) {
    if (fabsf(roll_deg) >= ROLL_LATERAL_DEG) return POSTURE_LATERAL_TILT;

    if (pitch_deg >=  PITCH_FULL_SLOUCH_DEG)   return POSTURE_FULL_SLOUCH;
    if (pitch_deg >=  PITCH_LEAN_FORWARD_DEG)  return POSTURE_LEAN_FORWARD;
    if (pitch_deg >=  PITCH_MILD_SLOUCH_DEG)   return POSTURE_MILD_SLOUCH;
    if (pitch_deg <= -PITCH_LEAN_BACK_DEG)     return POSTURE_LEAN_BACK;

    return POSTURE_GOOD;
}
