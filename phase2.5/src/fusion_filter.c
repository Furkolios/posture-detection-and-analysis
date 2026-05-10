#include <math.h>
#include <stddef.h>
#include "fusion_filter.h"

/* --- Madgwick AHRS, IMU-only formulation ------------------------------- *
 *
 * Reference: Madgwick, S.O.H. (2010), "An efficient orientation filter
 * for inertial and inertial/magnetic sensor arrays". Open access PDF
 * from x-io Technologies.
 *
 * Per-sensor algorithm (each IMU runs its own copy of state q):
 *
 *   1. Read accelerometer a (already in m/s^2) and gyro w (deg/s).
 *      Convert gyro to rad/s for integration.
 *   2. q_dot_gyro = 0.5 * q * (0, wx, wy, wz)
 *   3. If accel is non-zero: normalize it, then form the gradient of
 *      the objective function f(q) = q^-1 * g_world - g_body where
 *      g_world = (0, 0, 0, 1) and g_body = (0, ax, ay, az). The
 *      Jacobian-times-objective expansion gives a closed-form 4-vector
 *      gradient s; normalize it.
 *   4. q_dot = q_dot_gyro - beta * s
 *   5. q += q_dot * dt; renormalize.
 *
 * We run this on each IMU independently, then take the relative
 * orientation q_rel = conj(q_lower) * q_upper and extract pitch (Y)
 * and roll (X) from q_rel using the standard quaternion-to-Euler
 * formulas.
 */

/* Gradient descent gain. 0.1 is a reasonable starting point per the
 * Madgwick paper for a 100 Hz update rate. Re-tune empirically with
 * real sensors during CP9. */
#define MADGWICK_BETA          0.1f

/* deg <-> rad. */
#define DEG_TO_RAD             (3.14159265358979323846f / 180.0f)
#define RAD_TO_DEG             (180.0f / 3.14159265358979323846f)

/* --- Module state -------------------------------------------------------- *
 * Two orientation quaternions, one per IMU. Both default to identity. */
static ImuQuaternion s_q_lower = {1.0f, 0.0f, 0.0f, 0.0f};
static ImuQuaternion s_q_upper = {1.0f, 0.0f, 0.0f, 0.0f};

/* Cached pitch/roll from the most recent update, so
 * fusion_filter_get_angles() can serve them without recomputing. */
static float s_last_pitch_deg = 0.0f;
static float s_last_roll_deg  = 0.0f;

/* --- helpers ------------------------------------------------------------- */

static float inv_sqrtf(float x) {
    /* Plain 1/sqrt; the original Madgwick paper uses the fast
     * inverse square root trick, but on a Cortex-M4F with hardware
     * sqrt we don't need it and clarity wins. */
    return 1.0f / sqrtf(x);
}

/* Normalize a quaternion in place. Safe against zero (no-op). */
static void quat_normalize(ImuQuaternion *q) {
    float n2 = q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z;
    if (n2 <= 0.0f) {
        return;
    }
    float inv = inv_sqrtf(n2);
    q->w *= inv; q->x *= inv; q->y *= inv; q->z *= inv;
}

/* One Madgwick step on a single quaternion. dt in seconds, gyro in
 * deg/s, accel in any consistent unit (only direction matters). */
static void madgwick_step(ImuQuaternion *q,
                          float ax, float ay, float az,
                          float gx_dps, float gy_dps, float gz_dps,
                          float dt) {
    /* Gyro in rad/s. */
    float gx = gx_dps * DEG_TO_RAD;
    float gy = gy_dps * DEG_TO_RAD;
    float gz = gz_dps * DEG_TO_RAD;

    /* q_dot from gyro: 0.5 * q ⊗ (0, gx, gy, gz). */
    float qw = q->w, qx = q->x, qy = q->y, qz = q->z;
    float qdot_w = 0.5f * (-qx*gx - qy*gy - qz*gz);
    float qdot_x = 0.5f * ( qw*gx + qy*gz - qz*gy);
    float qdot_y = 0.5f * ( qw*gy - qx*gz + qz*gx);
    float qdot_z = 0.5f * ( qw*gz + qx*gy - qy*gx);

    /* Only apply the accelerometer correction if it carries useful
     * direction info. A near-zero accel reading would normalize to
     * garbage. */
    float a_norm2 = ax*ax + ay*ay + az*az;
    if (a_norm2 > 0.0f) {
        float inv = inv_sqrtf(a_norm2);
        ax *= inv; ay *= inv; az *= inv;

        /* Closed-form gradient s = J^T * f for the gravity objective.
         * Derivation in the Madgwick paper, eqs. (25)-(33). The two
         * common subexpressions used in eq (33) are pulled out for
         * readability. */
        float two_qw = 2.0f * qw;
        float two_qx = 2.0f * qx;
        float two_qy = 2.0f * qy;
        float two_qz = 2.0f * qz;

        float f1 = 2.0f*(qx*qz - qw*qy) - ax;
        float f2 = 2.0f*(qw*qx + qy*qz) - ay;
        float f3 = 2.0f*(0.5f - qx*qx - qy*qy) - az;

        float s_w =  -two_qy*f1 + two_qx*f2;
        float s_x =   two_qz*f1 + two_qw*f2 - 4.0f*qx*f3;
        float s_y =  -two_qw*f1 + two_qz*f2 - 4.0f*qy*f3;
        float s_z =   two_qx*f1 + two_qy*f2;

        /* Normalize the gradient before applying — keeps beta
         * dimensionally meaningful. */
        float s_norm2 = s_w*s_w + s_x*s_x + s_y*s_y + s_z*s_z;
        if (s_norm2 > 0.0f) {
            float s_inv = inv_sqrtf(s_norm2);
            s_w *= s_inv; s_x *= s_inv; s_y *= s_inv; s_z *= s_inv;

            qdot_w -= MADGWICK_BETA * s_w;
            qdot_x -= MADGWICK_BETA * s_x;
            qdot_y -= MADGWICK_BETA * s_y;
            qdot_z -= MADGWICK_BETA * s_z;
        }
    }

    /* Integrate and renormalize. */
    q->w += qdot_w * dt;
    q->x += qdot_x * dt;
    q->y += qdot_y * dt;
    q->z += qdot_z * dt;
    quat_normalize(q);
}

/* Quaternion conjugate. */
static ImuQuaternion quat_conj(ImuQuaternion q) {
    ImuQuaternion r = { q.w, -q.x, -q.y, -q.z };
    return r;
}

/* Hamilton product a ⊗ b. */
static ImuQuaternion quat_mul(ImuQuaternion a, ImuQuaternion b) {
    ImuQuaternion r;
    r.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
    r.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
    r.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
    r.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
    return r;
}

/* Extract pitch (rotation about Y) and roll (rotation about X) from a
 * unit quaternion using the standard ZYX Tait-Bryan formulas. Pitch is
 * clamped to ±90° via asin's natural domain, and the input to asinf is
 * clamped to [-1, 1] to defend against tiny floating-point overshoots. */
static void quat_to_pitch_roll(ImuQuaternion q, float *pitch, float *roll) {
    float sinp = 2.0f * (q.w*q.y - q.z*q.x);
    if (sinp >  1.0f) sinp =  1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    *pitch = asinf(sinp) * RAD_TO_DEG;

    float sinr_cosp = 2.0f * (q.w*q.x + q.y*q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x*q.x + q.y*q.y);
    *roll = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;
}

/* --- Public API ---------------------------------------------------------- */

void fusion_filter_reset(void) {
    s_q_lower.w = 1.0f; s_q_lower.x = 0.0f; s_q_lower.y = 0.0f; s_q_lower.z = 0.0f;
    s_q_upper.w = 1.0f; s_q_upper.x = 0.0f; s_q_upper.y = 0.0f; s_q_upper.z = 0.0f;
    s_last_pitch_deg = 0.0f;
    s_last_roll_deg  = 0.0f;
}

float fusion_filter_update(const ImuRaw *lower,
                           const ImuRaw *upper,
                           float dt_seconds) {
    madgwick_step(&s_q_lower,
                  lower->ax, lower->ay, lower->az,
                  lower->gx, lower->gy, lower->gz,
                  dt_seconds);
    madgwick_step(&s_q_upper,
                  upper->ax, upper->ay, upper->az,
                  upper->gx, upper->gy, upper->gz,
                  dt_seconds);

    /* Relative orientation: q_rel takes vectors from upper-IMU frame
     * into lower-IMU frame. q_rel = conj(q_lower) * q_upper. */
    ImuQuaternion q_rel = quat_mul(quat_conj(s_q_lower), s_q_upper);
    quat_normalize(&q_rel);

    quat_to_pitch_roll(q_rel, &s_last_pitch_deg, &s_last_roll_deg);
    return s_last_pitch_deg;
}

void fusion_filter_get_angles(float *out_pitch_deg, float *out_roll_deg) {
    if (out_pitch_deg != NULL) *out_pitch_deg = s_last_pitch_deg;
    if (out_roll_deg  != NULL) *out_roll_deg  = s_last_roll_deg;
}
