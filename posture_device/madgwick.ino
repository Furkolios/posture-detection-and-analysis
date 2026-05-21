/*
 * Madgwick IMU-only filter (no magnetometer).
 *
 * Reference: Madgwick, S.O.H. "An efficient orientation filter for inertial
 * and inertial/magnetic sensor arrays", 2010. We use the IMU-only variant.
 *
 * Inputs: gyro in rad/s, accel in any consistent unit (we normalise it).
 * State: a unit quaternion (q0, q1, q2, q3) per IMU.
 */

#define MADGWICK_BETA   0.2f
#define RAD_TO_DEG_F    57.29577951308232f

/* Reciprocal square root — plain version, the FPU on the MSP432 is fine. */
static inline float invSqrtf(float x) {
    return 1.0f / sqrtf(x);
}

/* One Madgwick update step. dt is the seconds since the last call.
 * Quaternion convention: q0 = w (scalar), (q1, q2, q3) = (x, y, z). */
void madgwickUpdate(MadgwickState *s,
                    float gx, float gy, float gz,
                    float ax, float ay, float az,
                    float dt) {
    float q0 = s->q0, q1 = s->q1, q2 = s->q2, q3 = s->q3;
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;

    // Quaternion rate from gyro.
    qDot1 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    qDot2 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    qDot3 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    qDot4 = 0.5f * ( q0*gz + q1*gy - q2*gx);

    // Apply accelerometer feedback only if it's not the zero vector.
    if (!(ax == 0.0f && ay == 0.0f && az == 0.0f)) {
        recipNorm = invSqrtf(ax*ax + ay*ay + az*az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        float _2q0 = 2.0f*q0, _2q1 = 2.0f*q1, _2q2 = 2.0f*q2, _2q3 = 2.0f*q3;
        float _4q0 = 4.0f*q0, _4q1 = 4.0f*q1, _4q2 = 4.0f*q2;
        float _8q1 = 8.0f*q1, _8q2 = 8.0f*q2;
        float q0q0 = q0*q0, q1q1 = q1*q1, q2q2 = q2*q2, q3q3 = q3*q3;

        s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
        s1 = _4q1*q3q3 - _2q3*ax + 4.0f*q0q0*q1 - _2q0*ay - _4q1
                       + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
        s2 = 4.0f*q0q0*q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2
                       + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
        s3 = 4.0f*q1q1*q3 - _2q1*ax + 4.0f*q2q2*q3 - _2q2*ay;

        recipNorm = invSqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

        qDot1 -= MADGWICK_BETA * s0;
        qDot2 -= MADGWICK_BETA * s1;
        qDot3 -= MADGWICK_BETA * s2;
        qDot4 -= MADGWICK_BETA * s3;
    }

    // Integrate rate of change of quaternion.
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Renormalise.
    recipNorm = invSqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    s->q0 = q0 * recipNorm;
    s->q1 = q1 * recipNorm;
    s->q2 = q2 * recipNorm;
    s->q3 = q3 * recipNorm;
}

/* Extract pitch (rotation around Y) and roll (rotation around X) in degrees
 * from a unit quaternion, using the aerospace ZYX convention. Yaw is dropped
 * because, without a magnetometer, it drifts and isn't physically meaningful
 * for posture. */
void quaternionToPitchRoll(const MadgwickState *s,
                           float *pitch_deg, float *roll_deg) {
    float q0 = s->q0, q1 = s->q1, q2 = s->q2, q3 = s->q3;

    // Pitch: clamp before asin to avoid NaNs from tiny FP overshoots.
    float sinp = 2.0f * (q0*q2 - q3*q1);
    if (sinp >  1.0f) sinp =  1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    float pitch = asinf(sinp);

    float roll = atan2f(2.0f * (q0*q1 + q2*q3),
                        1.0f - 2.0f * (q1*q1 + q2*q2));

    *pitch_deg = pitch * RAD_TO_DEG_F;
    *roll_deg  = roll  * RAD_TO_DEG_F;
}
