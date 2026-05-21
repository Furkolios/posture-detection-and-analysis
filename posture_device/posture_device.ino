/*
 * Wearable Posture Detection — Main Sketch
 *
 * Target: MSP432P401R LaunchPad (Energia)
 *
 * Wiring (verify pin numbers against your Energia board package):
 *
 *   Function          | LP Pin | MCU Pin   | Notes
 *   ------------------+--------+-----------+--------------------------------
 *   I2C SDA           | 10     | P6.4      | Shared by both MPU-6050s
 *   I2C SCL           |  9     | P6.5      | Shared by both MPU-6050s
 *   MPU-6050 lower    | I2C    | 0x68      | AD0 -> GND, hip / pelvis
 *   MPU-6050 upper    | I2C    | 0x69      | AD0 -> VCC, upper back
 *   HM-10 BLE RX      |  4     | P3.3 (TX) | LP TX -> HM-10 RX
 *   HM-10 BLE TX      |  3     | P3.2 (RX) | HM-10 TX -> LP RX
 *   Buzzer (+)        |  8     | P4.6      | Active piezo; pick any free pin
 *   Buzzer (-)        | GND    | -         |
 *
 * Power both MPU-6050s and the HM-10 from 3V3. The MSP432 is a 3.3 V part —
 * do NOT feed it 5 V signals. The HM-10 RX is 3.3 V tolerant.
 */

#include <Wire.h>
#include <math.h>
#include <string.h>
#include "types.h"   // PostureState, IMUSample, MadgwickState — keep this above all user code

// ---- Pin / address constants --------------------------------------------

#define MPU_ADDR_LOWER  0x68
#define MPU_ADDR_UPPER  0x69
#define BUZZER_PIN      8

// ---- Loop timing --------------------------------------------------------

#define IMU_PERIOD_MS         10     // 100 Hz sensor read + fusion
#define TELEMETRY_PERIOD_MS   50     // 20 Hz packet rate
#define CALIBRATION_MS        2000   // 2 s neutral-posture window

// ---- Globals ------------------------------------------------------------

static MadgwickState g_lower = {1.0f, 0.0f, 0.0f, 0.0f};
static MadgwickState g_upper = {1.0f, 0.0f, 0.0f, 0.0f};

static float g_neutral_pitch = 0.0f;
static float g_neutral_roll  = 0.0f;

static float g_pitch = 0.0f;             // current pitch deviation from neutral (deg)
static float g_roll  = 0.0f;             // current roll  deviation from neutral (deg)
static PostureState g_state = POSTURE_GOOD;

static unsigned long g_lastImuMs        = 0;
static unsigned long g_lastTelemetryMs  = 0;
static unsigned long g_lastMicros       = 0;

// ---- Forward declarations (definitions live in other tabs) --------------

void          imuInit(uint8_t addr);
bool          imuRead(uint8_t addr, IMUSample *out);

void          madgwickUpdate(MadgwickState *s,
                             float gx, float gy, float gz,
                             float ax, float ay, float az,
                             float dt);
void          quaternionToPitchRoll(const MadgwickState *s,
                                    float *pitch_deg, float *roll_deg);

PostureState  classify(float pitch_deg, float roll_deg);

void          buzzerInit();
void          buzzerUpdate(PostureState s);
void          buzzerOneShot(unsigned long duration_ms);

void          telemetryInit();
void          telemetrySend(float pitch_deg, float roll_deg, PostureState s);

// =========================================================================

void setup() {
    Wire.begin();
    // (Energia's MSP432 Wire library doesn't expose setClock(); default 100 kHz
    //  is plenty for two MPU-6050s at 100 Hz.)

    imuInit(MPU_ADDR_LOWER);
    imuInit(MPU_ADDR_UPPER);

    buzzerInit();
    telemetryInit();

    // Let the IMUs settle after wake-up before we trust their output.
    delay(5000);

    g_lastMicros = micros();

    // ---- Calibration window: average the relative angles for 2 s.
    // The user is expected to hold their target "good" posture during boot.
    float sum_pitch = 0.0f, sum_roll = 0.0f;
    unsigned long samples = 0;
    unsigned long cal_start = millis();

    while (millis() - cal_start < CALIBRATION_MS) {
        if (millis() - g_lastImuMs < IMU_PERIOD_MS) continue;

        unsigned long now_us = micros();
        float dt = (now_us - g_lastMicros) / 1.0e6f;
        g_lastMicros = now_us;
        if (dt <= 0.0f || dt > 0.1f) dt = (float)IMU_PERIOD_MS / 1000.0f;

        IMUSample lo, up;
        if (imuRead(MPU_ADDR_LOWER, &lo) && imuRead(MPU_ADDR_UPPER, &up)) {
            madgwickUpdate(&g_lower, lo.gx, lo.gy, lo.gz, lo.ax, lo.ay, lo.az, dt);
            madgwickUpdate(&g_upper, up.gx, up.gy, up.gz, up.ax, up.ay, up.az, dt);

            float pL, rL, pU, rU;
            quaternionToPitchRoll(&g_lower, &pL, &rL);
            quaternionToPitchRoll(&g_upper, &pU, &rU);

            sum_pitch += (pU - pL);
            sum_roll  += (rU - rL);
            samples++;
        }
        g_lastImuMs = millis();
    }

    if (samples > 0) {
        g_neutral_pitch = sum_pitch / (float)samples;
        g_neutral_roll  = sum_roll  / (float)samples;
    }

    // Short audible cue: calibration done, going live.
    buzzerOneShot(150);

    // Re-anchor the schedule so the loop starts with clean clocks.
    g_lastImuMs       = millis();
    g_lastTelemetryMs = millis();
    g_lastMicros      = micros();
}

void loop() {
    unsigned long now = millis();

    // ---- Sensor read + fusion + classification at IMU_PERIOD_MS. --------
    if (now - g_lastImuMs >= IMU_PERIOD_MS) {
        unsigned long now_us = micros();
        float dt = (now_us - g_lastMicros) / 1.0e6f;
        g_lastMicros = now_us;
        if (dt <= 0.0f || dt > 0.1f) dt = (float)IMU_PERIOD_MS / 1000.0f;

        IMUSample lo, up;
        bool ok_lo = imuRead(MPU_ADDR_LOWER, &lo);
        bool ok_up = imuRead(MPU_ADDR_UPPER, &up);

        if (ok_lo && ok_up) {
            madgwickUpdate(&g_lower, lo.gx, lo.gy, lo.gz, lo.ax, lo.ay, lo.az, dt);
            madgwickUpdate(&g_upper, up.gx, up.gy, up.gz, up.ax, up.ay, up.az, dt);

            float pL, rL, pU, rU;
            quaternionToPitchRoll(&g_lower, &pL, &rL);
            quaternionToPitchRoll(&g_upper, &pU, &rU);

            g_pitch = (pU - pL) - g_neutral_pitch;
            g_roll  = (rU - rL) - g_neutral_roll;
            g_state = classify(g_pitch, g_roll);
        }
        // If a sensor read failed, we keep the previous state. Better than
        // a momentary glitch flipping the buzzer.

        g_lastImuMs = now;
    }

    // ---- Buzzer pattern advances every loop iteration. ------------------
    buzzerUpdate(g_state);

    // ---- Telemetry packet at TELEMETRY_PERIOD_MS. -----------------------
    if (now - g_lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
        telemetrySend(g_pitch, g_roll, g_state);
        g_lastTelemetryMs = now;
    }
}
