/*
 * Buzzer driver — non-blocking pattern generator keyed off millis().
 *
 * Patterns repeat every BUZZ_CYCLE_MS:
 *   GOOD          : silent
 *   FULL_SLOUCH   : continuously on
 *   MILD_SLOUCH   : single 200 ms beep at start of cycle
 *   LEAN_FWD/BACK : two 200 ms beeps separated by a 200 ms gap
 *   LATERAL_TILT  : three 200 ms beeps separated by 200 ms gaps
 *
 * Assumes an *active* piezo buzzer: HIGH = sound, LOW = silent.
 */

#define BUZZ_BEEP_MS   200
#define BUZZ_GAP_MS    200
#define BUZZ_CYCLE_MS  3000

static PostureState s_lastState   = POSTURE_GOOD;
static unsigned long s_cycleStart = 0;

/* Configure the buzzer pin as a digital output. */
void buzzerInit() {
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
}

/* Drive the buzzer briefly. Blocking — only safe to call from setup().
 * Used to signal end-of-calibration. */
void buzzerOneShot(unsigned long duration_ms) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(duration_ms);
    digitalWrite(BUZZER_PIN, LOW);
}

/* Update the buzzer output based on the current posture state. Designed to
 * be called every loop iteration — no blocking, millis()-only. */
void buzzerUpdate(PostureState s) {
    unsigned long now = millis();

    // Reset the cycle when the state changes so patterns line up cleanly.
    if (s != s_lastState) {
        s_cycleStart = now;
        s_lastState  = s;
    }

    unsigned long t = (now - s_cycleStart) % BUZZ_CYCLE_MS;
    bool on = false;

    switch (s) {
        case POSTURE_GOOD:
            on = false;
            break;

        case POSTURE_FULL_SLOUCH:
            on = true;
            break;

        case POSTURE_MILD_SLOUCH:
            on = (t < BUZZ_BEEP_MS);
            break;

        case POSTURE_LEAN_FORWARD:
        case POSTURE_LEAN_BACK: {
            unsigned long beep2_start = BUZZ_BEEP_MS + BUZZ_GAP_MS;
            unsigned long beep2_end   = beep2_start + BUZZ_BEEP_MS;
            on = (t < BUZZ_BEEP_MS) || (t >= beep2_start && t < beep2_end);
            break;
        }

        case POSTURE_LATERAL_TILT: {
            unsigned long beep2_start = BUZZ_BEEP_MS + BUZZ_GAP_MS;
            unsigned long beep2_end   = beep2_start + BUZZ_BEEP_MS;
            unsigned long beep3_start = beep2_end + BUZZ_GAP_MS;
            unsigned long beep3_end   = beep3_start + BUZZ_BEEP_MS;
            on = (t < BUZZ_BEEP_MS)
              || (t >= beep2_start && t < beep2_end)
              || (t >= beep3_start && t < beep3_end);
            break;
        }

        default:
            on = false;
            break;
    }

    digitalWrite(BUZZER_PIN, on ? HIGH : LOW);
}
