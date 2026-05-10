#ifndef ALERT_H
#define ALERT_H

#include "posture_types.h"

/* Initialise the alert subsystem (calls buzzer_init internally,
 * resets the internal tick counter). Call once at startup. */
void alert_init(void);

/* Run one tick of the alert state machine.
 * Caller is expected to invoke this on every main-loop iteration
 * with the current classified posture state. The function decides
 * whether to drive the buzzer on or off this tick.
 *
 *   POSTURE_GOOD          -> buzzer always off
 *   POSTURE_MILD_SLOUCH   -> short periodic beep
 *   POSTURE_FULL_SLOUCH   -> buzzer continuously on
 *   POSTURE_LEAN_FORWARD  -> short double beep, periodic
 *   POSTURE_LEAN_BACK     -> short double beep, periodic (same pattern)
 *   POSTURE_LATERAL_TILT  -> three short pulses, periodic
 *
 * The cadences for the new states are intentionally different from
 * MILD_SLOUCH so the user can identify the issue by ear. */
void alert_update(PostureState state);

#endif /* ALERT_H */
