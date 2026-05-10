#include <stdio.h>
#include "hal_buzzer.h"

/* Host stub. Tracks state and prints transitions to stderr so test
 * scripts can grep for them. Stays out of stdout so it doesn't
 * interfere with telemetry packets there. */

static int s_buzzer_on = 0;

void buzzer_init(void) {
    s_buzzer_on = 0;
    fprintf(stderr, "[buzzer] init\n");
}

void buzzer_on(void) {
    if (!s_buzzer_on) {
        s_buzzer_on = 1;
        fprintf(stderr, "[buzzer] ON\n");
    }
}

void buzzer_off(void) {
    if (s_buzzer_on) {
        s_buzzer_on = 0;
        fprintf(stderr, "[buzzer] OFF\n");
    }
}
