#ifndef HAL_BUZZER_H
#define HAL_BUZZER_H

/* Configure the GPIO pin that drives the piezo buzzer. */
void buzzer_init(void);

/* Drive the buzzer pin high (audible). */
void buzzer_on(void);

/* Drive the buzzer pin low (silent). */
void buzzer_off(void);

#endif /* HAL_BUZZER_H */
