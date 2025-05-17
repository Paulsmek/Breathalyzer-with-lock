#ifndef TIMERS_H_
#define TIMERS_H_

#include <inttypes.h>

/* System millisecond counter */
extern volatile uint32_t systicks;

/* (Optional) duty cycle tracking for custom PWM */
extern volatile uint8_t red_duty_level;
extern volatile uint8_t blue_duty_level;

/* Time comparison macro */
#define SYSTICKS_PASSED(last_event, diff_amount) \
    ((systicks - (last_event)) >= (diff_amount))

/* Initialize Timer0 for 1ms systicks (CTC mode) */
void Timer0_init_systicks(void);

/* Initialize Timer1 for 50Hz servo PWM on OC1A */
void Timer1_init_servo_pwm(void);

/* Optional: Fast PWM on Timer2 */
void Timer2_init_pwm(void);

#endif // TIMERS_H_
