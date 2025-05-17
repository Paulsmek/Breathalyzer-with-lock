#include <avr/io.h>
#include <avr/interrupt.h>
#include "timers.h"

/* Global millisecond tick counter */
volatile uint32_t systicks = 0;

/* Dummy globals for PWM levels (optional) */
volatile uint8_t red_duty_level = 0;
volatile uint8_t blue_duty_level = 0;

/* ========================= Timer0 - systicks (1ms) ========================= */
void Timer0_init_systicks(void) {
    TCCR0A = (1 << WGM01);         // CTC mode
    TCCR0B = (1 << CS02);          // Prescaler 256
    OCR0A = 62;                    // 16MHz / 256 = 62.5kHz → 62 = ~1ms
    TIMSK0 |= (1 << OCIE0A);       // Enable Timer0 Compare Match A interrupt
}

ISR(TIMER0_COMPA_vect) {
    systicks++;
}

/* ========================= Timer1 - Servo PWM (50Hz) ========================= */
void Timer1_init_servo_pwm(void) {
    TCCR1A = (1 << WGM11) | (1 << COM1A1);                    // Fast PWM, non-inverting
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);       // Prescaler 8
    ICR1 = 40000;                                             // TOP → 20ms = 50Hz (16MHz/8 = 2MHz → 2M / 50Hz = 40000)
    DDRB |= (1 << PB1);                                       // Set OC1A (D9) as output
}

/* ========================= Timer2 - Optional Fast PWM ========================= */
void Timer2_init_pwm(void) {
    TCCR2A = (1 << WGM20) | (1 << WGM21) | (1 << COM2A1);  // Fast PWM, non-inverting
    TCCR2B = (1 << CS21);                                  // Prescaler 8
    DDRD |= (1 << PD7);                                    // OC2A (PD7) output
    OCR2A = 0;
}
