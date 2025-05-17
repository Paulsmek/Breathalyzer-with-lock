#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdint.h>

#include "usart.h"
#include "timers.h"

#define F_CPU 16000000UL

// LCD I2C settings
#define I2C_ADDRESS 0x27
#define LCD_BACKLIGHT 0x08
#define ENABLE 0x04
#define RW 0x02
#define RS 0x01

// Pins
#define BUTTON_PIN    PD2
#define BUZZER_PIN    PD4
#define SENSOR_PIN    0
#define SERVO_PIN     PB1
#define LED_RED       PB0
#define LED_YELLOW    PD6
#define LED_GREEN     PD7

#define WARMUP_TIME_MS   45000
#define READ_TIME_MS     7000

// State flags
volatile uint8_t isMeasuring = 0;
volatile uint8_t measurementDone = 0;
volatile uint8_t door_is_open = 0;
volatile uint8_t button_pressed = 0;
volatile uint32_t measurementStart = 0;
volatile uint32_t last_button_time = 0;
uint16_t alcoholValue = 0;

/* ---------------- LCD I2C low-level ---------------- */
void TWI_init(void) {
    TWSR = 0x00;
    TWBR = 72;
    TWCR = (1 << TWEN);
}

void TWI_start(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

void TWI_stop(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
    while (TWCR & (1 << TWSTO));
}

void TWI_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

void LCD_send_nibble(uint8_t nibble, uint8_t mode) {
    uint8_t data = (nibble & 0xF0) | LCD_BACKLIGHT | mode;
    TWI_start();
    TWI_write((I2C_ADDRESS << 1));
    TWI_write(data | ENABLE);
    _delay_us(1);
    TWI_write(data & ~ENABLE);
    _delay_us(50);
    TWI_stop();
}

void LCD_send_byte(uint8_t byte, uint8_t mode) {
    LCD_send_nibble(byte & 0xF0, mode);
    LCD_send_nibble((byte << 4) & 0xF0, mode);
}

void LCD_command(uint8_t cmd) {
    LCD_send_byte(cmd, 0);
}

void LCD_data(uint8_t data) {
    LCD_send_byte(data, RS);
}

void LCD_init(void) {
    _delay_ms(50);
    LCD_send_nibble(0x30, 0); _delay_ms(5);
    LCD_send_nibble(0x30, 0); _delay_ms(1);
    LCD_send_nibble(0x30, 0); _delay_us(150);
    LCD_send_nibble(0x20, 0); _delay_us(150);
    LCD_command(0x28);
    LCD_command(0x0C);
    LCD_command(0x06);
    LCD_command(0x01);
    _delay_ms(2);
}

void LCD_set_cursor(uint8_t col, uint8_t row) {
    LCD_command(0x80 | (row ? 0x40 : 0x00) | col);
}

void LCD_print(const char* str) {
    while (*str) LCD_data(*str++);
}

/* ---------------- Peripherals ---------------- */
void GPIO_init(void) {
    DDRD &= ~(1 << BUTTON_PIN);
    PORTD |= (1 << BUTTON_PIN); // pull-up

    DDRD |= (1 << BUZZER_PIN);
    PORTD &= ~(1 << BUZZER_PIN);

    DDRB |= (1 << SERVO_PIN) | (1 << LED_RED);
    DDRD |= (1 << LED_YELLOW) | (1 << LED_GREEN);

    PORTB &= ~(1 << LED_RED);
    PORTD &= ~(1 << LED_YELLOW) & ~(1 << LED_GREEN);
}

void Button_interrupt_init(void) {
    EICRA |= (1 << ISC01); // falling edge on INT0
    EIMSK |= (1 << INT0);  // enable INT0
}

ISR(INT0_vect) {
    if ((systicks - last_button_time) > 300) { // 300ms debounce
        button_pressed = 1;
        last_button_time = systicks;
    }
}

void ADC0_init(void) {
    ADMUX = (1 << REFS0);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);
}

uint16_t ADC0_read(void) {
    ADMUX = (ADMUX & 0xF0) | SENSOR_PIN;
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

void buzzer_beep(uint8_t times, uint16_t duration_ms) {
    for (uint8_t i = 0; i < times; i++) {
        PORTD |= (1 << BUZZER_PIN);
        for (uint16_t j = 0; j < duration_ms; j++) _delay_ms(1);
        PORTD &= ~(1 << BUZZER_PIN);
        _delay_ms(200);
    }
}

void servo_set_angle(uint8_t angle) {
    uint16_t pulse = (angle * 11) + 1000;
    OCR1A = pulse;
}

void turn_off_leds(void) {
    PORTB &= ~(1 << LED_RED);
    PORTD &= ~(1 << LED_YELLOW) & ~(1 << LED_GREEN);
}

/* ---------------- Main ---------------- */
int main(void) {
    GPIO_init();
    Button_interrupt_init();
    ADC0_init();
    Timer0_init_systicks();
    Timer1_init_servo_pwm();
    USART0_init(103);
    USART0_use_stdio();
    TWI_init();
    LCD_init();
    sei();

    printf("Warming up sensor...\n");
    LCD_set_cursor(0, 0); LCD_print("Warming up...");
    uint32_t warm_start = systicks;
    uint8_t last_pct = 255;

    while ((systicks - warm_start) < WARMUP_TIME_MS) {
        uint8_t pct = ((systicks - warm_start) * 100) / WARMUP_TIME_MS;
        if (pct != last_pct) {
            char buf[17];
            snprintf(buf, sizeof(buf), "Progress: %3d%%", pct);
            LCD_set_cursor(0, 1); LCD_print(buf);
            printf("Warm-up: %d%%\n", pct);
            last_pct = pct;
        }
        _delay_ms(10);
    }

    LCD_set_cursor(0, 0); LCD_print("Press button to");
    LCD_set_cursor(0, 1); LCD_print("start test...   ");

    while (1) {
        if (button_pressed && !isMeasuring && !measurementDone) {
            button_pressed = 0;
            isMeasuring = 1;
            measurementStart = systicks;
            alcoholValue = 0;
            LCD_set_cursor(0, 0); LCD_print("Measuring...    ");
            LCD_set_cursor(0, 1); LCD_print("                ");
            printf("Measuring started...\n");
        }

        if (isMeasuring) {
            if ((systicks - measurementStart) < READ_TIME_MS) {
                alcoholValue = ADC0_read();
                LCD_set_cursor(0, 1);
                char buf[17];
                snprintf(buf, sizeof(buf), "Value: %4u     ", alcoholValue);
                LCD_print(buf);
                printf("Value: %u\n", alcoholValue);
                _delay_ms(500);
            } else {
                isMeasuring = 0;
                measurementDone = 1;
                turn_off_leds();
                LCD_set_cursor(0, 0); LCD_print("Result:         ");
                LCD_set_cursor(0, 1);

                if (alcoholValue < 500) {
                    PORTD |= (1 << LED_GREEN);
                    buzzer_beep(1, 200);
                    servo_set_angle(90);
                    door_is_open = 1;
                    LCD_print("You can drive   ");
                    printf("Green: You can drive\n");

                } else if (alcoholValue < 570) {
                    PORTD |= (1 << LED_YELLOW);
                    buzzer_beep(3, 300);
                    servo_set_angle(0);
                    LCD_print("Wait 1 hour     ");
                    printf("Yellow: Wait 1 hour\n");

                } else if (alcoholValue < 650) {
                    PORTD |= (1 << LED_YELLOW);
                    buzzer_beep(3, 300);
                    servo_set_angle(0);
                    LCD_print("Wait few hours  ");
                    printf("Yellow: Wait a few hours\n");

                } else {
                    PORTB |= (1 << LED_RED);
                    PORTD |= (1 << BUZZER_PIN);
                    _delay_ms(2000);
                    PORTD &= ~(1 << BUZZER_PIN);
                    servo_set_angle(0);
                    LCD_print("DO NOT DRIVE!   ");
                    printf("Red: DO NOT DRIVE\n");
                }

                _delay_ms(2000);
                turn_off_leds();
                // ===== Sensor Stabilization Phase =====
                LCD_set_cursor(0, 0); LCD_print("Stabilizing...  ");
                LCD_set_cursor(0, 1); LCD_print("Please wait     ");
                printf("Stabilizing sensor...\n");

                uint32_t stabilize_start = systicks;
                while ((systicks - stabilize_start) < 60000) { // wait max 60s
                    uint16_t stab_val = ADC0_read();
                    printf("Stabilizing value: %u\n", stab_val);

                    if (stab_val < 350) {
                        LCD_set_cursor(0, 1); LCD_print("Ready for test  ");
                        printf("Sensor stabilized\n");
                        break;
                    }

                    _delay_ms(500);
                }

                // Safety fallback message if still high
                if (ADC0_read() >= 350) {
                    LCD_set_cursor(0, 0); LCD_print("Sensor active... ");
                    LCD_set_cursor(0, 1); LCD_print("Use with caution ");
                    printf("Sensor did not stabilize in time.\n");
                }
            }
        }

        if (door_is_open) {
            LCD_set_cursor(0, 0); LCD_print("Press to close  ");
            LCD_set_cursor(0, 1); LCD_print("the door...     ");
            while (1) {
                if (button_pressed) {
                    button_pressed = 0;
                    servo_set_angle(0);
                    door_is_open = 0;
                    LCD_set_cursor(0, 0); LCD_print("Door closed     ");
                    LCD_set_cursor(0, 1); LCD_print("Ready for test  ");
                    printf("Door closed\n");
                    break;
                }
            }
        }

        if (!door_is_open) {
            measurementDone = 0;
        }
    }
}
