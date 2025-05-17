#ifndef USART_H_
#define USART_H_

#include <stdio.h>

/* 
 * Macro to compute the UBRR register from 
 * CPU frequency & desired baud rate.
 */
#define CALC_USART_UBRR(BAUD) (F_CPU / 16 / (BAUD) - 1)

/* Initialize the USART peripheral. */
void USART0_init(uint16_t ubrr);

/* Replaces the default stdout with our USART implementation. */
void USART0_use_stdio(void);

/*
 * Receives a byte from USART.
 *
 * @return the data byte received;
 */
char USART0_receive(void);

/*
 * Transmit a byte through the USART.
 *
 * @param `data`: the character to send
 */
void USART0_transmit(char data);

/*
 * Trasmits a null-terminated string through the USART.
 *
 * @param str: the string to send
 */
void USART0_print(const char *str);

#endif // USART_H_
