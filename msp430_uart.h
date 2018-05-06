/*
 * msp430_uart.h
 *
 *  Created on: Feb 13, 2018
 *      Author: brian
 */
int uart_init();
void uart_init_pins();
int uart_putc(char c);
int uart_putbuf(char* str, int len);
int uart_available();
char uart_getc();
