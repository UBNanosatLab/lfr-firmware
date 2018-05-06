#ifndef MCU_H
#define MCU_H

#include <stdint.h>
#include <stdbool.h>

#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1

#define RISING 0x00
#define FALLING 0x01

#define USE_HFXT

void mcu_init();
void mcu_reset();
void wdt_feed();

void i2c_init();
int i2c_write(unsigned char slave_addr, unsigned char *buf, unsigned char len);

void spi_init();
void spi_write_byte(unsigned char b);
void spi_write_data(int len, unsigned char *data);
void spi_read_data(int len, unsigned char *data);

void gpio_config(int pin, int mode);
void gpio_write(int pin, int value);
int enable_pin_interrupt(int pin, int edge);

void delay_micros(unsigned long micros);

#endif
