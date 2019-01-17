/* Little Free Radio - An Open Source Radio for CubeSats
 * Copyright (C) 2018 Grant Iraci, Brian Bezanson
 * A project of the University at Buffalo Nanosatellite Laboratory
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef MCU_H
#define MCU_H

#include <stdint.h>
#include <stdbool.h>

#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x30
#define INPUT_PULLDOWN 0x20

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
void spi_write_data(int len, const unsigned char *data);
void spi_read_data(int len, unsigned char *data);

void gpio_config(int pin, int mode);
void gpio_write(int pin, int value);
unsigned char gpio_read(int pin);
int enable_pin_interrupt(int pin, int edge);

void delay_micros(unsigned long micros);

void get_device_uid(unsigned char* buf);

#endif
