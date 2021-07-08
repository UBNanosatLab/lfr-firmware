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

#ifndef _USER_H
#define _USER_H

#include <stdint.h>
#include <stdbool.h>

// Called once at startup
void user_init();

// Called with cmd = [0,7] when CMD_USER0..CMD_USER7 are received
void cmd_user(uint8_t cmd, uint8_t len, uint8_t *data);

// Called periodically, once every 50ms
void user_tick();

// Set user GPIO power on defaults
void set_sticky_gpio(uint8_t pin, uint8_t mode, uint8_t val);

// Get number of ticks (50 ms intervals) since boot
int32_t get_uptime_ticks();

// Get number of resets that have occurred since flashing
int16_t get_num_resets();

#endif
