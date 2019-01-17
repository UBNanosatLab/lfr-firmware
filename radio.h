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

#ifndef MAIN_H
#define MAIN_H

#include <stdbool.h>
#include <stdint.h>

/* Global variables */
extern struct si446x_device dev;
extern uint8_t buf[255];
extern volatile bool do_pong;
extern uint16_t tx_gate_bias;
extern uint8_t sys_stat;
// Bit error rate test control variables
extern bool ber_test;
extern volatile uint64_t ber_data[2];
extern volatile uint8_t ber_i;
extern volatile bool ber_send;
extern volatile uint8_t ber_bitcount;
extern volatile bool ber_synced;

void set_cmd_flag(uint8_t flag);
int pre_transmit();
void rx_cb(struct si446x_device *dev, int err, int len, uint8_t *data);
void tx_cb(struct si446x_device *dev, int err);

#endif
