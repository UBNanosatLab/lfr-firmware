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

#ifndef _CMD_PARSER_H
#define _CMD_PARSER_H

#include <stdint.h>
#include <stdbool.h>
#define MAX_PAYLOAD_LEN 255

/* sync word */
#define SYNCWORD_H 0xbe
#define SYNCWORD_L 0xef
/* status byte flags */
#define FLAG_TXBUSY (1<<7)
#define FLAG_INVALID (1<<6)
#define FLAG_BADSUM (1<<5)
#define FLAG_GOODCMD (1<<4)
/* commands */
#define CMD_NOP 0
#define CMD_RESET 1
#define CMD_TXDATA 2
#define CMD_READ_TXPWR 3
#define CMD_SET_TXPWR 4
#define CMD_SET_FREQ 5
#define CMD_TX_PSR 6
#define CMD_TX_ABORT 7
#define CMD_GET_CFG 8
#define CMD_SET_CFG 9
#define CMD_SAVE_CFG 10
#define CMD_CFG_DEFAULT 11

/* not really a command, only used in spontaneous reports */
#define CMD_RXDATA 16

#define CMD_START_BER_TEST 32
#define CMD_STOP_BER_TEST 33

#define CMD_ERR 0xFF


#ifdef __cplusplus
extern "C" {
#endif

void parse_char(uint8_t c);

void reply_error(uint8_t sys_stat, uint8_t code);
void reply(uint8_t sys_stat, uint8_t cmd, int len, uint8_t *payload);

#ifdef __cplusplus
}
#endif

#endif
