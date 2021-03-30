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

/* commands */
/* System Group */
#define CMD_NOP                 0x00
#define CMD_RESET               0x01
//#define CMD_UPTIME              0x02

/* Data Group */
#define CMD_TXDATA              0x10
#define CMD_RXDATA              0x11
#define CMD_TX_ABORT            0x12
#define CMD_TX_PSR              0x13

/* Configuration Group */
#define CMD_GET_CFG             0x20
#define CMD_SET_CFG             0x21
#define CMD_SAVE_CFG            0x22
#define CMD_CFG_DEFAULT         0x23
#define CMD_SET_FREQ            0x24
#define CMD_READ_TXPWR          0x25
#define CMD_SET_TXPWR           0x26

/* Status Group */
//#define CMD_GET_STATUS          0x30
//#define CMD_CLEAR_STATUS        0x31
#define CMD_GET_QUEUE_DEPTH     0x32
#define CMD_GET_TEMPS           0x33

/* Peripheral Group */
//#define CMD_GPIO_WRITE          0x40

/* Error / User Use Group */
#define CMD_USER0               0x70
#define CMD_USER1               0x71
#define CMD_USER2               0x72
#define CMD_USER3               0x73
#define CMD_USER4               0x74
#define CMD_USER5               0x75
#define CMD_USER6               0x76
#define CMD_USER7               0x77
// ...
#define CMD_INTERNALERR         0x7E
#define CMD_REPLYERR            0x7F

/* Reply bit */
#define CMD_REPLY               0x80


#ifdef __cplusplus
extern "C" {
#endif

void parse_char(uint8_t c);
void send_reply_to_host();
void internal_error(uint8_t code);
void reply_cmd_error(uint8_t code);
void reply(uint8_t cmd, int len, uint8_t *payload);

#ifdef __cplusplus
}
#endif

#endif
