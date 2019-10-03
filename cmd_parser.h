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

/* Peripheral Group */
//#define CMD_GPIO_WRITE          0x40

/* Error / Internal Use Group */
#define CMD_INTERNALERR         0x7E
#define CMD_REPLYERR            0x7F

/* Reply bit */
#define CMD_REPLY               0x80


#ifdef __cplusplus
extern "C" {
#endif


/**
 * enum for states of the byte parser state machine
 * Each state is named for the byte which the state machine expects to receive.
 */
enum parser_state_e {S_SYNC0, S_SYNC1, S_CMD, S_PAYLOADLEN, S_PAYLOAD, S_CHECKSUM0, S_CHECKSUM1};

/**
 * enum for the result of parsing a byte
 * The results can be:
 * wait for another character (take no action)
 * execute the command (if the byte completed a valid command)
 * invalid (the command was not valid, or the payload length was not valid for the command)
 * bad checksum (checksum mismatch)
 */
enum parser_result_e {R_WAIT, R_ACT, R_INVALID, R_BADSUM};

typedef struct {
    enum parser_state_e next_state;
    enum parser_result_e result;
    uint8_t cmd;
    uint8_t payload_len, payload_counter;
    uint8_t payload[MAX_PAYLOAD_LEN];
    uint16_t checksum;
    uint16_t calc_checksum;
} cmd_parser;

void parser_init(cmd_parser *cp);
void parse_char(cmd_parser *cp, uint8_t c);
void send_reply_to_host();
void internal_error(uint8_t code);
void reply_cmd_error(uint8_t code);
void reply(uint8_t cmd, int len, uint8_t *payload);

#ifdef __cplusplus
}
#endif

#endif
