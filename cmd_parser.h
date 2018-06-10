#ifndef _CMD_PARSER_H
#define _CMD_PARSER_H

#include <stdint.h>
#include <stdbool.h>
#define MAX_PAYLOAD_LEN 255

#define ECMDBADSUM 0x16
#define ECMDINVAL  0x13

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
/* not really a command, only used in spontaneous reports */
#define CMD_RXDATA 16

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
