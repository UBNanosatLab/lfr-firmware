#ifndef _CMD_PARSER_H
#define _CMD_PARSER_H

#include <stdint.h>
#define MAX_PAYLOAD_LEN 255

/* results from validate_cmd */
#define CMD_NOPAYLOAD 1
#define CMD_PAYLOAD 2
#define CMD_INVALID -1

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
/* not really a command, only used in spontaneous reports */
#define CMD_RXDATA 16 


#ifdef __cplusplus
extern "C" {
#endif

void parse_char(uint8_t c);
int validate_cmd(uint8_t cmd);
int validate_length(uint8_t cmd, uint8_t len);
uint16_t fletcher(uint16_t old_checksum, uint8_t c);

void default_handler(uint8_t cmd, uint8_t len, uint8_t* payload);
extern void (*_handle_cmd)(uint8_t, uint8_t, uint8_t*);
void set_cmd_handler(void (*fn)(uint8_t, uint8_t, uint8_t*));

//void reply_nopayload(uint8_t cmd);
extern void (*_handle_error)(uint8_t);
void default_error_handler(uint8_t);
void set_error_handler(void(*fn)(uint8_t));


#ifdef __cplusplus
}
#endif

#endif
