#include "cmd_parser.h"
//#define USE_PRINTF
#ifdef USE_PRINTF
#include <stdio.h>
#endif

void (*_handle_cmd)(uint8_t, uint8_t, uint8_t*) = default_handler;
void (*_handle_error)(uint8_t) = default_error_handler;

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

/* \fn parse_char(uint8_t c)
 * \brief Character-based command parser
 * \details Takes one character at a time, checks for validity, reports error or executes a completed command
 * \param c The next byte
 */
void parse_char(uint8_t c) {
  static enum parser_state_e next_state = S_SYNC0;
  static enum parser_result_e result = R_WAIT;
  static uint8_t cmd;
  static uint8_t payload_len = 0, payload_counter = 0;
  static uint8_t payload[MAX_PAYLOAD_LEN];
  static uint16_t checksum;
  static uint16_t calc_checksum;

  /* step through the packet structure based on the latest character */
  switch (next_state) {
    case S_SYNC0:
      if (SYNCWORD_H == c) next_state = S_SYNC1;
      break;
    case S_SYNC1:
      if (SYNCWORD_L == c) next_state = S_CMD;
      /* for a sync word "Sy", "SSy" should be detected as valid */
      else if (SYNCWORD_H != c)next_state = S_SYNC0;
      break;
    case S_CMD:
      if (CMD_NOPAYLOAD == validate_cmd(c)) {
        cmd = c;
        payload_len = 0;
        calc_checksum = fletcher(0, c);
        next_state = S_CHECKSUM0;
      } else if (CMD_PAYLOAD == validate_cmd(c)) {
        cmd = c;
        //calc_checksum = fletcher(calc_checksum, c);
        calc_checksum = fletcher(0, c);
        next_state = S_PAYLOADLEN;
      } else result = R_INVALID;
      break;
    case S_PAYLOADLEN:
      if (validate_length(cmd, c)) {
        payload_len = c;
        payload_counter = 0;
        calc_checksum = fletcher(calc_checksum, c);
        next_state = S_PAYLOAD;
      } else result = R_INVALID;
      break;
    case S_PAYLOAD:
      payload[payload_counter] = c;
      calc_checksum = fletcher(calc_checksum, c);
      payload_counter++;
      if (payload_counter == payload_len) {
        next_state = S_CHECKSUM0;
      }
      break;
    case S_CHECKSUM0:
      checksum = ((uint16_t) c) << 8;
      next_state = S_CHECKSUM1;
      break;
    case S_CHECKSUM1:
      checksum = checksum | (uint16_t)c;
      if (calc_checksum == checksum) {
        result = R_ACT;
      } else result = R_BADSUM;
      break;
    default:
      //error();
      break;
  }//switch(c)

  /* if that character created an error or concluded a valid packet, do something */
  switch (result) {
    case R_INVALID:
      next_state = S_SYNC0;
      result = R_WAIT;
      _handle_error(FLAG_INVALID | cmd);
      break;
    case R_BADSUM:
      next_state = S_SYNC0;
      result = R_WAIT;
      _handle_error(FLAG_BADSUM | cmd);
      break;
    case R_ACT:
      next_state = S_SYNC0;
      result = R_WAIT;
      _handle_cmd(cmd, payload_len, payload);
    case R_WAIT:
      break;
  }
}
/* returns positive for valid command types, negative for invalid */
int validate_cmd(uint8_t cmd) {
  switch (cmd) {
    case CMD_NOP:
    case CMD_RESET:
    case CMD_READ_TXPWR:
      return CMD_NOPAYLOAD;
      break;
    case CMD_SET_TXPWR:
    case CMD_TXDATA:
      return CMD_PAYLOAD;
      break;
    default:
      return CMD_INVALID;
  }
}

/* returns 1 if the payload length is valid for the command type, 0 if not */
int validate_length(uint8_t cmd, uint8_t len) {
  switch (cmd) {
    case CMD_SET_TXPWR:
      return 1 == len;
      break;
    case CMD_TXDATA:
      return len > 0;
      break;
    default:
      return 0 == len;
  }
}

/* update the mod-256 Fletcher checksum with the byte c */
uint16_t fletcher(uint16_t old_checksum, uint8_t c) {
  uint8_t lsb, msb;
  lsb = old_checksum;
  msb = (old_checksum >> 8) + c;
  lsb += msb;
  return ((uint16_t) msb<<8) | (uint16_t)lsb;
}

/* do something with the command received. needs to actually exist. */
void default_handler(uint8_t cmd, uint8_t len, uint8_t* payload) {
#ifdef USE_PRINTF
  int i;
  switch(cmd){
	case CMD_NOP:
	  printf("NOP\r\n");
	  break;
	case CMD_TXDATA:
	  printf("TX data: ");
	  for(i=0; i<len; i++){
		  printf("%x ", payload[i]);
	  }
	  printf("\r\n");
	  break;
	default:
	  printf("not in the list\r\n");
  }
#endif
}

void default_error_handler(uint8_t cmd){
#ifdef USE_PRINTF
	printf("reply_nopayload: %x\r\n", cmd);
#endif
}

void set_cmd_handler(void (*fn)(uint8_t, uint8_t, uint8_t*)){
	if(0 != fn){
		_handle_cmd = fn;
	}
}

void set_error_handler(void (*fn)(uint8_t)){
	if(0 != fn){
		_handle_error = fn;
	}
}
