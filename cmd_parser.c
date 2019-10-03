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

#include "cmd_parser.h"
#include "cmd_handler.h"

#include "error.h"
#include "lfr.h"

void command_handler(uint8_t cmd, uint8_t len, uint8_t* payload);

bool validate_cmd(uint8_t cmd);
bool validate_length(uint8_t cmd, uint8_t len);
uint16_t fletcher(uint16_t old_checksum, uint8_t c);

void parser_init(cmd_parser *cp){
    cp->next_state = S_SYNC0;
    cp->result = R_WAIT;
    cp->payload_len = 0;
    cp->payload_counter = 0;
}

/* \fn parse_char(uint8_t c)
 * \brief Character-based command parser
 * \details Takes one character at a time, checks for validity, reports error or executes a completed command
 * \param c The next byte
 */
void parse_char(cmd_parser *cp, uint8_t c) {


  /* step through the packet structure based on the latest character */
  switch (cp->next_state) {
    case S_SYNC0:
      if (SYNCWORD_H == c) cp->next_state = S_SYNC1;
      break;
    case S_SYNC1:
      if (SYNCWORD_L == c) cp->next_state = S_CMD;
      /* for a sync word "Sy", "SSy" should be detected as valid */
      else if (SYNCWORD_H != c) cp->next_state = S_SYNC0;
      break;
    case S_CMD:
      if (validate_cmd(c)) {
        cp->cmd = c;
        //calc_checksum = fletcher(calc_checksum, c);
        cp->calc_checksum = fletcher(0, c);
        cp->next_state = S_PAYLOADLEN;
      } else {
        cp->result = R_INVALID;
      }
      break;
    case S_PAYLOADLEN:
      if (validate_length(cp->cmd, c)) {
          cp->payload_len = c;
          cp->payload_counter = 0;
          cp->calc_checksum = fletcher(cp->calc_checksum, c);
        if (cp->payload_len) {
            cp->next_state = S_PAYLOAD;
        } else {
            cp->next_state = S_CHECKSUM0;
        }
      } else cp->result = R_INVALID;
      break;
    case S_PAYLOAD:
        cp->payload[cp->payload_counter] = c;
        cp->calc_checksum = fletcher(cp->calc_checksum, c);
        cp->payload_counter++;
      if (cp->payload_counter == cp->payload_len) {
          cp->next_state = S_CHECKSUM0;
      }
      break;
    case S_CHECKSUM0:
        cp->checksum = ((uint16_t) c) << 8;
        cp->next_state = S_CHECKSUM1;
      break;
    case S_CHECKSUM1:
        cp->checksum = cp->checksum | (uint16_t)c;
      if (cp->calc_checksum == cp->checksum) {
          cp->result = R_ACT;
      } else cp->result = R_BADSUM;
      break;
    default:
      //error();
      break;
  }//switch(c)

  /* if that character created an error or concluded a valid packet, do something */
  switch (cp->result) {
    case R_INVALID:
        cp->next_state = S_SYNC0;
        cp->result = R_WAIT;
        cmd_err(ECMDINVAL);
      break;
    case R_BADSUM:
        cp->next_state = S_SYNC0;
        cp->result = R_WAIT;
      cmd_err(ECMDBADSUM);
      break;
    case R_ACT:
        cp->next_state = S_SYNC0;
        cp->result = R_WAIT;
      command_handler(cp->cmd, cp->payload_len, cp->payload);
      send_reply_to_host();
    case R_WAIT:
      break;
  }
}
/* returns true for valid command types, false for invalid */
bool validate_cmd(uint8_t cmd) {
  switch (cmd) {
    case CMD_NOP:
    case CMD_RESET:
    case CMD_READ_TXPWR:
    case CMD_SET_TXPWR:
    case CMD_TXDATA:
    case CMD_TX_PSR:
    case CMD_TX_ABORT:
    case CMD_SET_FREQ:
    case CMD_GET_CFG:
    case CMD_SET_CFG:
    case CMD_SAVE_CFG:
    case CMD_CFG_DEFAULT:
    case CMD_GET_QUEUE_DEPTH:
      return true;
    default:
      return false;
  }
}

/* returns true if the payload length is valid for the command type, false if not */
bool validate_length(uint8_t cmd, uint8_t len) {
  if (len > MAX_PAYLOAD_LEN) {
    return false;
  }

  switch (cmd) {
    case CMD_SET_TXPWR:
      return len == 2;
    case CMD_TXDATA:
      return len > 0;
    case CMD_SET_FREQ:
      return len == 4;
    case CMD_SET_CFG:
          return len > 0; // Allow any non-zero here, we check it in the cmd callback
    default:
      return len == 0;
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

void command_handler(uint8_t cmd, uint8_t len, uint8_t* payload) {

    switch (cmd) {
      case CMD_NOP:
        cmd_nop();
        break;
      case CMD_RESET:
        cmd_reset();
        break;
      case CMD_READ_TXPWR:
        cmd_get_txpwr();
        break;
      case CMD_SET_TXPWR:
        cmd_set_txpwr((uint16_t) payload[0] << 8 | payload[1]);
        break;
      case CMD_TXDATA:
        cmd_tx_data(len, payload);
        break;
      case CMD_SET_FREQ:
        cmd_set_freq((uint32_t) payload[0] << 24 | (uint32_t) payload[1] << 16 | (uint32_t) payload[2] << 8 |
                     payload[3]);
        break;
      case CMD_TX_PSR:
        cmd_tx_psr();
        break;
      case CMD_TX_ABORT:
        cmd_abort_tx();
        break;
      case CMD_GET_CFG:
          cmd_get_cfg();
          break;
      case CMD_SET_CFG:
          cmd_set_cfg(len, payload);
        break;
      case CMD_SAVE_CFG:
          cmd_save_cfg();
          break;
      case CMD_CFG_DEFAULT:
          cmd_cfg_default();
          break;
      case CMD_GET_QUEUE_DEPTH:
          cmd_get_queue_depth();
          break;
    }
}

// Place to hold the current command reply
uint8_t reply_command = CMD_INTERNALERR;
uint8_t reply_buffer[255];
uint8_t reply_length = 0;

// Always goes to flight computer interface
void internal_error(uint8_t code)
{
    host_reply_putc(SYNCWORD_H);
    host_reply_putc(SYNCWORD_L);
    uint16_t chksum = 0;
    host_reply_putc(CMD_INTERNALERR);
    chksum = fletcher(chksum, CMD_INTERNALERR);
    host_reply_putc(1);
    chksum = fletcher(chksum, 1);
    host_reply_putc(code);
    chksum = fletcher(chksum, code);
    host_reply_putc((char) (chksum >> 8));
    host_reply_putc((char) chksum);
}

void send_reply_to_host()
{
    host_reply_putc(SYNCWORD_H);
    host_reply_putc(SYNCWORD_L);
    uint16_t chksum = 0;
    host_reply_putc(reply_command);
    chksum = fletcher(chksum, reply_command);

    host_reply_putc(reply_length);
    chksum = fletcher(chksum, reply_length);

    uint8_t *payload = reply_buffer;
    int len = reply_length;
    for (; len > 0; len--) {
        host_reply_putc(*payload);
        chksum = fletcher(chksum, *payload);
        payload++;
    }

    host_reply_putc((char) (chksum >> 8));
    host_reply_putc((char) chksum);
}


void reply_cmd_error(uint8_t code)
{
    reply_command = (CMD_REPLY | CMD_REPLYERR);
    reply_length = 1;
    reply_buffer[0] = code;
}

void reply(uint8_t cmd, int len, uint8_t *payload)
{
    reply_command = (CMD_REPLY | cmd);
    reply_length = len;
    memcpy(reply_buffer, payload, len);
}
