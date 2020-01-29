/*
 * simple-ota.h
 *
 *  Created on: Feb 13, 2019
 *      Author: brian
 */

#ifndef SIMPLE_OTA_H_
#define SIMPLE_OTA_H_
#include <stdint.h>

#define OTA_CMD_PING (0)
#define OTA_CMD_GPIO (1)
#define OTA_CMD_TX_INHIBIT (2)
#define OTA_CMD_TX_RELEASE (3)
#define OTA_CMD_GET_RESETS (4)
#define OTA_CMD_GET_WDT_RESETS (5)
#define OTA_CMD_GET_UPTIME (6)

#define OTA_FLAG 0xDB
#define OTA_KEY "LinkSat=BestSat"
#define OTA_KEY_LEN (16)
#define OTA_CMD_OFFSET (16)
#define OTA_PAYLOAD_LEN_OFFSET (17)
#define OTA_PAYLOAD_OFFSET (18)
#define OTA_MIN_LEN (20) //Minimum length is 1 flag + 15 key + 1 command + 1 payload length + 2 checksum
/*
 * Over-the-air command structure:
 * OTA flag (0xDB)
 * OTA key [15 bytes, ASCII "LinkSat=BestSat"
 * Command [1 byte]
 * Payload length [1 byte, 0 for no-payload commands]
 * (Optional payload, <235 bytes)
 * Checksum [2 bytes] evaluated over command, payload length, and payload
 *
 */

void ota_handler(uint8_t* pkt, unsigned int len);

#endif /* SIMPLE_OTA_H_ */
