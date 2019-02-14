/*
 * simple-ota.c
 *
 *  Created on: Feb 13, 2019
 *      Author: brian
 */

#include "simple-ota.h"
#include "mcu.h"
#include "lfr.h"
#include "lib446x/si446x.h"
#include "cmd_parser.h"
#include "checksum.h"

uint8_t ota_key[] = OTA_KEY;

int get_ota_cmd_payload_len(uint8_t cmd){
    switch(cmd){
    case OTA_CMD_PING:
        return 0;

    case OTA_CMD_GPIO:
    case OTA_CMD_TX_INHIBIT:
    case OTA_CMD_TX_RELEASE:
    case OTA_CMD_GET_RESETS:
    case OTA_CMD_GET_WDT_RESETS:
    case OTA_CMD_GET_UPTIME:
        return -ENOTIMPL;

    default:
        return -EINVAL;
    }
}

void ota_handler(uint8_t* pkt, unsigned int len){
    unsigned int i=0;
    int pay_len=-1;
    unsigned int calc_checksum=0;
    uint8_t cmd=0;
    //Check that it's safe to access the header portions of pkt
    if(!pkt || len < OTA_MIN_LEN){
        return;
    }
    // check the flag again
    if(pkt[0] != OTA_FLAG){
        return;
    }
    //check the rest of the key
    for(i=1; i<15; i++){
        if(pkt[i] != ota_key[i-1]){
            return;
        }
    }
    //check that the command is valid and len matches the expected packet length for that command
    cmd = pkt[16];
    pay_len = get_ota_cmd_payload_len(cmd);
    if(pay_len < 0 || len != pay_len + OTA_MIN_LEN){
        return;
    }
    //Calculate the checksum over the command byte, payload length, and payload
    for(i=16; i<len-2; i++){
        calc_checksum = fletcher(calc_checksum, pkt[i]);
    }
    //Compare the checksum calculated to the one in the packet
    if(calc_checksum != (pkt[len-2]<<8)|pkt[len-1]){
        return;
    }
    //Okay, command, length, and checksum all look good
    switch(cmd){
    case OTA_CMD_PING:
        sys_stat |= FLAG_TXBUSY;
        uint8_t resp[] = {
                'P',
                'O',
                'I',
                'O',
                'I',
                'N',
                'G',
            };
        if( pre_transmit() ){
            si446x_setup_tx(&dev, sizeof(resp), resp, tx_cb);
            do_pong = true;
        } else {
            sys_stat &= ~(FLAG_TXBUSY);
        }
        break;
    default:
        return;
    }
}
