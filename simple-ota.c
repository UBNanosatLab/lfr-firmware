/*
 * simple-ota.c
 *
 *  Created on: Feb 13, 2019
 *      Author: brian
 */

#include "simple-ota.h"
#include "mcu.h"
#include "lfr.h"
#include "status.h"
#include "amp.h"
#include "adc.h"
#include "settings.h"
#include "lib446x/si446x.h"
#include "cmd_parser.h"
#include "cmd_handler.h"
#include "checksum.h"
#include "pins.h"
#include "user.h"
#include "fm.h"

uint8_t ota_key[] = OTA_KEY;
void handle_gpio(uint8_t pin, uint8_t mode, uint8_t value);
void handle_sticky_gpio(uint8_t pin, uint8_t mode, uint8_t value);
void send_pong();
void handle_reset();
void handle_uptime();
void handle_adc(uint8_t chan);
void handle_tx_pwr(uint16_t gate_bias);
void handle_fm(uint8_t *data, uint8_t len);
void handle_rssi();
void handle_get_temps();


bool validate_ota_cmd(uint8_t cmd, uint8_t len)
{
    switch(cmd){
    case OTA_CMD_PING:
        return len == 0;
    case OTA_CMD_RESET_LFR:
        return len == 0;
    case OTA_CMD_GPIO:
        return len == 3;
    case OTA_CMD_ADC:
        return len == 1;
    case OTA_CMD_SET_TX_PWR:
        return len == 1;
    case OTA_CMD_FM_VOICE:
        // 127 as an upper bound is kinda arbitrary, but I don't want to send
        // a very long message
        return len > 0 && len < 128;
    case OTA_CMD_UPTIME:
        return len == 0;
    case OTA_CMD_READ_RSSI:
        return len == 0;
    case OTA_CMD_STICKY_GPIO:
        return len == 3;
    case OTA_CMD_GET_TEMPS:
        return len == 0;
    default:
        return false; // Invalid command
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
    for(i=1; i<OTA_KEY_LEN; i++){
        if(pkt[i] != ota_key[i-1]){
            return;
        }
    }
    //check that the command is valid and len matches the expected packet length for that command
    cmd = pkt[OTA_CMD_OFFSET];
    pay_len = pkt[OTA_PAYLOAD_LEN_OFFSET];
    if((len != pay_len + OTA_MIN_LEN) || !validate_ota_cmd(cmd, pay_len)) {
        return;
    }


    //Calculate the checksum over the command byte, payload length, and payload
    for(i=OTA_CMD_OFFSET; i<len-2; i++){
        calc_checksum = fletcher(calc_checksum, pkt[i]);
    }
    //Compare the checksum calculated to the one in the packet
    if(calc_checksum != ((pkt[len-2]<<8)|pkt[len-1])){
        return;
    }
    //Okay, command, length, and checksum all look good
    switch(cmd){
    case OTA_CMD_PING:
        send_pong();
        break;
    case OTA_CMD_RESET_LFR:
        handle_reset();
        break;
    case OTA_CMD_GPIO:
        handle_gpio(pkt[OTA_PAYLOAD_OFFSET],pkt[OTA_PAYLOAD_OFFSET+1],pkt[OTA_PAYLOAD_OFFSET+2]);
        break;
    case OTA_CMD_ADC:
        handle_adc(pkt[OTA_PAYLOAD_OFFSET]);
        break;
    case OTA_CMD_SET_TX_PWR:
        handle_tx_pwr(pkt[OTA_PAYLOAD_OFFSET]<<8 | pkt[OTA_PAYLOAD_OFFSET+1]);
        break;
    case OTA_CMD_FM_VOICE:
        handle_fm(pkt + OTA_PAYLOAD_OFFSET, pay_len);
        break;
    case OTA_CMD_UPTIME:
        handle_uptime();
        break;
    case OTA_CMD_READ_RSSI:
        handle_rssi();
        break;
    case OTA_CMD_GET_TEMPS:
        handle_get_temps();
        break;
    case OTA_CMD_STICKY_GPIO:
        handle_sticky_gpio(pkt[OTA_PAYLOAD_OFFSET],pkt[OTA_PAYLOAD_OFFSET+1],pkt[OTA_PAYLOAD_OFFSET+2]);
        break;
    default:
        return;
    }
}

void send_pong()
{
    uint8_t resp[] = {
        OTA_FLAG,       // OTA HEADER
        OTA_CMD_PING,   // OTA COMMAND
        4,              // REPLY LEN
        'P',            // REPLY DATA
        'O',
        'N',
        'G',
    };
    set_status(STATUS_TXBUSY, true);
    gpio_write(TX_ACT_PIN, HIGH);
    set_gate_bias(settings.tx_gate_bias);
    si446x_setup_tx(&dev, sizeof(resp), resp, tx_cb);
    do_pong = true;
}

void handle_reset()
{
    // Use the reset command handler
    cmd_reset();
}

void handle_gpio(uint8_t pin, uint8_t mode, uint8_t value)
{
    uint8_t iopin=0;
    if(pin == 0) {
        iopin=EXT_IO0;
    } else if (pin == 1) {
        iopin=EXT_IO1;
    } else {
        return;
    }

    if(mode != OUTPUT && mode != INPUT && mode != INPUT_PULLUP && mode != INPUT_PULLDOWN){
       return;
    }

    if((mode == OUTPUT || mode == INPUT_PULLUP || mode == INPUT_PULLDOWN) && (value == 1 || value == 0)){
       gpio_config(iopin, mode);
       gpio_write(iopin, value);
    } else {
        gpio_config(iopin, mode);
    }
}


void handle_sticky_gpio(uint8_t pin, uint8_t mode, uint8_t value)
{
    uint8_t iopin=0;
    if(pin == 0) {
        iopin=EXT_IO0;
    } else if (pin == 1) {
        iopin=EXT_IO1;
    } else {
        return;
    }

    if(mode != OUTPUT && mode != INPUT && mode != INPUT_PULLUP && mode != INPUT_PULLDOWN){
       return;
    }

    set_sticky_gpio(iopin, mode, value);

    if((mode == OUTPUT || mode == INPUT_PULLUP || mode == INPUT_PULLDOWN) && (value == 1 || value == 0)){
       gpio_config(iopin, mode);
       gpio_write(iopin, value);
    } else {
        gpio_config(iopin, mode);
    }
}

void handle_adc(uint8_t chan)
{
    int result = adc_read(chan);

    if (result > 0) {
        uint8_t resp[] = {
            OTA_FLAG,               // OTA HEADER
            OTA_CMD_ADC,            // OTA COMMAND
            2,                      // REPLY LEN
            (result >> 8) & 0xFF,   // REPLY DATA
            result & 0xFF,
        };

        send_w_retry(sizeof(resp), resp);
    }
}

void handle_tx_pwr(uint16_t gate_bias)
{
    settings.tx_gate_bias = gate_bias;
}

void handle_fm(uint8_t *data, uint8_t len) {
    fm_set_msg(len, data);
    fm_start();
}

void handle_uptime()
{
    int32_t uptime = get_uptime_ticks();
    int16_t resets = get_num_resets();

    uint8_t resp[] = {
        OTA_FLAG,               // OTA HEADER
        OTA_CMD_UPTIME,         // OTA COMMAND
        6,                      // REPLY LEN
        (uptime >> 24) & 0xFF,  // REPLY DATA
        (uptime >> 16) & 0xFF,
        (uptime >> 8) & 0xFF,
        uptime & 0xFF,
        (resets >> 8) & 0xFF,
        resets & 0xFF,
    };

    send_w_retry(sizeof(resp), resp);
}

void handle_rssi() {

    uint8_t rssi = 255;
    uint8_t rssi_latch = 255;
    delay_micros(800); // 4 bit periods plus margin
    si446x_read_rssi(&dev, &rssi, &rssi_latch);

    uint8_t resp[] = {
        OTA_FLAG,               // OTA HEADER
        OTA_CMD_READ_RSSI,           // OTA COMMAND
        2,                      // REPLY LEN
        rssi,                   // REPLY DATA
        rssi_latch,
    };

    send_w_retry(sizeof(resp), resp);

}

//Calibration values for internal temp sensor from TLV
#define ADC_2V5_30C_CAL (*(int*) 0x01A22)
#define ADC_2V5_85C_CAL (*(int*) 0x01A24)

void handle_get_temps(){
    int32_t mcu_temp_scale = (8500L-3000L)/(ADC_2V5_85C_CAL - ADC_2V5_30C_CAL);
    int16_t mcu_temp = 0;
    int16_t pa_temp = 0;
    mcu_temp = adc_read(ADC_CHAN_TEMP);
    pa_temp = adc_read(PA_TEMP_ACHAN);
    mcu_temp = 3000+(mcu_temp - ADC_2V5_30C_CAL) * mcu_temp_scale;
    //TMP235, 10mV/C, 500mV at 0C, so 4095 means 2.5V means 200C, 0 means 0V means -50C
    pa_temp =((pa_temp * 25000L)>>12) - 5000;
    uint8_t resp[] = {
       OTA_FLAG,               // OTA HEADER
       OTA_CMD_GET_TEMPS,      // OTA COMMAND
       4,                      // REPLY LEN
       pa_temp >> 8,
       pa_temp & 0xFF,

       mcu_temp >> 8,
       mcu_temp & 0xFF
    };

    send_w_retry(sizeof(resp), resp);
}

