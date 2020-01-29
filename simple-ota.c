/*
 * simple-ota.c
 *
 *  Created on: Feb 13, 2019
 *      Author: brian
 */

#include <msp430.h>
#include "simple-ota.h"
#include "mcu.h"
#include "lfr.h"
#include "lib446x/si446x.h"
#include "cmd_parser.h"
#include "checksum.h"
#include "pins.h"

uint8_t ota_key[] = OTA_KEY;
void handle_gpio(uint8_t pin, uint8_t mode, uint8_t value);

void send_pong(){
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
    //TODO: handle errors from this?
    send_w_retry(sizeof(resp), resp);
}

int get_ota_cmd_payload_len(uint8_t cmd){
    switch(cmd){
    case OTA_CMD_PING:
        return 0;
    case OTA_CMD_GPIO:
        return 3;
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
    for(i=1; i<OTA_KEY_LEN; i++){
        if(pkt[i] != ota_key[i-1]){
            return;
        }
    }
    //check that the command is valid and len matches the expected packet length for that command
    cmd = pkt[OTA_CMD_OFFSET];
    pay_len = get_ota_cmd_payload_len(cmd);
    if(pay_len < 0 || (len != pay_len + OTA_MIN_LEN) || (pay_len != pkt[OTA_PAYLOAD_LEN_OFFSET]) ){
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
    case OTA_CMD_GPIO:
        handle_gpio(pkt[OTA_PAYLOAD_OFFSET],pkt[OTA_PAYLOAD_OFFSET+1],pkt[OTA_PAYLOAD_OFFSET+2]);
    default:
        return;
    }
}

// Pulse GPIOs high for twenty half-second ticks
#define GPIO_PULSE_TICKS 19
volatile uint8_t gpio_0_pulse_counter=0;
volatile uint8_t gpio_1_pulse_counter=0;

void start_gpio_timer(uint8_t pin){

    if(pin == 0){
        gpio_0_pulse_counter = GPIO_PULSE_TICKS;
    } else if(pin == 1){
        gpio_1_pulse_counter = GPIO_PULSE_TICKS;
    } else return;

    if(! (TA2CTL & MC_3)){ //if the timer is stopped
        // 8 MHz clock, div by 64, count to 62500 gives 500 ms period
        TA2CCR0 = 62500;
        // Enable compare interrupt for CCR0
        TA2CCTL0 = CCIE;
        // Set expanded divider to 8
        TA2EX0 = TAIDEX__8;
        // source from SMCLK, div by 8, count up to CCR0 value, clear state
        TA2CTL = TASSEL__SMCLK | ID__8 | MC__UP | TACLR;
    }

}

void handle_gpio(uint8_t pin, uint8_t mode, uint8_t value){
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
    //For balloon cut-down, only drive output high for a finite duration
    if( mode == OUTPUT && value == 1){
        start_gpio_timer(pin);
    }
}


#pragma vector=TIMER2_A0_VECTOR //CCR0 interrupt for TA2
__interrupt void gpio_pulse_isr(){

    if(!(gpio_0_pulse_counter || gpio_0_pulse_counter)){
        TA2CTL &= ~MC_3; //set MC bits to zero, i.e. put counter into sleep mode
    }

    if(gpio_0_pulse_counter){
        gpio_0_pulse_counter--;
    } else {
        //at timeout, return IO0 to hi-Z
        //gpio_config(EXT_IO0, INPUT);
        gpio_write(EXT_IO0, 0);
    }

    if(gpio_1_pulse_counter){
       gpio_1_pulse_counter--;
    } else {
        //at timeout, return IO1 to hi-Z
        //gpio_config(EXT_IO1, INPUT);
        gpio_write(EXT_IO1, 0);
    }

}
