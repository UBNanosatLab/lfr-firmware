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

#include <msp430.h>
#include <stdlib.h>

#include "radio.h"
#include "mcu.h"
#include "lib446x/si446x.h"
#include "cmd_handler.h"
#include "cmd_parser.h"
#include "msp430_uart.h"
#include "board.h"

void cmd_nop() {
    set_cmd_flag(FLAG_GOODCMD);
    reply(sys_stat, CMD_NOP, 0, NULL);
}

void cmd_reset() {
    WDTCTL = 0x0000; // Force WDT reset
}

void cmd_set_txpwr(uint16_t pwr) {
    tx_gate_bias = pwr;
    set_cmd_flag(FLAG_GOODCMD);
    reply(sys_stat, CMD_SET_TXPWR, 0, NULL);
}

void cmd_get_txpwr() {
    uint8_t resp[] = {(uint8_t)(tx_gate_bias >> 8), (uint8_t)(tx_gate_bias & 0xFF)};
    set_cmd_flag(FLAG_GOODCMD);
    reply(sys_stat, CMD_READ_TXPWR, sizeof(resp), resp);
}

void cmd_tx_data(int len, uint8_t *data) {
    memcpy(buf, data, len);

    sys_stat |= FLAG_TXBUSY;
    pre_transmit();

    int err = si446x_send_async(&dev, len, buf, tx_cb);

    if (err) {
        // Halt and catch fire
        reply_error(sys_stat, (uint8_t) -err);
    } else {
        set_cmd_flag(FLAG_GOODCMD);
        reply(sys_stat, CMD_TXDATA, 0, NULL);
    }
}

void cmd_set_freq(uint32_t freq) {
    int err = si446x_set_frequency(&dev, freq);

    if (err) {
        // Halt and catch fire
        reply_error(sys_stat, (uint8_t) -err);
    } else {
        set_cmd_flag(FLAG_GOODCMD);
        reply(sys_stat, CMD_SET_FREQ, 0, NULL);
    }
}

void start_ber_test(){
    int err;
    //Set up for direct receiving on GPIO1/GPIO2 for BER test
    ber_data[0] = ber_data[1] = 0;
    ber_i = ber_bitcount = 0;
    ber_synced = ber_send = false;
    ber_test = true;
    gpio_config(GPIO1, INPUT_PULLDOWN);
    gpio_config(GPIO2, INPUT_PULLDOWN);
    //RX_DATA_CLOCK pin
    enable_pin_interrupt(GPIO2, RISING);
    err = si446x_cfg_gpio(&dev, GPIO_SYNC_WORD_DETECT, GPIO_RX_DATA, GPIO_RX_DATA_CLK, GPIO_RX_STATE);
    if (err) {
        reply_error(sys_stat, (uint8_t) -err);
    }else {
        set_cmd_flag(FLAG_GOODCMD);
        reply(sys_stat, CMD_START_BER_TEST, 0, NULL);
    }
}

void stop_ber_test(){
    ber_test = ber_send = false;
    disable_pin_interrupt(GPIO2);
    int err;
    //return RFIC GPIOs to normal state
    err = si446x_cfg_gpio(&dev, GPIO_SYNC_WORD_DETECT, GPIO_TX_DATA, GPIO_TX_DATA_CLK, GPIO_RX_STATE);
    if (err) {
        reply_error(sys_stat, (uint8_t) -err);
    }else {
        set_cmd_flag(FLAG_GOODCMD);
        reply(sys_stat, CMD_STOP_BER_TEST, 0, NULL);
    }
}

void cmd_err(int err) {
    reply_error(sys_stat, (uint8_t) err);
}

int reply_putc(uint8_t c) {
    uart_putc(c);
    return 0;
}
