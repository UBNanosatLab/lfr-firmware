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
#include "settings.h"

void cmd_nop() {
    set_cmd_flag(FLAG_GOODCMD);
    reply(sys_stat, CMD_NOP, 0, NULL);
}

void cmd_reset() {
    WDTCTL = 0x0000; // Force WDT reset
}

void cmd_set_txpwr(uint16_t pwr) {
    settings.tx_gate_bias = pwr;
    set_cmd_flag(FLAG_GOODCMD);
    reply(sys_stat, CMD_SET_TXPWR, 0, NULL);
}

void cmd_get_txpwr() {
    uint8_t resp[] = {(uint8_t)(settings.tx_gate_bias >> 8), (uint8_t)(settings.tx_gate_bias & 0xFF)};
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
    // TODO: Should this update settings?
    int err = si446x_set_frequency(&dev, freq);

    if (err) {
        // Halt and catch fire
        reply_error(sys_stat, (uint8_t) -err);
    } else {
        set_cmd_flag(FLAG_GOODCMD);
        reply(sys_stat, CMD_SET_FREQ, 0, NULL);
    }
}

void cmd_abort_tx()
{
    int err;
    err = si446x_recv_async(&dev, 255, buf, rx_cb);
    if (err) {
        reply_error(sys_stat, (uint8_t) -err);
    } else {
        set_cmd_flag(FLAG_GOODCMD);
        reply(sys_stat, CMD_TX_PSR, 0, NULL);
    }
}

void cmd_tx_psr()
{
    int err;
    err = si446x_set_mod_type(&dev, MOD_SRC_RAND | MOD_TYPE_2GFSK);
    if (err) {
        reply_error(sys_stat, (uint8_t) -err);
        return;
    }

    err = si446x_fire_tx(&dev);
    if (err) {
        reply_error(sys_stat, (uint8_t) -err);
    } else {
        set_cmd_flag(FLAG_GOODCMD);
        reply(sys_stat, CMD_TX_PSR, 0, NULL);
    }
}

void cmd_err(int err) {
    reply_error(sys_stat, (uint8_t) err);
}

int reply_putc(uint8_t c) {
    uart_putc(c);
    return 0;
}
