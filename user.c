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
#include <stdlib.h>

#include "cmd_parser.h"
#include "cmd_handler.h"
#include "user.h"
#include "fm.h"
#include "mcu.h"
#include "pins.h"

#include "error.h"
#include "lfr.h"

#pragma PERSISTENT(user_ext0_val)
static uint8_t user_ext0_val = HIGH;
#pragma PERSISTENT(user_ext0_mode)
static uint8_t user_ext0_mode = OUTPUT;

//#pragma PERSISTENT(user_ext1_val)
//static uint8_t user_ext1_val = LOW;
//#pragma PERSISTENT(user_ext1_mode)
//static uint8_t user_ext1_mode = OUTPUT;

#pragma PERSISTENT(num_resets)
static uint16_t num_resets = 0x0000;

static uint32_t uptime_ticks = 0;

const char *hello = "Hello, world!";

void user_init() {

    num_resets++;

    gpio_config(EXT_IO0, user_ext0_mode);
    gpio_write(EXT_IO0, user_ext0_val);

    // gpio_config(EXT_IO1, user_ext1_mode);
    // gpio_write(EXT_IO1, user_ext1_val);

    // We do not allow the setting of the burnwire flag to be sticky!
    // This either kills the satellite by drawing all the power or by 
    // getting stuck in a boot loop
    gpio_config(EXT_IO1, LOW);
    gpio_write(EXT_IO1, OUTPUT);
}

void set_sticky_gpio(uint8_t pin, uint8_t mode, uint8_t val) {
    if (pin == EXT_IO0) {
        user_ext0_mode = mode;
        user_ext0_val = val;
    } else if (pin == EXT_IO1) {
        // NO!
        // We do not allow the setting of the burnwire flag to be sticky!
        // This either kills the satellite by drawing all the power or by 
        // getting stuck in a boot loop
        // user_ext1_mode = mode;
        // user_ext1_val = val;
    }
}

void tx_dead_key()
{
    int err;
    int attempts;

    // So I think this is the packet handler causing problems.
    // For some reason, the Si446x needs to be reset to disable the packet
    // handler. Failure to do this will result in the packet handler exiting
    // the transmit state automatically after about 250 ms. I'm not entirely
    // convinced this is indeed the packet handler, but it only appears to
    // happen after sending a data packet.

    err = reset_si446x();
    if (err) {
        return;
    }

    err = si446x_set_mod_type(&dev, MOD_SRC_RAND | MOD_TYPE_CW);

    if (err) {
        reply_cmd_error((uint8_t) -err);
        return;
    }

    err = pre_transmit();
    if (err) {
        reply_cmd_error((uint8_t) -err);
        return;
    }

    for (attempts = 0; attempts < 3; attempts++) {
        err = si446x_fire_tx(&dev);

        if (err == -ERESETSI) {

            err = reset_si446x();
            if (err) {
                reply_cmd_error((uint8_t) -err);
                return;
            }

            // Retry

        } else if (err) {
            reply_cmd_error((uint8_t) -err);
            return;
        } else {
            reply(CMD_USER4, 0, NULL);
            return;
        }
    }

    // Give up
    reply_cmd_error((uint8_t) ETIMEOUT);
}

void cmd_user(uint8_t cmd, uint8_t len, uint8_t *data)
{
    uint8_t arr[] = {
        (uptime_ticks >> 24) & 0xFF, (uptime_ticks >> 16) & 0xFF,
        (uptime_ticks >> 8) & 0xFF, uptime_ticks & 0xFF
    };

    switch(cmd) {
    case 0:
        reply(CMD_USER0, 13, (uint8_t *)hello);
        break;
    case 1:
        reply(CMD_USER1, 4, arr);
        break;

    case 4:
        tx_dead_key();
        reply(CMD_USER4, 0, NULL);
        break;
    case 5:
        fm_set_msg(len, data);
        fm_start();
        reply(CMD_USER5, 0, NULL);
        break;
    default:
        cmd_err(ECMDINVAL);
    }
}

void user_tick()
{
    uptime_ticks++;
}

int32_t get_uptime_ticks() {
    return uptime_ticks;
}

int16_t get_num_resets() {
    return num_resets;
}
