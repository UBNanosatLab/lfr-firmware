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
#include <stdint.h>

#include "settings.h"
#include "radio.h"

#define DEFAULT_SETTINGS {\
    .freq = 434000000L,   \
    .modem_config = DATA_10K_DEV_5K,    \
    .tcxo_vpull = 1256,   \
    .tx_gate_bias = 500,  /* 0.5V bias is enough to see in testing, but very very low output power */ \
    .tx_vdd = 5000,       /* 5V will wake the PA up, but dissipate little heat even with a lot of gate bias */ \
    .pa_ilimit = 1000,    /* 1A is a bit more than the RA07H4047M draws at Vdd=5V, Vgg=3.5V, Pin=13dBm */ \
    .tx_vdd_delay = 2000, /* 2ms is plenty for the example 5V/1A values */ \
    .flags = FLAG_CRC_CHECK,\
    .callsign = "NOCALL  "\
};


const char sw_ver[] = __DATE__ " " __TIME__;

const struct lfr_board_info board_info = {
//    .fw_git_hash = 0,
    .sw_ver = sw_ver,
    .serial_no = 0x0000
};

const struct lfr_settings default_settings = DEFAULT_SETTINGS;

struct lfr_settings saved_settings __attribute__((persistent)) = DEFAULT_SETTINGS;

// Must call settings_load_xxx() before use!!
struct lfr_settings settings = {0};

int settings_load_default()
{
    unsigned short state = __get_interrupt_state();
    __disable_interrupt();
    memcpy(&settings, &default_settings, sizeof(settings));
    __set_interrupt_state(state);
    return 0;
}

int settings_load_saved()
{
    unsigned short state = __get_interrupt_state();
    __disable_interrupt();
    memcpy(&settings, &saved_settings, sizeof(settings));
    __set_interrupt_state(state);
    return 0;
}

int settings_save()
{
    unsigned short state = __get_interrupt_state();
    __disable_interrupt();
    memcpy(&saved_settings, &settings, sizeof(settings));
    __set_interrupt_state(state);
    return 0;
}

// TODO: TEST THIS!!
uint64_t settings_get_silicon_id()
{
#if defined (__MSP430FR5994__)
    return *(uint64_t *)(0x00001A0A);
#else
#warning "Silicon ID not available on this chipset!"
    return 0L;
#endif
}


