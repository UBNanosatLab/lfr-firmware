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

#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdbool.h>
#include <stdint.h>

#define SETTINGS_VER 0x02

#define FLAG_CRC_CHECK 0x0001

struct lfr_settings {
    uint32_t freq;
    uint16_t deviation;
    uint16_t data_rate;
    uint16_t tcxo_vpull;
    uint16_t tx_gate_bias;
    uint16_t tx_vdd;
    uint16_t pa_ilimit;
    uint16_t tx_vdd_delay;
    uint16_t flags;
    uint8_t callsign[8];
};

struct lfr_board_info {
    uint32_t fw_git_hash;
    const char *sw_ver;
    uint16_t serial_no;
};


// Must call settings_load_xxx() before use!!
extern struct lfr_settings settings;
extern const struct lfr_board_info board_info;

int settings_load_default();
int settings_load_saved();
int settings_save();

uint64_t settings_get_silicon_id();



#endif /* SETTINGS_H */
