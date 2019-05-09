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

#ifndef STATUS_H
#define STATUS_H

#include <stdbool.h>
#include <stdint.h>

// TODO: Mostly unimplemented

#define STATUS_RESET            0
//#define STATUS_CMDREPLY         1
//#define STATUS_CMDERR           2

//#define STATUS_INTERNALERR      3
//#define STATUS_RFICBUG          4
//#define STATUS_CRCMISS          5
//#define STATUS_PATIMEOUT        6
//#define STATUS_PAOVERCUR        7

#define STATUS_TXBUSY           8
//#define STATUS_TXFULL           9
//#define STATUS_TXEMPTY          10
//#define STATUS_RXBUSY           11
//#define STATUS_RXFULL           12
//#define STATUS_RXNOTEMPTY       13


void set_status(uint8_t key, bool val);
bool get_status(uint8_t key);
uint32_t get_all_status();

#endif
