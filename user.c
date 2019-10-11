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

#include "cmd_parser.h"
#include "cmd_handler.h"
#include "user.h"

#include "error.h"
#include "lfr.h"

const char *hello = "Hello, world!";

uint32_t ticks = 0;

void cmd_user(uint8_t cmd, uint8_t len, uint8_t *data)
{
    switch(cmd) {
    case CMD_USER0:
        reply(CMD_USER0, 13, hello);
        break;
    case CMD_USER1:
        uint8_t arr[] = {
            (ticks >> 24) & 0xFF, (ticks >> 16) & 0xFF, 
            (ticks >> 8) & 0xFF, ticks & 0xFF
        };
        reply(CMD_USER1, 4, &ticks);
        break;
    default:
        cmd_err(ECMDINVAL);
    }
}

void user_tick()
{
    ticks++;
}