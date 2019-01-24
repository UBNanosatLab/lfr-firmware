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

#include <stdint.h>
#include <msp430.h>

#include "mcu.h"
#include "error.h"
#include "pins.h"

int set_gate_bias(uint16_t bias_mv)
{
    // Already 1 mV / LSB
    return set_dac_output(GATE_CHAN, bias_mv);
}

int set_drain_voltage(uint16_t vdd_mv)
{
    // 97 counts = 1V
    // So 99 counts = 1024 mv
    return set_dac_output(VSET_CHAN, (uint16_t)((vdd_mv * (uint32_t)99) >> 10));
}

int set_current_limit(uint16_t ilim_ma)
{
    // 2mA/LSB, up to 1000 (2A)
    return set_dac_output(ISET_CHAN, ilim_ma >> 1);
}
