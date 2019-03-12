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

#ifndef PINS_H_
#define PINS_H_

// Pin definitions for Rev. 2B
#define NSEL_PIN        0x37
#define SDN_PIN         0x45
#define INT_PIN         0x53
#define GPIO0           0x20
#define GPIO1           0x10
#define GPIO2           0x36
#define GPIO3           0x35
#define TX_ACT_PIN      0x72
#define PA_IMON_PIN     0x42
#define PA_IMON_ACHAN   10      //ADC channel 10
#define PA_VMON_PIN     0x14
#define PA_VMON_ACHAN   4       //ADC & comparator channel 4
#define PA_VSET_PIN     0x15
#define PA_VSET_ACHAN   5       //ADC & comparator channel 5
#define PA_PWR_EN_PIN   0x43
#define PA_PGOOD_PIN    0x74
#define DAC_nPWR_PIN    0x73
#define SDA_PIN         0x70
#define SCL_PIN         0x71

#define GATE_CHAN   0
#define TCXO_CHAN   1
#define VSET_CHAN   2
#define ISET_CHAN   3

#define XTAL_FREQ   26000000L
#define OSC_TYPE    OPT_TCXO

#endif /* PINS_H_ */
