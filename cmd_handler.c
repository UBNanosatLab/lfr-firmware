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

#include "error.h"
#include "lfr.h"
#include "radio.h"
#include "mcu.h"
#include "lib446x/si446x.h"
#include "cmd_handler.h"
#include "cmd_parser.h"
#include "msp430_uart.h"
#include "settings.h"
#include "status.h"
#include "adc.h"
#include "pins.h"

void cmd_nop() {
    reply(CMD_NOP, 0, NULL);
}

void cmd_reset() {
    WDTCTL = 0x0000; // Force WDT reset
}

void cmd_set_txpwr(uint16_t pwr) {
    settings.tx_gate_bias = pwr;
    reply(CMD_SET_TXPWR, 0, NULL);
}

void cmd_get_txpwr() {
    uint8_t resp[] = {(uint8_t)(settings.tx_gate_bias >> 8), (uint8_t)(settings.tx_gate_bias & 0xFF)};
    reply(CMD_READ_TXPWR, sizeof(resp), resp);
}

void cmd_tx_data(int len, uint8_t *data) {
    int err;

    err = pkt_buf_enqueue(&tx_queue, len, data);

    if (err) {
        reply_cmd_error((uint8_t) -err);
    } else {
        if (!get_status(STATUS_TXBUSY)) {
            set_status(STATUS_TXBUSY, true);
            pre_transmit();

            uint16_t pkt_len = MAX_PAYLOAD_LEN;
            err = pkt_buf_dequeue(&tx_queue, (int*)&pkt_len, buf);

            if (err) {
                reply_cmd_error((uint8_t) -err);
                return;
            }

            err = send_w_retry(pkt_len, buf);

            if (err) {
                reply_cmd_error((uint8_t) -err);
            } else {
                reply(CMD_TXDATA, 0, NULL);
            }
        } else {
            reply(CMD_TXDATA, 0, NULL);
        }
    }
}

void cmd_get_queue_depth() {
    uint16_t depth = pkt_buf_depth(&tx_queue);
    uint8_t data[] = {depth >> 8, depth & 0xFF};
    reply(CMD_GET_QUEUE_DEPTH, sizeof(data), data);
}

void cmd_set_freq(uint32_t freq) {
    settings.freq = freq;
    int err = set_frequency(freq);

    if (err) {
        reply_cmd_error((uint8_t) -err);
    } else {
        reply(CMD_SET_FREQ, 0, NULL);
    }
}

void cmd_abort_tx()
{

    int err;

    err = post_transmit();

    if (err) {
        reply_cmd_error((uint8_t) -err);
        return;
    }

    err = si446x_abort_tx(&dev);
    if (err) {
        reply_cmd_error((uint8_t) -err);
        return;
    }

    err = set_modem_config(settings.modem_config);

    if (err) {
        reply_cmd_error((uint8_t) -err);
        return;
    }

    if ((settings.flags & FLAG_MOD_MASK) == FLAG_MOD_CW) {
        err = si446x_set_mod_type(&dev, MOD_TYPE_CW);
    } else if ((settings.flags & FLAG_MOD_MASK) == FLAG_MOD_FSK) {
        err = si446x_set_mod_type(&dev, MOD_TYPE_2FSK);
    } else if ((settings.flags & FLAG_MOD_MASK) == FLAG_MOD_GFSK) {
        err = si446x_set_mod_type(&dev, MOD_TYPE_2GFSK);
    } else {
        err = si446x_set_mod_type(&dev, MOD_TYPE_2GFSK);
    }

    if (err) {
        reply_cmd_error((uint8_t) -err);
        return;
    }

    err = si446x_recv_async(&dev, 255, buf, rx_cb);

    if (err) {
        reply_cmd_error((uint8_t) -err);
    } else {
        reply(CMD_TX_ABORT, 0, NULL);
    }
}

void cmd_tx_psr()
{
    int err;
    int attempts;

    if ((settings.flags & FLAG_MOD_MASK) == FLAG_MOD_CW) {
        err = si446x_set_mod_type(&dev, MOD_SRC_RAND | MOD_TYPE_CW);
    } else if ((settings.flags & FLAG_MOD_MASK) == FLAG_MOD_FSK) {
        err = si446x_set_mod_type(&dev, MOD_SRC_RAND | MOD_TYPE_2FSK);
    } else if ((settings.flags & FLAG_MOD_MASK) == FLAG_MOD_GFSK) {
        err = si446x_set_mod_type(&dev, MOD_SRC_RAND | MOD_TYPE_2GFSK);
    } else {
        err = si446x_set_mod_type(&dev, MOD_SRC_RAND | MOD_TYPE_2GFSK);
    }

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
            reply(CMD_TX_PSR, 0, NULL);
            return;
        }
    }

    // Give up
    reply_cmd_error((uint8_t) ETIMEOUT);
}

void cmd_set_cfg(int len, uint8_t *data)
{
    unsigned int i = 0;

    // Right length?
    if (len != 26) {
        cmd_err(ECMDINVAL);
        return;
    }

    // Check cfg struct version
    if (data[i++] != SETTINGS_VER) {
        cmd_err(EINVAL);
        return;
    }

    // Okay, we've got the correct length and version!
    // Set the settings!

    settings.freq = ((uint32_t)data[i] << 24) | ((uint32_t)data[i+1] << 16) | ((uint32_t)data[i+2] << 8) | data[i+3];
    i += 4;

    settings.modem_config = data[i];
    i += 1;

    settings.tcxo_vpull = (data[i] << 8) | data[i+1];
    i += 2;

    settings.tx_gate_bias = (data[i] << 8) | data[i+1];
    i += 2;

    settings.tx_vdd = (data[i] << 8) | data[i+1];
    i += 2;

    settings.pa_ilimit = (data[i] << 8) | data[i+1];
    i += 2;

    settings.tx_vdd_delay = (data[i] << 8) | data[i+1];
    i += 2;

    settings.flags = (data[i] << 8) | data[i+1];
    i += 2;

    unsigned int j;
    for (j = 0; j < 8; j++) {
        settings.callsign[j] = data[i++];
    }

    int err = reload_config();

    if (err) {
        reply_cmd_error((uint8_t) -err);
    } else {
        reply(CMD_SET_CFG, 0, NULL);
    }

}

void cmd_get_cfg()
{
    uint8_t data[] = {
                    SETTINGS_VER,

                    settings.freq >> 24,
                    (settings.freq >> 16) & 0xFF,
                    (settings.freq >> 8) & 0xFF,
                    settings.freq & 0xFF,

                    settings.modem_config,

                    settings.tcxo_vpull >> 8,
                    settings.tcxo_vpull & 0xFF,

                    settings.tx_gate_bias >> 8,
                    settings.tx_gate_bias & 0xFF,

                    settings.tx_vdd >> 8,
                    settings.tx_vdd & 0xFF,

                    settings.pa_ilimit >> 8,
                    settings.pa_ilimit & 0xFF,

                    settings.tx_vdd_delay >> 8,
                    settings.tx_vdd_delay & 0xFF,

                    settings.flags >> 8,
                    settings.flags & 0xFF,

                    settings.callsign[0],
                    settings.callsign[1],
                    settings.callsign[2],
                    settings.callsign[3],
                    settings.callsign[4],
                    settings.callsign[5],
                    settings.callsign[6],
                    settings.callsign[7]
    };

    reply(CMD_GET_CFG, sizeof(data), data);
}

void cmd_save_cfg()
{
    int err;
    err = settings_save();
    if (err) {
        reply_cmd_error((uint8_t) -err);
    } else {
        reply(CMD_SAVE_CFG, 0, NULL);
    }
}

void cmd_cfg_default()
{
    int err;
    err = settings_load_default();
    if (err) {
        reply_cmd_error((uint8_t) -err);
    } else {
        reload_config();
        reply(CMD_CFG_DEFAULT, 0, NULL);
    }
}

//Calibration values for internal temp sensor from TLV
#define ADC_2V5_30C_CAL (*(int*) 0x01A22)
#define ADC_2V5_85C_CAL (*(int*) 0x01A24)

void cmd_get_temps(){
    int32_t mcu_temp_scale = (8500L-3000L)/(ADC_2V5_85C_CAL - ADC_2V5_30C_CAL);
    int16_t mcu_temp = 0;
    int16_t pa_temp = 0;
    mcu_temp = adc_read(ADC_CHAN_TEMP);
    pa_temp = adc_read(PA_TEMP_ACHAN);
    mcu_temp = 3000+(mcu_temp - ADC_2V5_30C_CAL) * mcu_temp_scale;
    //TMP235, 10mV/C, 500mV at 0C, so 4095 means 2.5V means 200C, 0 means 0V means -50C
    pa_temp =((pa_temp * 25000L)>>12) - 5000;
    uint8_t data[4] = {
                       pa_temp >> 8,
                       pa_temp & 0xFF,

                       mcu_temp >> 8,
                       mcu_temp & 0xFF
    };
    reply(CMD_GET_TEMPS, 4, (uint8_t*) data);
}

void cmd_err(int err) {
    reply_cmd_error((uint8_t) err);
}

int host_reply_putc(uint8_t c) {
    uart_putc(c);
    return 0;
}
