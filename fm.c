/*
 * fm.c
 *
 *  Created on: Jan 19, 2020
 *      Author: iracigt
 */

#include <msp430.h>
#include <stdbool.h>
#include <stdint.h>
#include "mcu.h"
#include "pins.h"
#include "settings.h"
#include "lib446x/si446x.h"
#include "lfr.h"
#include "radio.h"
#include "fm.h"

#include "audio.h"

static long snd_index = 0;
static long snd_duration = 0;

static uint8_t snd_msg[255];
static uint8_t snd_msg_len = 0;
static int snd_msg_index = 0;

void fm_step()
{
    while (snd_duration <= 0) {
        // step to next sound bite

        // Careful with 8bit rollover here
        if (snd_msg_index >= snd_msg_len) {
            // Done with msg
            si446x_set_freq_offset(&dev, 0);
            fm_stop();

            return;
        }

        if (snd_msg[snd_msg_index] < sizeof(snd_offsets) / sizeof(*snd_offsets)) {
            snd_index = snd_offsets[snd_msg[snd_msg_index]];
            snd_duration = snd_durations[snd_msg[snd_msg_index]];
        } else {
            // Skip unknown sounds
        }

        snd_msg_index++;
    }

    int8_t samp = snd[snd_index] - 0x78;

    //    uint8_t outdiv = 8;
    //    int16_t offset_freq =  ((int16_t)samp - 0x78) * 20;
    //    int16_t offset_counts =  outdiv * ((int32_t) offset_freq << 18) / XTAL_FREQ;
    // Each count is about 12.4 Hz at 435 MHz
    // So 127 * 2 * 12.4 Hz = 3150 Hz
    int16_t offset_counts = 2 * samp; // about 3 kHz deviation

    si446x_set_freq_offset(&dev, offset_counts);

    snd_duration--;

    if (++snd_index >= sizeof(snd) / sizeof(*snd))
    {
        snd_index = 0;
    }
}

int fm_set_msg(uint8_t len, uint8_t *data)
{
    // len <= 255
    memcpy(snd_msg, data, len);
    snd_msg_len = len;
    snd_msg_index = 0;

    return 0;
}

int fm_init()
{
    TA3CCTL0 = 0;                           // TACCR0 interrupt disabled
    TA3CTL = TACLR;
    TA3CCR0 = 1600L;                        // 5khz @ 8e6 Hz clk
    TA3CTL = TASSEL__SMCLK | TACLR | MC__UP;         // SMCLK
    return 0;
}

static int tx_dead_key()
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
        return err;
    }

    err = si446x_set_mod_type(&dev, MOD_SRC_RAND | MOD_TYPE_CW);

    if (err) {
        return err;
    }

    err = pre_transmit();
    if (err) {
        return err;
    }

    for (attempts = 0; attempts < 3; attempts++) {
        err = si446x_fire_tx(&dev);

        if (err == -ERESETSI) {

            err = reset_si446x();
            if (err) {
                return err;
            }

            // Retry

        } else if (err) {
            return err;
        } else {
            return 0;
        }
    }

    // Give up
    return -ETIMEOUT;
}

static int tx_abort()
{
    int err;

    err = post_transmit();
    if (err) {
        return err;
    }

    err = si446x_idle(&dev);
    if (err) {
        return err;
    }

    err = set_modem_config(settings.modem_config);
    if (err) {
        return err;
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
        return err;
    }

    err = si446x_recv_async(&dev, 255, buf, rx_cb);
    if (err) {
        return err;
    }

    return 0;
}


int fm_start()
{
    int err = tx_dead_key();
    if (err) {
        return err;
    }

    TA3CCTL0 = CCIE;                        // TACCR0 interrupt enabled
    return 0;
}

int fm_stop()
{
    TA3CCTL0 = 0;                           // TACCR0 interrupt disabled
    return tx_abort();
}
