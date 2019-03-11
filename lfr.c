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
#include <stdbool.h>
#include <stdio.h>
#include "pins.h"
#include "error.h"
#include "lib446x/si446x.h"
#include "mcu.h"
#include "radio.h"
#include "amp.h"
#include "msp430_uart.h"
#include "cmd_parser.h"
#include "settings.h"
#include "pkt_buf.h"

// Backing buffer for tx_queue
uint8_t __attribute__((persistent)) tx_backing_buf[16384] = {0};

uint8_t buf[255];

//status byte for command replies. Bits are [TXBUSY]:[BADCMD]:[BADCKSUM]:[GOODCMD]:[reserved for tx buffer depth 3:0]
uint8_t sys_stat = 0;

struct si446x_device dev;
struct pkt_buf tx_queue;

volatile bool do_pong = false;
volatile bool radio_irq = true;

void tx_cb(struct si446x_device *dev, int err);
void rx_cb(struct si446x_device *dev, int err, int len, uint8_t *data);
int reset_si446x();

void set_cmd_flag(uint8_t flag){
    sys_stat &= ~(FLAG_GOODCMD | FLAG_INVALID | FLAG_BADSUM);
    sys_stat |= flag;
}

int pre_transmit_no_delay()
{
    int err;
    gpio_write(PA_PWR_EN_PIN, HIGH);
    err = set_current_limit(settings.pa_ilimit);
    if(err) return err;
    err = set_drain_voltage(settings.tx_vdd);
    if(err) return err;
    err = set_gate_bias(settings.tx_gate_bias);
    if(err) return err;
    return 0;
}

int pre_transmit()
{
    int err;
    err = pre_transmit_no_delay();
    if(err) return err;
    delay_micros(settings.tx_vdd_delay);
    return 0;
}

int pa_power_keepalive(){
    //Pulse the PA_PWR_EN pin low to recharge the RC filter cap
    int err;
    //cut the drain current by killing the gate bias
    err = set_gate_bias(0);
    if(err) return err;
    gpio_write(PA_PWR_EN_PIN, LOW);
    //turn the gate bias back on
    //use the I2C transaction period as the delay to recharge the cap
    err = set_gate_bias(settings.tx_gate_bias);
    if(err) return err;
    gpio_write(PA_PWR_EN_PIN, HIGH);
    return 0;
}

int post_transmit()
{
    int err;
    gpio_write(PA_PWR_EN_PIN, LOW);
    err = set_gate_bias(0);
    if(err) return err;
    err = set_drain_voltage(0);
    if(err) return err;
    err = set_current_limit(0);
    if(err) return err;
    return 0;
}

int send_w_retry(int len, uint8_t *buf)
{
    int attempts;
    int err;

    pre_transmit();

    for (attempts = 0; attempts < 3; attempts++) {

        err = si446x_send_async(&dev, len, buf, tx_cb);

        if (err == -ERESETSI) {

            err = reset_si446x();
            if (err) {
                return err;
            }

            // Retry

        } else if (err) {
            return err;
        } else {
            return ESUCCESS;
        }
    }

    // Give up
    return -ETIMEOUT;
}

void tx_cb(struct si446x_device *dev, int err)
{
    if (err) {
        reply_error(sys_stat, (uint8_t) -err);
    }

    wdt_feed();

    if (pkt_buf_depth(&tx_queue) > 0) {
        int pkt_len = MAX_PAYLOAD_LEN;
        err = pkt_buf_dequeue(&tx_queue, &pkt_len, buf);

        if (err) {
            reply_error(sys_stat, (uint8_t) -err);
            post_transmit();
            sys_stat &= ~(FLAG_TXBUSY);
            si446x_recv_async(dev, 255, buf, rx_cb);
            return;
        }

        pa_power_keepalive();
        err = send_w_retry(pkt_len, buf);

        if (err) {
            reply_error(sys_stat, (uint8_t) -err);
            post_transmit();
            sys_stat &= ~(FLAG_TXBUSY);
            si446x_recv_async(dev, 255, buf, rx_cb);
            return;
        }
    } else {
        post_transmit();
        sys_stat &= ~(FLAG_TXBUSY);
        si446x_recv_async(dev, 255, buf, rx_cb);
    }
}

void rx_cb(struct si446x_device *dev, int err, int len, uint8_t *data)
{

    // Stop rx timeout timer
    TA1CCTL0 = 0;                           // TACCR0 interrupt disabled
    TA1CTL |= TACLR;                        // Stop timer

    if (err) {
        sys_stat &= ~(FLAG_TXBUSY);
        reply_error(sys_stat, (uint8_t) -err);
        si446x_recv_async(dev, 255, buf, rx_cb);
        return;
    }

    if (len == 4 && data[0] == 'P' && data[1] == 'I' && data[2] == 'N'
        && data[3] == 'G') {

        uint8_t resp[] = {
            'P',
            'O',
            'N',
            'G',
        };
        sys_stat |= FLAG_TXBUSY;
        gpio_write(TX_ACT_PIN, HIGH);
        err = set_gate_bias(settings.tx_gate_bias);
        si446x_setup_tx(dev, sizeof(resp), resp, tx_cb);
        do_pong = true;

        return;
    } else {
        // Handle RX'd packet
        reply(sys_stat, CMD_RXDATA, len, data);
    }
    sys_stat &= ~(FLAG_TXBUSY);
    si446x_recv_async(dev, 255, buf, rx_cb);
}

void error(int err, char *file, int line) {
    printf("Error: %s (%d) at %s:%d\n", "" /* strerror[err] */ , err, file, line);
    gpio_config(0x40, OUTPUT);
    gpio_write(0x40, HIGH);
    __disable_interrupt();
    while (1) LPM0;
}

int reload_config()
{
    int err;

    err = set_frequency(settings.freq);
    if (err) {
        return err;
    }

    err = set_dac_output(TCXO_CHAN, settings.tcxo_vpull);
    if (err) {
        return err;
    }

    err = si446x_check_crc(&dev, settings.flags & FLAG_CRC_CHECK);
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

    return 0;
}

int config_si446x()
{
    int err;

    err = reload_config();

    if (err) {
        settings_load_default();
        // TODO: Report this error
    }

    err = si446x_config_crc(&dev, CRC_SEED_1 | CRC_CCIT_16);
    if (err) {
        return err;
    }

    err = si446x_set_tx_pwr(&dev, 0x14);
    if (err) {
        return err;
    }

    err = si446x_cfg_gpio(&dev, GPIO_SYNC_WORD_DETECT, GPIO_TX_DATA, GPIO_TX_DATA_CLK, GPIO_TX_STATE);
    if (err) {
        return err;
    }

    return ESUCCESS;
}

int reset_si446x()
{

    int err;

    // Disable pin interrupts while we reset
    disable_pin_interrupt(GPIO0);
    disable_pin_interrupt(INT_PIN);

    err = si446x_reinit(&dev);

    if (err) {
        return err;
    }

    err = config_si446x();
    if (err) {
        return err;
    }

    enable_pin_interrupt(GPIO0, RISING);
    enable_pin_interrupt(INT_PIN, FALLING);

    return ESUCCESS;

}

int main(void)
{
    int err;
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    settings_load_saved();
    pkt_buf_init(&tx_queue, sizeof(tx_backing_buf), tx_backing_buf);
    mcu_init();
    uart_init();

    gpio_config(TX_ACT_PIN, OUTPUT);
    gpio_write(TX_ACT_PIN, LOW);
    gpio_config(PA_PWR_EN_PIN, OUTPUT);
    gpio_write(PA_PWR_EN_PIN, LOW);

    //Power-on tests
    printf("LFR Starting up...\n");
    printf("LFR Build: %s\n", board_info.sw_ver);
    reply(sys_stat, CMD_RESET, 0, NULL);

    i2c_init();
    err = set_gate_bias(0x000);

    if (err) {
        error(-err, __FILE__, __LINE__);
        return err;
    }

    si446x_create(&dev, NSEL_PIN, SDN_PIN, INT_PIN, XTAL_FREQ, OSC_TYPE);

    err = si446x_init(&dev);
    if (err) {
        error(-err, __FILE__, __LINE__);
        return err;
    }

    config_si446x();

    printf("Successfully initialized Si446x!\n");

    enable_pin_interrupt(GPIO0, RISING);
    enable_pin_interrupt(INT_PIN, FALLING);

    err = si446x_recv_async(&dev, 255, buf, rx_cb);

    if (err) {
        error(-err, __FILE__, __LINE__);
        return err;
    }

    // Main loop
    while (true) {

        if (radio_irq) {
            radio_irq = false;
            err = si446x_update(&dev);
            if (err) {
                printf("Err: %d", err);
                // TODO: Better error handling
                si446x_reinit(&dev);
                config_si446x();
                si446x_recv_async(&dev, 255, buf, rx_cb);
                reply_error(sys_stat, -err);
            }
        } else if(uart_available()) {
                char c = uart_getc();
                parse_char(c);
        } else {
                //LPM0; //enter LPM0 until an interrupt happens on the uart
        }
    }
}

#pragma vector=PORT5_VECTOR
__interrupt void irq_isr()
{
    P5IFG = 0;
    radio_irq = true;
}

#pragma vector=PORT2_VECTOR
__interrupt void sync_word_isr()
{
    P2IFG &= ~(1 << (GPIO0 & 0x0F));
    TA0CTL |= TACLR;                        // Clear count
    TA0CTL |= MC__UP;                       // Start timer in UP mode
    TA0CCTL0 = CCIE;                        // TACCR0 interrupt enabled

    // Start RX timeout timer
    TA1CTL |= TACLR;                        // Clear count
    TA1CTL |= MC__UP;                       // Start timer in UP mode
    TA1CCTL0 = CCIE;                        // TACCR0 interrupt enabled
}

#pragma vector=TIMER0_A0_VECTOR
__interrupt void ping_timer_isr()
{
    TA0CCTL0 = 0;                           // TACCR0 interrupt disabled
    TA0CTL |= TACLR;                        // Stop timer

    if (do_pong) {
        do_pong = false;
        if (si446x_fire_tx(&dev) == -ERESETSI) {
            reset_si446x();
            si446x_recv_async(&dev, 255, buf, rx_cb);
        }
    }
}

#pragma vector=TIMER1_A0_VECTOR
__interrupt void rx_timeout_isr()
{
    TA1CCTL0 = 0;                           // TACCR0 interrupt disabled
    TA1CTL |= TACLR;                        // Stop timer

    si446x_rx_timeout(&dev);
    radio_irq = true;
}
