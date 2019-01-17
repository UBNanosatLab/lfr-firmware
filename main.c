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
#include "lib446x/si446x.h"
#include "mcu.h"
#include "msp430_uart.h"
#include "cmd_parser.h"
#include "settings.h"

#define XTAL_FREQ   26000000L
#define OSC_TYPE    OPT_TCXO

// Pin definitions for Rev. 1A
#define NSEL_PIN        0x37
#define SDN_PIN         0x24
#define INT_PIN         0x53
#define GPIO0           0x20
#define GPIO1           0x44
#define GPIO2           0x36
#define GPIO3           0x35
#define TX_ACT_PIN      0x15
#define PA_IMON_PIN     0x42
#define PA_PWR_EN_PIN   0x43
#define PA_PGOOD_PIN    0x74

#define GATE_CHAN   0
#define TCXO_CHAN   1
#define VSET_CHAN   2
#define ISET_CHAN   3

struct si446x_device dev;
uint8_t buf[255];

//status byte for command replies. Bits are [TXBUSY]:[BADCMD]:[BADCKSUM]:[GOODCMD]:[reserved for tx buffer depth 3:0]
uint8_t sys_stat = 0;

volatile bool do_pong = false;
volatile bool radio_irq = true;

void rx_cb(struct si446x_device *dev, int err, int len, uint8_t *data);
void tx_cb(struct si446x_device *dev, int err);

int set_gate_bias(uint16_t bias_mv);
int set_drain_voltage(uint16_t vdd_mv);
int set_current_limit(uint16_t ilim_ma);

int pre_transmit();
int post_transmit();

void set_cmd_flag(uint8_t flag){
    sys_stat &= ~(FLAG_GOODCMD | FLAG_INVALID | FLAG_BADSUM);
    sys_stat |= flag;
}

void tx_cb(struct si446x_device *dev, int err)
{
    if (err) {
        reply_error(sys_stat, (uint8_t) -err);
    }

    post_transmit();
    sys_stat &= ~(FLAG_TXBUSY);
    si446x_recv_async(dev, 255, buf, rx_cb);
    wdt_feed();
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

int set_dac_output(uint8_t chan, uint16_t val) {
    if (chan > 3 || val > 0xFFF) {
        return -EINVAL;
    }


    // Multi-write (doesn't update EEPROM)
    // -----------------------------------------------
    // Byte 1: 0    1    0    0    0    DAC1 DAC0 UDAC
    // Byte 2: VREF PD1  PD0  Gx   D11  D10  D9   D8
    // Byte 3: D7   D6   D5   D4   D3   D2   D1   D0

    uint8_t cmd[] = {
        0x40 | (chan << 1) | 0x00,      // Multiwrite | channel | ~UDAC (update immediately)
        0x80 | 0x00 | 0x10 | val >> 8,  // VREF = 2.048V internal | PD = Normal operation | Gain = x2 | val[11:8]
        val & 0xFF                      // val[7:0]
    };

    int len = i2c_write(0x60, cmd, sizeof(cmd));
    return len == sizeof(cmd) ? ESUCCESS : -EINVALSTATE;
}

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

    err = si446x_set_frequency(&dev, settings.freq);
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

    return 0;
}

int main(void)
{
    int err;
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    settings_load_saved();
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

    reload_config();

    err = si446x_config_crc(&dev, CRC_SEED_1 | CRC_CCIT_16);
    if (err) {
        error(-err, __FILE__, __LINE__);
        return err;
    }

    err = si446x_set_tx_pwr(&dev, 0x14);
    if (err) {
        error(-err, __FILE__, __LINE__);
        return err;
    }

    err = si446x_cfg_gpio(&dev, GPIO_SYNC_WORD_DETECT, GPIO_TX_DATA, GPIO_TX_DATA_CLK, GPIO_RX_STATE);
    if (err) {
        error(-err, __FILE__, __LINE__);
        return err;
    }

    printf("Successfully initialized Si4464!\n");

    enable_pin_interrupt(GPIO0, RISING);
    enable_pin_interrupt(INT_PIN, FALLING);

//    uint8_t rst[] = {'R', 'E', 'S', 'E', 'T'};
//
//    gpio_write(TX_ACT_PIN, HIGH);
//    set_gate_bias(tx_gate_bias);
//    err = si446x_send(&dev, sizeof(rst), rst);
//    set_gate_bias(0x000);
//    gpio_write(TX_ACT_PIN, LOW);
//
//    if (err) {
//        error(-err, __FILE__, __LINE__);
//        return err;
//    }
//    sys_stat &= ~(FLAG_TXBUSY);


    err = si446x_recv_async(&dev, 255, buf, rx_cb);


    // Main loop
    while (true) {

        if (radio_irq) {
            radio_irq = false;
            err = si446x_update(&dev);
            if (err) {
                printf("Err: %d", err);
                si446x_reset(&dev);
                reply_error(sys_stat, -err);
            }
        } else if(uart_available()) {
                char c = uart_getc();
                parse_char(c);
        } else {
//                LPM0; //enter LPM0 until an interrupt happens on the uart
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
        si446x_fire_tx(&dev);
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
