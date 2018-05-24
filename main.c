#include <msp430.h>
#include <stdbool.h>
#include <stdio.h>
#include "lib446x/si446x.h"
#include "mcu.h"
#include "msp430_uart.h"
#include "cmd_parser.h"

#define XTAL_FREQ   26000000L
#define OSC_TYPE    OPT_TCXO
#define FREQ_OFFSET 0

#define NSEL_PIN    0x63
#define SDN_PIN     0x13
#define INT_PIN     0x57

struct si446x_device dev;
uint8_t buf[255];

//status byte for command replies. Bits are [TXBUSY]:[BADCMD]:[BADCKSUM]:[GOODCMD]:[reserved for tx buffer depth 3:0]
uint8_t sys_stat = 0;

volatile bool do_pong = false;
volatile bool radio_irq = false;

uint16_t tx_gate_bias = 0xDF0; // ~29 dBm
//uint16_t tx_gate_bias = 0xA80; // ~0 dBm


void rx_cb(struct si446x_device *dev, int err, int len, uint8_t *data);
void tx_cb(struct si446x_device *dev, int err);
int set_gate_bias(uint16_t bias);

void set_cmd_flag(uint8_t flag){
    sys_stat &= ~(FLAG_GOODCMD | FLAG_INVALID | FLAG_BADSUM);
    sys_stat |= flag;
}

void reply_error(uint8_t code)
{
    uart_putc(SYNCWORD_H);
    uart_putc(SYNCWORD_L);
    uart_putc(sys_stat);
    uint16_t chksum = fletcher(0, sys_stat);
    uart_putc('E');
    chksum = fletcher(chksum, 'E');
    uart_putc(code);
    chksum = fletcher(chksum, code);
    uart_putc((char) (chksum >> 8));
    uart_putc((char) chksum);
}

void reply_nopay(uint8_t cmd)
{
    uart_putc(SYNCWORD_H);
    uart_putc(SYNCWORD_L);
    uart_putc(sys_stat);
    uint16_t chksum = fletcher(0, sys_stat);
    uart_putc(cmd);
    chksum = fletcher(chksum, cmd);
    uart_putc((char) (chksum >> 8));
    uart_putc((char) chksum);
}

void reply_pay(uint8_t cmd, int len, uint8_t *payload)
{
    uart_putc(SYNCWORD_H);
    uart_putc(SYNCWORD_L);
    uart_putc(sys_stat);
    uint16_t chksum = fletcher(0, sys_stat);
    uart_putc(cmd);
    chksum = fletcher(chksum, cmd);

    uart_putc(len);
    chksum = fletcher(chksum, len);

    for (; len >= 0; len--) {
        uart_putc(*payload);
        chksum = fletcher(chksum, *payload);
        payload++;
    }

    uart_putc((char) (chksum >> 8));
    uart_putc((char) chksum);
}

void tx_cb(struct si446x_device *dev, int err)
{
    if (err) {
        reply_error((uint8_t) -err);
    }

    set_gate_bias(0x000);
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
        reply_error((uint8_t) -err);
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
        err = set_gate_bias(tx_gate_bias);
        si446x_setup_tx(dev, sizeof(resp), resp, tx_cb);
        do_pong = true;

        return;
    } else {
        // Handle RX'd packet
        reply_pay(CMD_RXDATA, len, data);
    }
    sys_stat &= ~(FLAG_TXBUSY);
    si446x_recv_async(dev, 255, buf, rx_cb);
}

void command_handler(uint8_t cmd, uint8_t len, uint8_t* payload){
    if (cmd == CMD_TXDATA) {

        memcpy(buf, payload, len);

        sys_stat |= FLAG_TXBUSY;
        set_gate_bias(tx_gate_bias);
        int err = si446x_send_async(&dev, len, buf, tx_cb);

        if (err) {
            // Halt and catch fire
            reply_error((uint8_t) -err);
        } else {
            // We're good
            set_cmd_flag(FLAG_GOODCMD);
            reply_nopay(cmd);
        }

    } else if (cmd == CMD_SET_TXPWR) {
        if (len != 2) {
            reply_error(0xFF);
        } else {
            tx_gate_bias = (payload[0] << 8) | payload[1];
            set_cmd_flag(FLAG_GOODCMD);
            reply_nopay(cmd);
        }
    } else if (cmd == CMD_READ_TXPWR) {
        if (len != 0) {
            reply_error(0xFF);
        } else {
            uint8_t resp[] = {(uint8_t)(tx_gate_bias & 0xFF), (uint8_t)(tx_gate_bias >> 8)};
            set_cmd_flag(FLAG_GOODCMD);
            reply_pay(CMD_READ_TXPWR, sizeof(resp), resp);
        }
    } else if (cmd == CMD_RESET) {
        WDTCTL = 0x0000; // Force WDT reset
    } else if (cmd == CMD_NOP) {
        set_cmd_flag(FLAG_GOODCMD);
        reply_nopay(CMD_NOP);
    } else {
        set_cmd_flag(FLAG_INVALID);
        reply_error(0xFF);
    }
}

void cmd_err_handler(uint8_t cmd){
    set_cmd_flag(cmd & 0xf0); //transfer error flags from cmd byte to system status byte
    reply_error(cmd);
}

int set_gate_bias(uint16_t bias)
{
    uint8_t cmd[] = {
        0x40, // Write DAC register, normal mode
        bias >> 4,
        bias << 4
    };
    int len = i2c_write(0x61, cmd, sizeof(cmd));
    return len == sizeof(cmd) ? ESUCCESS : -EINVALSTATE;
}

int main(void)
{
    int err;
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer


    mcu_init();
    uart_init();

    //Power-on tests
    printf("hello world this is the right port\n");
    reply_nopay(CMD_RESET);

    gpio_config(0x10, OUTPUT);
    gpio_write(0x10, LOW);
    gpio_config(0x11, OUTPUT);
    gpio_write(0x11, LOW);

    set_cmd_handler(&command_handler);
    set_error_handler(&cmd_err_handler);

    i2c_init();
    err = set_gate_bias(0x000);

    if (err) {
        gpio_write(0x10, HIGH);
        mcu_reset();
    }

    si446x_create(&dev, NSEL_PIN, SDN_PIN, INT_PIN, XTAL_FREQ, OSC_TYPE);

    enable_pin_interrupt(0x81, RISING);
    enable_pin_interrupt(0x57, FALLING);

    err = si446x_init(&dev);
    if (err) {
        gpio_write(0x10, HIGH);
        mcu_reset();
    }

    err = si446x_set_frequency(&dev, 434000000L + FREQ_OFFSET);
    if (err) {
        gpio_write(0x10, HIGH);
        mcu_reset();
    }

    err = si446x_config_crc(&dev, CRC_SEED_1 | CRC_CCIT_16);
    if (err) {
        gpio_write(0x10, HIGH);
        mcu_reset();
    }

    err = si446x_set_tx_pwr(&dev, 0x14);
    if (err) {
        gpio_write(0x10, HIGH);
        mcu_reset();
    }

    err = si446x_cfg_gpio(&dev, 0x1A, 0x14, 0x11, 0x21);
    if (err) {
        gpio_write(0x10, HIGH);
        mcu_reset();
    }

    uint8_t rst[] = {'R', 'E', 'S', 'E', 'T'};

    set_gate_bias(tx_gate_bias);
    err = si446x_send(&dev, sizeof(rst), rst);
    set_gate_bias(0x000);

    if (err) {
        gpio_write(0x10, HIGH);
        mcu_reset();
    }

    sys_stat &= ~(FLAG_TXBUSY);
    err = si446x_recv_async(&dev, 255, buf, rx_cb);
    if (err) {
        gpio_write(0x10, HIGH);
        mcu_reset();
    }

    // Main loop
    while (true) {

        if (radio_irq) {
            radio_irq = false;
            err = si446x_update(&dev);
            if (err) {
                printf("Err: %d", err);
                si446x_reset(&dev);
                gpio_write(0x10, HIGH);
                reply_error(-err);
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

#pragma vector=PORT8_VECTOR
__interrupt void sync_word_isr()
{
    P8IFG = 0;
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
