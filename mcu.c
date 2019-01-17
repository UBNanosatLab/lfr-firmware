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
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "mcu.h"


#define CYC_PER_US      8
#define FOR_LOOP_CYC    4 // guessing here...
#define F_CPU            8000000L
//#define F_SMCLK         (4000000ul)
#define F_SMCLK         (8000000ul)

#define TLV_SILICON_UID_BASE (0x01A0A)

#define SPI_CLKDIV      (F_SMCLK / 1000000L) //0.5 MHz SPI
#define FREQ_I2C (400000ul)

volatile unsigned char _i2c_tx_byte_ctr; //bytes remaining this transaction
unsigned char _i2c_tx_bytes; //bytes to send this transaction
unsigned char *_i2c_tx_data; //pointer to buffer of data to send
volatile bool _i2c_did_nack = false;

static volatile unsigned char *const gpio_dir_for_port[] =
{
    0x0000,
    &P1DIR,
    &P2DIR,
    &P3DIR,
    &P4DIR,
    &P5DIR,
    &P6DIR,
    &P7DIR,
    &P8DIR,
};

static volatile unsigned char *const gpio_out_for_port[] =
{
    0x0000,
    &P1OUT,
    &P2OUT,
    &P3OUT,
    &P4OUT,
    &P5OUT,
    &P6OUT,
    &P7OUT,
    &P8OUT,
};

static volatile unsigned char *const gpio_in_for_port[] =
{
    0x0000,
    &P1IN,
    &P2IN,
    &P3IN,
    &P4IN,
    &P5IN,
    &P6IN,
    &P7IN,
    &P8IN,
};

static volatile unsigned char *const gpio_ren_for_port[] =
{
    0x0000,
    &P1REN,
    &P2REN,
    &P3REN,
    &P4REN,
    &P5REN,
    &P6REN,
    &P7REN,
    &P8REN,
};
static volatile unsigned char *const gpio_ie_for_port[] =
{
    0x0000,
    &P1IE,
    &P2IE,
    &P3IE,
    &P4IE,
    &P5IE,
    &P6IE,
    &P7IE,
    &P8IE,
};

static volatile unsigned char *const gpio_ifg_for_port[] =
{
    0x0000,
    &P1IFG,
    &P2IFG,
    &P3IFG,
    &P4IFG,
    &P5IFG,
    &P6IFG,
    &P7IFG,
    &P8IFG,
};

static volatile unsigned char *const gpio_ies_for_port[] =
{
    0x0000,
    &P1IES,
    &P2IES,
    &P3IES,
    &P4IES,
    &P5IES,
    &P6IES,
    &P7IES,
    &P8IES,
};

/**
 * @brief Initialize the backchannel/debug UART
 *
 * Configures pins, sets baud rate, and enables UCA1 as the
 * backchannel/debug UART for printf-style output at 9600 baud
 */

void bc_uart_init()
{
    P2SEL1 |= BIT5 | BIT6;                // Configure debug UART pins
    P2SEL0 &= ~(BIT5| BIT6);


    // Config UCA1 (debug UART)
    // For 8 MHz SMCLK
    UCA1CTLW0 = UCSWRST;                    // Put eUSCI in reset
    UCA1CTLW0 |= UCSSEL__SMCLK;             // CLK = SMCLK
    // Baud Rate calculation
    // Table 30-5
    UCA1BRW = 52;
    //                             UCBRSx
    UCA1MCTLW = UCOS16 | UCBRF_1 | 0x4900;
    UCA1CTLW0 &= ~UCSWRST;                  // Initialize eUSCI

}

int fputc(int _c, register FILE *_fp)
{
  while(!(UCA1IFG & UCTXIFG));
  UCA1TXBUF = (unsigned char) _c;

  return((unsigned char)_c);
}

int fputs(const char *_ptr, register FILE *_fp)
{
  unsigned int i, len;

  len = strlen(_ptr);

  for(i=0 ; i<len ; i++)
  {
    while(!(UCA0IFG & UCTXIFG));
    UCA1TXBUF = (unsigned char) _ptr[i];
  }

  return len;
}

/**
 * @brief Initialize system clocks
 *
 * Sets DCO to 16 MHz to be used as a fallback,
 * sets main and subsystem clock dividers to 2, to run at 8 MHz,
 * configures the HFXT for operation with the 16 MHz crystal,
 * waits for it to stabilize, and switches the main clock to the HFXT.
 *
 * Routes VLO to ACLK.
 */
void init_clock()
{
    // PJSEL0 |= BIT4 | BIT5;                                  // Set up external 32kHz xtal pins

    CSCTL0 = CSKEY;                                         // Enable Access to CS Registers

    // For 16 MHz external xtal

    PJSEL0 |= BIT6 | BIT7;                                  // Set up external high freq xtal pins

    CSCTL0_H = CSKEY_H;                                     // Unlock CS registers
    CSCTL1 = DCOFSEL_4 | DCORSEL;                           // Set DCO to 16MHz
    CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;
    CSCTL3 = DIVA__1 | DIVS__2 | DIVM__2;                   // MCLK, SMCLK 8MHz (HFXT / 2)
    CSCTL4 = HFFREQ_1 | HFXTDRIVE_3 | HFXTOFF | LFXTOFF;    // Set drive strength and (8, 16] MHz xtal
    CSCTL4 &= ~(HFXTOFF);                                   // Start HFXT
    CSCTL5 &= ~(HFXTOFFG | LFXTOFFG);                       // Clear fault flags

    do                                                      // Wait for HFXT to stabilize
    {
        CSCTL5 &= ~(HFXTOFFG);                              // Clear HFXT fault flag
        SFRIFG1 &= ~OFIFG;
    } while (SFRIFG1 & OFIFG);                              // Test oscillator fault flag


    CSCTL2 = SELA__VLOCLK | SELS__HFXTCLK | SELM__HFXTCLK;  // jump to HFXT

    CSCTL0_H = 0;                                           // Lock CS registers
}

void mcu_init()
{
    PM5CTL0 &= ~LOCKLPM5;                                   // Disable the GPIO power-on default high-impedance mode

    init_clock();
    bc_uart_init();

    __disable_interrupt();
//    P8IES &=  ~BIT1;
//    P8IE |= BIT1;
//    P8IFG |= BIT1;

    TA0CCTL0 = 0;                           // TACCR0 interrupt disabled
    TA1CCTL0 = 0;                           // TACCR0 interrupt disabled
    TA0CCR0 = 48000L;                       // 6 ms @ 8e6 Hz clk
    TA0CTL = TASSEL__SMCLK | TACLR;         // SMCLK
    TA1CTL = TACLR;
    TA1CCR0 = 6882L;                        // 210 ms @ 2^15 (32K) Hz clk
    TA1CTL = TASSEL__ACLK | TACLR;          // ACLK

    // 16 sec w/ 32.768 kHz ACLK
    // TODO: Re-enable
//     WDTCTL = WDTPW | WDTSSEL__ACLK | WDTCNTCL | WDTIS__512K;

    __enable_interrupt();
}

void wdt_feed(){
    WDTCTL = WDTPW | (WDTCTL & 0x00FF) | WDTCNTCL;
}

void mcu_reset(){
    WDTCTL = 0;
}

/** @brief Initialize internal I2C bus
 *
 * Configures UCB2 for I2C operation on pins 7.0 and 7.1
 * Initialization of the peripheral is preceded by a series of pulses on SCL
 * to free the bus in case it was left in mid-transaction by a reset
 */
void i2c_init(){

    P7DIR |= BIT1; //set SCL to output

    P7OUT |= BIT1 | BIT0; //make sure the resistors will pull UP
    P7REN |= BIT1 | BIT0; //enable internal pull-ups on I2C pins

    int z;
    for(z=0; z<32; z++){
        P7DIR ^= BIT1; //toggle SCL a bunch to shake the bus loose
        __delay_cycles(40); //at 8MHz, delay ~5us for 100kHz clock
    }

    // Configure P7.0 and P7.1
    P7SEL0 |= BIT0 | BIT1;
    P7SEL1 &= ~(BIT0 | BIT1);

    // Configure USCI_B2 for I2C mode
    UCB2CTLW0 = UCSWRST;                    // put eUSCI_B in reset state
    UCB2CTLW0 |= UCMODE_3 | UCMST | UCSSEL__SMCLK; // I2C master mode, SMCLK
    UCB2BRW = F_SMCLK/FREQ_I2C;           //divide clock for 400kHz
    UCB2CTLW0 &= ~UCSWRST;                  // clear reset register
    UCB2IE |= UCTXIE0 | UCNACKIE;           // transmit and NACK interrupt enable
}

/**
 * @brief Blocking write to an I2C slave
 *
 * Writes a buffer to an I2C slave device, returning when the transfer has
 * completed or a NACK has been received.
 * @param slave_addr The 7-bit slave address
 * @param buf Pointer to the data to be written
 * @param len Number of bytes to write
 * @return Number of bytes successfully written. -1 indicates that the address byte was NACKed.
 */
// Returns bytes written, -1 indicates START failed (no such device?)
int i2c_write(unsigned char slave_addr, unsigned char *buf, unsigned char len){
    while (UCB2CTLW0 & UCTXSTP);        // if there's a pending stop from a previous transaction, wait for it to be sent
    UCB2I2CSA = slave_addr;// set slave address
    _i2c_tx_bytes = _i2c_tx_byte_ctr = len;
    _i2c_tx_data = buf;
    _i2c_did_nack = false;
    UCB2CTLW0 |= UCTR | UCTXSTT;        // I2C TX, start condition
    unsigned short sr;
    sr = __get_SR_register();         //store existing interrupt state
    while (_i2c_tx_byte_ctr && !_i2c_did_nack) {
        __bis_SR_register(LPM0_bits | GIE);  //go into LPM0 until data is sent
    }
    __set_interrupt_state(sr);          //restore previous interrupt state
    if (_i2c_did_nack) {
        // Last byte not successfully sent
        _i2c_tx_byte_ctr++;
    }
    return len - _i2c_tx_byte_ctr;
}


#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = EUSCI_B2_VECTOR
__interrupt void USCI_B2_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(EUSCI_B2_VECTOR))) USCI_B2_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(UCB2IV, USCI_I2C_UCBIT9IFG))
    {
        case USCI_NONE:          break;     // Vector 0: No interrupts
        case USCI_I2C_UCALIFG:   break;     // Vector 2: ALIFG
        case USCI_I2C_UCNACKIFG:            // Vector 4: NACKIFG
            //UCB2CTLW0 |= UCTXSTT;           // resend start if NACK
            _i2c_did_nack = true;
            __bic_SR_register_on_exit(LPM0_bits); // exit LPM0 to abort
            break;
        case USCI_I2C_UCSTTIFG:  break;     // Vector 6: STTIFG
        case USCI_I2C_UCSTPIFG:  break;     // Vector 8: STPIFG
        case USCI_I2C_UCRXIFG3:  break;     // Vector 10: RXIFG3
        case USCI_I2C_UCTXIFG3:  break;     // Vector 12: TXIFG3
        case USCI_I2C_UCRXIFG2:  break;     // Vector 14: RXIFG2
        case USCI_I2C_UCTXIFG2:  break;     // Vector 16: TXIFG2
        case USCI_I2C_UCRXIFG1:  break;     // Vector 18: RXIFG1
        case USCI_I2C_UCTXIFG1:  break;     // Vector 20: TXIFG1
        case USCI_I2C_UCRXIFG0:  break;     // Vector 22: RXIFG0
        //for my next trick, I'll implement reading!
        case USCI_I2C_UCTXIFG0:             // Vector 24: TXIFG0
            if (_i2c_tx_byte_ctr)                  // Check TX byte counter
            {
                // Load TX buffer
                UCB2TXBUF = _i2c_tx_data[_i2c_tx_bytes - _i2c_tx_byte_ctr];
                _i2c_tx_byte_ctr--;                // Decrement TX byte counter
            }
            else
            {
                UCB2CTLW0 |= UCTXSTP;       // send stop condition
                UCB2IFG &= ~UCTXIFG;        // Clear USCI_B2 TX int flag
                __bic_SR_register_on_exit(LPM0_bits); // Exit LPM0
            }
            break;
        case USCI_I2C_UCBCNTIFG: break;     // Vector 26: BCNTIFG
        case USCI_I2C_UCCLTOIFG: break;     // Vector 28: clock low timeout
        case USCI_I2C_UCBIT9IFG: break;     // Vector 30: 9th bit
        default: break;
    }
}

/**
 * @brief Delay for a specified number of microseconds.
 *        Not calibrated yet, may be significantly wrong.
 */

void delay_micros(unsigned long micros)
{

    for(; micros != 0; micros--) {
        __delay_cycles(CYC_PER_US - FOR_LOOP_CYC);
    }
    __delay_cycles(CYC_PER_US - FOR_LOOP_CYC);
}

/**
 * @brief Set input/output mode of a GPIO pin
 *
 * Configures a GPIO pin for input or output, with optional pull-up or
 * pull-down resistor for inputs.  Currently does not check if the pin has
 * been configured for a module function, which would prevent the parameters
 * configured here from having any effect.
 *
 * @param pin The pin to configure.  Takes a single byte, whose upper nibble
 *            specifies the port, and lower nibble specifies the pin.  E.g.
 *            0x23 translates to port 2, pin 3
 * @param mode The mode to set the pin to.  Use one of the macros OUTPUT,
 *             INPUT, INPUT_PULLUP, or INPUT_PULLDOWN
 */

void gpio_config(int pin, int mode)
{
    unsigned int port = (pin >> 4) & 0x0F;
    unsigned int pin_num = (pin >> 0) & 0x07;
    unsigned char pull_enable = (mode & 0x20);
    unsigned char pull_direction = (mode & 0x10);

    /* Set pin direction */
    char cur_dir = *gpio_dir_for_port[port];

    *gpio_dir_for_port[port] = (mode & OUTPUT) ? cur_dir | (1 << pin_num) : cur_dir & ~(1 << pin_num);

    /* Set pull resistor to up or down, if enabled */
    if(pull_enable)
    {
        gpio_write(pin, pull_direction);
    }

    /* Enable or disable pull up/down resistor */
    char cur_ren = *gpio_ren_for_port[port];

    *gpio_ren_for_port[port] = pull_enable ? cur_ren | (1 << pin_num) : cur_ren & ~(1 << pin_num);
}

/**
 * @brief Write a value to a GPIO pin
 *
 * If the pin has been configured as an output, write the specified state to
 * it. If the pin is an input with the pull resistor enabled, it will instead
 * set the state to which the pin is pulled.
 *
 * @param pin The pin to configure.  Takes a single byte, whose upper nibble
 *            specifies the port, and lower nibble specifies the pin.  E.g.
 *            0x23 translates to port 2, pin 3
 * @param value The value to write to the pin
 */

void gpio_write(int pin, int value)
{
    unsigned int port = (pin >> 4) & 0x0F;
    unsigned int pin_num = (pin >> 0) & 0x07;

    char cur_dir = *gpio_out_for_port[port];

    *gpio_out_for_port[port] = value ? cur_dir | (1 << pin_num) : cur_dir & ~(1 << pin_num);
}

/**
 * @brief Read from a GPIO pin
 *
 * @param pin The pin to configure.  Takes a single byte, whose upper nibble
 *            specifies the port, and lower nibble specifies the pin.  E.g.
 *            0x23 translates to port 2, pin 3
 * @return The state of the GPIO pin's bit in the PxIN register
 */

unsigned char gpio_read(int pin)
{
    unsigned int port = (pin >> 4) & 0x0F;
    unsigned int pin_num = (pin >> 0) & 0x07;

    return (*gpio_in_for_port[port] & (1 << pin_num)) != 0;
}

/**
 * @brief Perform a one-byte transfer over SPI
 *
 * @param b The byte to send
 * @return The byte received
 */
static unsigned char spi_xfer(unsigned char b)
{

    /* Wait for previous tx to complete. */
    while (!(UCB1IFG & UCTXIFG));

    /* Setting TXBUF clears the TXIFG flag. */
    UCB1TXBUF = b;

    /* Wait for a rx character. */
    while (!(UCB1IFG & UCRXIFG));

    /* Reading clears RXIFG flag. */
    return UCB1RXBUF;
}

/**
 * @brief Initialize UCB1 for SPI operation
 *
 * Configures UCB1 to operate in SPI mode, with SCLK on P5.2, MISO on P5.1,
 * and MOSI on P5.0, with speed controlled by SMLCK and the SPI_CLKDIV macro
 */
void spi_init()
{
    /* Put USCI in reset mode, source clock from SMCLK. */
    UCB1CTLW0 = UCSWRST | UCSSEL_2;

    /* SPI mode 0, MSB first, synchronous, master mode, 3 wire SPI*/
    UCB1CTLW0 |= UCCKPH | UCMSB | UCSYNC | UCMST | UCMODE0;

    /* Set pins to SPI mode. (FR5994 version) */
    //SCK P5.2
    P5SEL1 &= ~(1 << 2);
    P5SEL0 |= (1 << 2);

    //SIMO P5.0
    P5SEL1 &= ~(1 << 0);
    P5SEL0 |= (1 << 0);

    //SOMI P5.1
    P5SEL1 &= ~(1 << 1);
    P5SEL0 |= (1 << 1);

    /* Set speed */
    UCB1BR0 = SPI_CLKDIV & 0xFF;
    UCB1BR1 = (SPI_CLKDIV >> 8 ) & 0xFF;

    /* Release USCI for operation. */
    UCB1CTLW0 &= ~UCSWRST;
}

/**
 * @brief Send a byte over SPI while ignoring return traffic
 */
void spi_write_byte(unsigned char b)
{
    spi_xfer(b);
}

/**
 * @brief Write a buffer over SPI while ignoring return traffic.  Blocking.
 *
 * @param len The number of bytes to write
 * @param data Pointer to the bytes to be sent
 */
void spi_write_data(int len, const unsigned char *data)
{
    unsigned int i;
    for(i = 0; i < len; i++) {
        spi_xfer(data[i]);
    }
}

/**
 * @brief Read from SPI into a buffer, while sending 0xFF.  Blocking.
 *
 * @param len The number of bytes to send and read
 * @param data The buffer into which received data will be written
 */
void spi_read_data(int len, unsigned char *data)
{
    unsigned int i;
    for(i = 0; i < len; i++) {
        data[i] = spi_xfer(0xFF);
    }
}

/**
 * @brief Enable a GPIO pin interrupt
 *
 * Enables an edge-triggered interrupt on a GPIO pin.  Does not handle
 * configuring the input state of the pin.  Clears pending interrupts
 * before enabling the new interrupt.  Won't do much good unless you have
 * set up the appropriate port ISR to handle the pin in question.
 *
 * @param pin The pin to configure.  Takes a single byte, whose upper nibble
 *            specifies the port, and lower nibble specifies the pin.  E.g.
 *            0x23 translates to port 2, pin 3
 * @param edge Use the RISING or FALLING macro
 */
int enable_pin_interrupt(int pin, int edge)
{
    __disable_interrupt();
    unsigned int port = (pin >> 4) & 0x0F;
    unsigned int pin_num = (pin >> 0) & 0x07;

    char cur_ies = *gpio_ies_for_port[port];
    *gpio_ies_for_port[port] = edge ? cur_ies | (1 << pin_num) : cur_ies & ~(1 << pin_num);

    char cur_ie = *gpio_ie_for_port[port];
    *gpio_ie_for_port[port] = cur_ie | (1 << pin_num);

    // Clear any pending interrupts on this pin
    // You may not want this in the future...
    *gpio_ifg_for_port[port] &= ~(1 << pin_num);

    __enable_interrupt(); // enable all interrupts

    return 0;
}

/**
 * @brief Get the microcontroller's unique ID
 *
 * The MSP430 implementation reads the 32-bit wafer/lot ID and the 16-bit X and Y
 * die positions.  This is globally unique among MSP430s.
 *
 * @param buf  A buffer of at least 8 bytes into which the ID will be written
 */
void get_device_uid(unsigned char* buf){
    uint8_t *tlv = (uint8_t*)TLV_SILICON_UID_BASE;
    int i=0;
    for(;i<8; i++){
        buf[i] = *tlv;
        tlv++;
    }
}
