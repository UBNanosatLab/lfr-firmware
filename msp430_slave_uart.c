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
#include "rbuf.h"

//case-dependent aliases for pins and peripheral things
#define UCAxCTLW0 UCA0CTLW0
#define UCAxBRW UCA0BRW
#define UCAxMCTLW UCA0MCTLW
#define UCAxIE UCA0IE
#define UCAxIV UCA0IV
#define UCAxIFG UCA0IFG
#define UCAxTXBUF UCA0TXBUF
#define UCAxRXBUF UCA0RXBUF
#define EUSCI_Ax_VECTOR EUSCI_A0_VECTOR
#define USCI_Ax_ISR USCI_A0_ISR

rbuf txbuf, rxbuf;

void slave_uart_init_pins(){

    // Note: the correct setting here varies by port
    // USCI_A0 UART operation
    P2SEL1 |= (BIT1 | BIT0);
    P2SEL0 &= ~(BIT1 | BIT0);
}


int slave_uart_init(){
    slave_uart_init_pins();
    rbuf_init(&rxbuf);
    rbuf_init(&txbuf);
    // Configure USCI_Ax for UART mode
    UCAxCTLW0 = UCSWRST;                // Put eUSCI in reset
    UCAxCTLW0 |= UCSSEL__SMCLK;         // CLK = SMCLK

    // Assuming 8 MHz SMCLK
    // See User's Guide Table 30-5 on page 779

    // 9600 baud
//    UCAxBRW = 52;
//    //                    UCBRFx   UCBRSx
//    UCAxMCTLW |= UCOS16 | 0x0001 | 0x4900;

    // 38400
//    UCAxBRW = 13;
//    //                    UCBRFx   UCBRSx
//    UCAxMCTLW = UCOS16 | UCBRF_0 | 0x8400;

    // 115200
    UCAxBRW = 4;
    //                    UCBRFx   UCBRSx
    UCAxMCTLW = UCOS16 | UCBRF_5 | 0x5500;


    UCAxCTLW0 &= ~UCSWRST;              // Initialize eUSCI

    UCAxIE |= UCRXIE;                   // Enable USCI_Ax RX interrupt
    __bis_SR_register(GIE);             // enable global interrupts
    return 0;
}

int slave_uart_putc(char c){
    int err;
    unsigned short sreg = __get_interrupt_state();
    do{
        __disable_interrupt();      //disable interrupts while accessing UART buffer
        err=rbuf_put(&txbuf, c);    //attempt to put c in the transmit queue
        __set_interrupt_state(sreg);//return interrupts to previous enable/disable state
    }while(err<0);                  //and keep trying until it fits
    UCAxIE |= UCTXIE; //enable the interrupt if it wasn't already
    return 0;
}

int slave_uart_putbuf(char* buf, int len){
    int i;
    for(i=0;i<len;i++){
        slave_uart_putc(buf[i]);
    }
    return 0;
}
/*
 * uart_available: query the size of the receive ring buffer
 * returns the number of bytes waiting in the RX ring buffer
 */
int slave_uart_available(){
    return rbuf_size(&rxbuf);
}

/*
 * uart_getc: get a received character out of the UART receive ring buffer
 * Careful! blocks until there's something to give you.  Check uart_available() first!
 */
char slave_uart_getc(){
    char c;
    int err;
    unsigned short sreg = __get_interrupt_state();
    do{
        __disable_interrupt();          //disable interrupts while accessing UART buffer
        err=rbuf_get(&rxbuf, &c);       //attempt to get a character
        __set_interrupt_state(sreg);    //return interrupts to previous enable/disable state
    }while(err<0);                      //and keep trying until a character is acquired
    return c;
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=EUSCI_Ax_VECTOR
__interrupt void USCI_Ax_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(EUSCI_Ax_VECTOR))) USCI_Ax_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(UCAxIV, USCI_UART_UCTXCPTIFG))
    {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG:
            //UCAxTXBUF = UCAxRXBUF;
            //move the received byte into the RX ring buffer
            rbuf_put(&rxbuf, UCAxRXBUF);
            break;
        case USCI_UART_UCTXIFG:
            //send from the TX ring buffer if possible
            if(rbuf_size(&txbuf)){
                char c;
                rbuf_get(&txbuf, &c);
                UCAxTXBUF = c;
            } else { //tx buffer's empty
                UCAxIE &= ~(UCTXIE); //disable the TX interrupt until there's something to send
                UCAxIFG |= UCTXIFG; //and set the interrupt flag so the next transmission will start
            }
            break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
        default: break;
    }
    LPM4_EXIT; //This clears all the LPM bits, so it will leave the chip in run mode after the ISR
}
