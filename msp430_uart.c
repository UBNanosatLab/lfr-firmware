/*
 * msp430_uart.c
 *
 *  Created on: Feb 13, 2018
 *      Author: brian
 */
#include <msp430.h>
#include "rbuf.h"
#define UART_PINS_BOOSTERPACK
//#define UART_PINS_BACKCHANNEL

//case-dependent aliases for pins and peripheral things
#ifdef UART_PINS_BOOSTERPACK
#define PxSEL1 P6SEL1
#define PxSEL0 P6SEL0
#define UCAxCTLW0 UCA3CTLW0
#define UCAxBRW UCA3BRW
#define UCAxMCTLW UCA3MCTLW
#define UCAxIE UCA3IE
#define UCAxIV UCA3IV
#define UCAxIFG UCA3IFG
#define UCAxTXBUF UCA3TXBUF
#define UCAxRXBUF UCA3RXBUF
#define EUSCI_Ax_VECTOR EUSCI_A3_VECTOR
#define USCI_Ax_ISR USCI_A3_ISR
#endif

#ifdef UART_PINS_BACKCHANNEL
#define PxSEL1 P2SEL1
#define PxSEL0 P2SEL0
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
#endif

rbuf txbuf, rxbuf;
uint8_t uartInitialized=0;

    /*
     * initUartPins()
     * Sets Port 6 pins 0 and 1 to eUSCI_A3 TXD and RXD, respectively
     */
void uart_init_pins(){
#ifdef UART_PINS_BACKCHANNEL
    PxSEL1 |= (BIT0 | BIT1);
    PxSEL0 &= ~(BIT0 | BIT1);    // USCI_A3 UART operation
#endif

#ifdef UART_PINS_BOOSTERPACK
    // Opposite for boosterpack as for backchannel. WHY TI?
    PxSEL0 |= (BIT0 | BIT1);
    PxSEL1 &= ~(BIT0 | BIT1);    // USCI_A3 UART operation
#endif
}


int uart_init(){
    uart_init_pins();
    rbuf_init(&rxbuf);
    rbuf_init(&txbuf);
    // Configure USCI_A3 for UART mode
    UCAxCTLW0 = UCSWRST;                    // Put eUSCI in reset
    UCAxCTLW0 |= UCSSEL__SMCLK;             // CLK = SMCLK
    // See User's Guide Table 30-5 on page 779
    UCAxBRW = 52;
    //                    UCBRFx   UCBRSx
    UCAxMCTLW |= UCOS16 | 0x0001 | 0x4900;
    UCAxCTLW0 &= ~UCSWRST;                  // Initialize eUSCI
    //UCAxIE |= UCRXIE | UCTXIE;                       // Enable USCI_Ax RX interrupt
    UCAxIE |= UCRXIE;
    __bis_SR_register(GIE);   // enable global interrupts
    return 0;
}

int uart_putc(char c){
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

int uart_putbuf(char* buf, int len){
    int i;
    for(i=0;i<len;i++){
        uart_putc(buf[i]);
    }
    return 0;
}
/*
 * uart_available: query the size of the receive ring buffer
 * returns the number of bytes waiting in the RX ring buffer
 */
int uart_available(){
    return rbuf_size(&rxbuf);
}

/*
 * uart_getc: get a received character out of the UART receive ring buffer
 * Careful! blocks until there's something to give you.  Check uart_available() first!
 */
char uart_getc(){
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
void __attribute__ ((interrupt(EUSCI_A3_VECTOR))) USCI_Ax_ISR (void)
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
