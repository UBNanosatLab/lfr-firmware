/*
 * adc.c
 *
 *  Created on: Feb 14, 2019
 *      Author: brian
 */
#include <msp430.h>
#include "adc.h"
#include "error.h"
#include "mcu.h"

const unsigned char analog_pins[] = {
    0x10,
    0x11,
    0x12,
    0x13,
    0x14,
    0x15,
    0x23,
    0x24,
    0x40,
    0x41,
    0x42,
    0x43,
    0x30,
    0x31,
    0x32,
    0x33,
    0x74
};


void adc_init(){
    // By default, REFMSTR=1 => REFCTL is used to configure the internal reference
    while(REFCTL0 & REFGENBUSY);            // If ref generator busy, WAIT
    //REFCTL0 |= REFVSEL_0 | REFON;           // Select internal ref = 1.2V
                                            // Internal Reference ON
    REFCTL0 |= REFVSEL_2 | REFON;           //Internal 2.5V reference

    // Configure ADC12
    ADC12CTL0 = ADC12SHT0_4 | ADC12ON;
    ADC12CTL1 = ADC12SHP;                   // ADCCLK = MODOSC; sampling timer
    ADC12CTL2 |= ADC12RES_2;                // 12-bit conversion results
    ADC12CTL3 |= ADC12TCMAP;                // Connect internal temperature sensor to channel 30d/1Eh
    //ADC12IER0 |= ADC12IE0;                  // Enable ADC conv complete interrupt
    ADC12MCTL0 |= ADC12VRSEL_1;             //Select the internal reference as VREF

    while(!(REFCTL0 & REFGENRDY));          // Wait for reference generator
                                            // to settle
}

/*
 * Search the ADC-to-GPIO map for the specified pin.  Pins not muxable to the ADC will return -EINVAL
 */
int gpio_to_adc(unsigned char iopin){
    unsigned char i;
    for(i=0; i<sizeof(analog_pins); i++){
        if(analog_pins[i] == iopin){
            return i;
        }
    }
    return -EINVAL;
}

/*
 * Look up the GPIO pin associated to an ADC channel
 */
int adc_to_gpio(unsigned char channel){
    if(channel < sizeof(analog_pins)){
        return analog_pins[channel];
    } else {
        return -EINVAL;
    }
}

/*
 * Take an ADC reading from the specified channel
 */
int adc_read(unsigned int chan){
    unsigned int adcval;
    // channel 0x1E is connected to the internal temp sensor; allow that or anything in the gpio-analog pin map
    if(!(chan == ADC_CHAN_TEMP || chan < sizeof(analog_pins))){
        return -EINVAL;
    }
    ADC12MCTL0 &= ~(BIT0|BIT1|BIT2|BIT3|BIT4); // clear the channel select bits in ADC12MCTL0
    ADC12MCTL0 |= chan;                 // Select the channel
    ADC12CTL0 |= ADC12ENC | ADC12SC;    // Sampling and conversion start
    while(!(ADC12IFGR0 & ADC12IFG0));   // wait for the conversion to finish
    adcval = ADC12MEM0;                 // get the result
    ADC12CTL0 &= ~(ADC12ENC);           // stop the ADC
    return adcval;
}
