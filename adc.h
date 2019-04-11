/*
 * adc.h
 *
 *  Created on: Feb 14, 2019
 *      Author: brian
 */

#ifndef ADC_H_
#define ADC_H_

void adc_init();

int gpio_to_adc(unsigned char pin);
int adc_to_gpio(unsigned char channel);
/*
 * adc_read: Accepts an ADC channel ID A0..A16
 */
int adc_read(unsigned int chan);

#endif /* ADC_H_ */
