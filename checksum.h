/*
 * checksum.h
 *
 *  Created on: Feb 13, 2019
 *      Author: brian
 */

#ifndef CHECKSUM_H_
#define CHECKSUM_H_
#include <stdint.h>

uint16_t fletcher(uint16_t old_checksum, uint8_t c);

#endif /* CHECKSUM_H_ */
