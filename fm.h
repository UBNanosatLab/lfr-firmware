/*
 * fm.h
 *
 *  Created on: Jan 19, 2020
 *      Author: iracigt
 */

#ifndef FM_H_
#define FM_H_

int fm_init();
int fm_set_msg(uint8_t len, uint8_t *data);
int fm_start();
int fm_stop();
void fm_step();


#endif /* FM_H_ */
