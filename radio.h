#ifndef MAIN_H
#define MAIN_H

#include <stdbool.h>
#include <stdint.h>

/* Global variables */
extern struct si446x_device dev;
extern uint8_t buf[255];
extern volatile bool do_pong;
extern uint16_t tx_gate_bias;
extern uint8_t sys_stat;

void set_cmd_flag(uint8_t flag);
int set_gate_bias(uint16_t bias);
void rx_cb(struct si446x_device *dev, int err, int len, uint8_t *data);
void tx_cb(struct si446x_device *dev, int err);

#endif
