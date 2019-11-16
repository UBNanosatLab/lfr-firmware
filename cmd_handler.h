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

#ifndef CMD_HANDLER_H
#define CMD_HANDLER_H

#define CONFIG_LEN (30)

/**
 * Send reply character to the flight computer
 * @param c the character to send
 */
int host_reply_putc(uint8_t c);

/**
 * No-OPeration
 * Replies with success always
 */
void cmd_nop();

/**
 * Reset
 * Forces a reset of the MCU
 */
void cmd_reset();

/**
 * Get transmit power
 * Returns the transmit power level (in arbitrary units)
 * TODO: Use dBm?
 */
void cmd_get_txpwr();

/**
 * Set transmit power
 * Sets the transmitter power level (in arbitrary units)
 * TODO: Use dBm?
 * @param pwr the transmitter power level (gate bias)
 */
void cmd_set_txpwr(uint16_t pwr);

/**
 * Transmit the provided data
 * @param len the length of the data in bytes
 * @param data pointer to the data
 */
void cmd_tx_data(int len, uint8_t *data);

/**
 * Set configuration
 * @param len the length of the cfg data in bytes
 * @param data pointer to the cfg data
 */
void cmd_set_cfg(int len, uint8_t *data);

/**
 * Get configuration
 * Returns the configuration data
 */
void cmd_get_cfg();

/**
 * Save configuration
 * Writes the current configuration data to non-volatile storage
 */
void cmd_save_cfg();

/**
 * Load default configuration
 * Loads the default (factory) configuration into the volatile settings
 */
void cmd_cfg_default();


/**
 * Set frequency
 * Changes the frequency
 * @param freq desired frequency in Hz
 */
void cmd_set_freq(uint32_t rx_freq, uint32_t tx_freq);

/**
 * Abort TX
 * Immediately kills the transmitter and returns to normal receive mode
 */
void cmd_abort_tx();

/**
 * Transmit psuedo-random sequence
 * Transmits a psuedo-random sequence
 */
void cmd_tx_psr();

/**
 * Get received packet
 * Returns the next packet in the receive queue
 */
void cmd_rx_data();

/**
 * Get the depth of the transmit queue
 * Returns the (16-bit) depth of the queue
 */
void cmd_get_queue_depth();

/**
 * Send an error back
 * @param err the (positive) error code
 */
void cmd_err(int err);

#endif
