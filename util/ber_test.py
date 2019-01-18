#!/usr/bin/env python2
# -*- coding: utf-8 -*-

##################################################
# GNU Radio Python Flow Graph
##################################################

from gnuradio import analog
from gnuradio import blocks
from gnuradio import eng_notation
from gnuradio import filter
from gnuradio import gr
from gnuradio import uhd
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from optparse import OptionParser
import limesdr
import math
import numpy
import time
import sys
import pmt
import si446x
import serial
import struct
import logging
from threading import Timer 
import thread
import os
from enum import Enum

class auto_ber_test(gr.top_block):

    def __init__(self, samp_rate, tune_offset, source, sink, sw_gain=0):
        gr.top_block.__init__(self, "Auto BER Test")

        self.source = source
        self.sink = sink

        ##################################################
        # Variables
        ##################################################
        self.sym_rate = sym_rate = 10000
        self.samp_per_sym = samp_per_sym = int(samp_rate / sym_rate)
        self.bt = bt = 0.5
        self.gaussian_taps = gaussian_taps = firdes.gaussian(1.0,samp_per_sym, bt, 4 * samp_per_sym)
        self.tx_taps = tx_taps = numpy.convolve(numpy.array(gaussian_taps),numpy.array((1,) * samp_per_sym))
        self.tx_deviation = tx_deviation = 20e3
        self.sw_gain = sw_gain

        ##################################################
        # Blocks
        ##################################################

        # Disable CRC generation to make sure we're in the right CRC_ENABLE mode
        self.var_len_packet_creator = si446x.var_len_packet_creator(8, 0x2DD4, True, 0x0000)
        self.pdu_to_tagged_stream = blocks.pdu_to_tagged_stream(blocks.byte_t, 'packet_len')

        self.uchar_to_float = blocks.uchar_to_float()
        self.sub_one_half = blocks.add_const_vff((-0.5, ))
        self.mult_two = blocks.multiply_const_vff((2, ))

        self.gaussian_filter = filter.interp_fir_filter_fff(samp_per_sym, (tx_taps))
        self.gaussian_filter.declare_sample_delay(0)
        self.freq_xlating_fir_filter = filter.freq_xlating_fir_filter_ccc(1, ([10**(sw_gain / 20.0)]), tune_offset, samp_rate)
        self.freq_mod = analog.frequency_modulator_fc(2 * math.pi * tx_deviation / samp_rate)

        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.source, 'out'), (self.var_len_packet_creator, 'in'))    
        self.msg_connect((self.var_len_packet_creator, 'out'), (self.pdu_to_tagged_stream, 'pdus')) 
        self.connect((self.pdu_to_tagged_stream, 0), (self.uchar_to_float, 0))    
        self.connect((self.freq_mod, 0), (self.freq_xlating_fir_filter, 0))    
        self.connect((self.freq_xlating_fir_filter, 0), (self.sink, 0))
        self.connect((self.gaussian_filter, 0), (self.freq_mod, 0))    
        self.connect((self.mult_two, 0), (self.gaussian_filter, 0))    
        self.connect((self.sub_one_half, 0), (self.mult_two, 0))    
        self.connect((self.uchar_to_float, 0), (self.sub_one_half, 0))    

    def get_sw_gain(self):
        return self.sw_gain

    def set_sw_gain(self, sw_gain):
        self.sw_gain = sw_gain
        self.freq_xlating_fir_filter.set_taps(([10**(self.sw_gain / 20.0)]))

class msg_source(gr.basic_block):

	def __init__(self):

		gr.basic_block.__init__(
			 self,
			 name="msg_source",
			 in_sig=None,
			 out_sig=None)

		self.message_port_register_out(pmt.intern('out'))
	
	def transmit(self, data):
		vector = pmt.make_u8vector(len(data), 0)
		for i, c in enumerate(data):
			pmt.u8vector_set(vector, i, ord(data[i]))
		pdu = pmt.cons(pmt.make_dict(), vector)
		self.message_port_pub(pmt.intern('out'), pdu)


########################################
#         LFR Command Handling         #
########################################

SYNCWORD_H = 0xBE
SYNCWORD_L = 0xEF

class ParseState(Enum):
    SYNC_H = 0
    SYNC_L = 1
    FLAGS = 2
    CMD = 3
    PAYLOAD_LEN = 4
    PAYLOAD = 5
    CHKSUM_H = 6
    CHKSUM_L = 7

class Command(Enum):
    NOP = 0x00
    RESET = 0x01
    TXDATA = 0x02
    GET_TXPWR = 0x03
    SET_TXPWR = 0x04

    GET_CFG = 0x08
    SET_CFG = 0x09
    SAVE_CFG = 0x0A
    CFG_DEFAULT = 0x0B

    RXDATA = 0x10

    ERROR = 0x7F

    REPLY = 0x80

def feltcher(chksum, byte):
    lsb = chksum & 0xFF
    msb = chksum >> 8
    msb += byte
    msb &= 0xFF
    lsb += msb
    lsb &= 0xFF
    return (msb << 8) | lsb

def compute_chksum(data):
    chksum = 0
    for x in data:
        chksum = feltcher(chksum, ord(x))
    return chksum

class RadioException(Exception):

    def __init__(self, code):
        self.code = code

        if code == 0:
            self.error = 'ESUCCESS'
            self.msg = 'command succeeded'
        elif code == 1:
            self.error = 'ETIMEOUT'
            self.msg = 'timeout waiting for CTS'
        elif code == 2:
            self.error = 'EWRONGPART'
            self.msg = 'unsupported part number'
        elif code == 3:
            self.error = 'EINVAL'
            self.msg = 'invalid parameter'
        elif code == 4:
            self.error = 'EINVALSTATE'
            self.msg = 'invalid internal state'
        elif code == 5:
            self.error = 'ETOOLONG'
            self.msg = 'packet too long'
        elif code == 6:
            self.error = 'ECHKSUM'
            self.msg = 'invalid checksum'
        elif code == 7:
            self.error = 'EBUSY'
            self.msg = 'pending operation'
        elif code == 8:
            self.error = 'ERXTIMEOUT'
            self.msg = 'Si446x RX timed out (zero len bug?)'
        else:
            self.error = 'UNKNOWN'
            self.msg = 'An unknown error occurred'

    def __str__(self):
        return 'RadioException(' + self.error + '): ' + self.msg

class Radio:

    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baud)
        self.state = ParseState.SYNC_H

    def tx(self, data):
        pkt = chr(Command.TXDATA.value)
        pkt += chr(len(data))
        pkt += data
        chksum = compute_chksum(pkt)
        pkt += chr(chksum >> 8)
        pkt += chr(chksum & 0xFF)
        pkt = '\xBE\xEF' + pkt

        self.ser.write(pkt)

        (flags, cmd, pay) = self.recv()

        if cmd  == Command.ERROR.value | Command.REPLY.value:
            err = RadioException(ord(pay[0]))
            if err.error == 'EBUSY':
                self.tx(data)
            else:
                raise err

        elif Command.TXDATA.value | Command.REPLY.value:
            return
        else:
            raise Exception('Unexpected response: ' + str((flags, cmd, pay)))

    def rx(self):
        (flags, cmd, pay) = self.recv()

        if cmd  == Command.ERROR.value | Command.REPLY.value:
            raise RadioException(pay[0])
        elif Command.RXDATA.value | Command.REPLY.value:
            return pay
        else:
            raise Exception('Unexpected response: ' + str((flags, cmd, pay)))

    def recv(self):

        payload = ''

        while True:
            c = ord(self.ser.read(1))

            if self.state == ParseState.SYNC_H:
                if c == SYNCWORD_H:
                    self.state = ParseState.SYNC_L
            elif self.state == ParseState.SYNC_L:
                if c == SYNCWORD_L:
                    self.state = ParseState.FLAGS
                elif c == SYNCWORD_H:
                    self.state = ParseState.SYNC_L
                else:
                    self.state = ParseState.SYNC_H
            elif self.state == ParseState.FLAGS:
                flags = c
                self.state = ParseState.CMD
            elif self.state == ParseState.CMD:
                cmd = c
                self.state = ParseState.PAYLOAD_LEN
            elif self.state == ParseState.PAYLOAD_LEN:
                length = c
                # TODO: Validate len for cmd
                if (length):
                    self.state = ParseState.PAYLOAD
                else:
                    chksum = compute_chksum(''.join([chr(flags), chr(cmd), chr(0)]))
                    self.state = ParseState.CHKSUM_H

            elif self.state == ParseState.PAYLOAD:
                payload += chr(c)
                length -= 1
                self.state = ParseState.PAYLOAD
                if (length == 0):
                    chksum = compute_chksum(''.join([chr(flags), chr(cmd), chr(len(payload))]) + payload)
                    self.state = ParseState.CHKSUM_H
            elif self.state == ParseState.CHKSUM_H:
                if (c == chksum >> 8):
                    self.state = ParseState.CHKSUM_L
                else:
                    # TODO: Handle error
                    pass
                    self.state = ParseState.SYNC_H
                    break
            elif self.state == ParseState.CHKSUM_L:
                if (c != chksum & 0xFF):
                    # TODO: Handle error
                    pass
                self.state = ParseState.SYNC_H
                break

        return (flags, cmd, payload)

    def get_cfg(self):
        pkt = chr(Command.GET_CFG.value)
        pkt += chr(0)
        chksum = compute_chksum(pkt)
        pkt += chr(chksum >> 8)
        pkt += chr(chksum & 0xFF)
        pkt = '\xBE\xEF' + pkt

        self.ser.write(pkt)

        (flags, cmd, pay) = self.recv()

        if cmd  == Command.ERROR.value | Command.REPLY.value:
            err = RadioException(pay[0])
            raise err

        elif Command.GET_CFG.value | Command.REPLY.value:

            cfg = {}

            cfg['cfg_ver'], cfg['freq'], cfg['deviation'], cfg['data_rate'], \
            cfg['txco_vpull'], cfg['tx_gate_bias'], cfg['tx_vdd'], \
            cfg['pa_ilimit'], cfg['tx_vdd_delay'], cfg['flags'],\
            cfg['callsign'] = struct.unpack("!BIHHHHHHHH8s", pay)

            return cfg
        else:
            raise Exception('Unexpected response: ' + str((flags, cmd, pay)))

    def set_cfg(self, cfg_in):

        cfg = dict(cfg_in)

        data = struct.pack("!BIHHHHHHHH8s", \
            cfg['cfg_ver'], cfg['freq'], cfg['deviation'], cfg['data_rate'], \
            cfg['txco_vpull'], cfg['tx_gate_bias'], cfg['tx_vdd'], \
            cfg['pa_ilimit'], cfg['tx_vdd_delay'], cfg['flags'], \
            cfg['callsign'])

        pkt = chr(Command.SET_CFG.value)
        pkt += chr(len(data))
        pkt += data
        chksum = compute_chksum(pkt)
        pkt += chr(chksum >> 8)
        pkt += chr(chksum & 0xFF)
        pkt = '\xBE\xEF' + pkt

        self.ser.write(pkt)

        (flags, cmd, pay) = self.recv()

        if cmd  == Command.ERROR.value | Command.REPLY.value:
            err = RadioException(ord(pay[0]))
            raise err

        elif Command.SET_CFG.value | Command.REPLY.value:
            return
        else:
            raise Exception('Unexpected response: ' + str((flags, cmd, pay)))


    def save_cfg(self):
        pkt = chr(Command.SAVE_CFG.value)
        pkt += chr(0)
        chksum = compute_chksum(pkt)
        pkt += chr(chksum >> 8)
        pkt += chr(chksum & 0xFF)
        pkt = '\xBE\xEF' + pkt

        self.ser.write(pkt)

        (flags, cmd, pay) = self.recv()

        if cmd  == Command.ERROR.value | Command.REPLY.value:
            err = RadioException(ord(pay[0]))
            raise err

        elif Command.GET_CFG.value | Command.REPLY.value:
            return
        else:
            raise Exception('Unexpected response: ' + str((flags, cmd, pay)))

    def cfg_default(self):
        pkt = chr(Command.CFG_DEFAULT.value)
        pkt += chr(0)
        chksum = compute_chksum(pkt)
        pkt += chr(chksum >> 8)
        pkt += chr(chksum & 0xFF)
        pkt = '\xBE\xEF' + pkt

        self.ser.write(pkt)

        (flags, cmd, pay) = self.recv()

        if cmd  == Command.ERROR.value | Command.REPLY.value:
            err = RadioException(ord(pay[0]))
            raise err

        elif Command.GET_CFG.value | Command.REPLY.value:
            return
        else:
            raise Exception('Unexpected response: ' + str((flags, cmd, pay)))

########################################
#                  Main                #
########################################

def main():
    ########################################
    #              Configuration           #
    ########################################

    port = "/dev/tty.SLAB_USBtoUART72"
    baud = 115200

    freq = 434.000e6
    tune_offset = 250e3
    samp_rate = 1e6

    usrp = True
    lime = False
    lime_serial = '1D3AC9891D2233'

    num_pkts = 100

    hw_gain = 20
    gain_offset = -105

    lvls = numpy.linspace(-15.0, -5.0, 11).tolist()

    ########################################

    sys.stdout = open('/dev/null', 'w')
    logging.basicConfig(format='%(message)s', stream=sys.stderr, level=logging.DEBUG)

    if usrp:
        sink = uhd.usrp_sink(
            ",".join(("", "")),
            uhd.stream_args(
                cpu_format="fc32",
                channels=range(1),
            ),
        )
        sink.set_samp_rate(samp_rate)
        sink.set_center_freq(freq + tune_offset, 0)
        sink.set_gain(hw_gain, 0)
        sink.set_antenna('TX/RX', 0)
    elif lime:
        sink = limesdr.sink(lime_serial, 1, 1, 0, 0, '', freq + tune_offset, samp_rate, 0, 0, 10e6, 0, 10e6, 1, 1, 1, 1, 
                            5e6, 1, 5e6, 0, 0, 0, 0, hw_gain, 30, 0, 0, 0, 0)
    else:
        logging.error("ERROR: Select a radio type!")
        sys.exit(1)

    source = msg_source()

    tb = auto_ber_test(samp_rate, tune_offset, source, sink, 0)
    tb.start()

    pkt_data = numpy.random.bytes(255)

    lfr = Radio(port, baud)
    
    cfg = lfr.get_cfg()
    cfg['flags'] &= ~(1) # CRC_ENABLE
    lfr.set_cfg(cfg)


    # Grr... Python scoping...
    global timer
    timer = None

    global num_bits
    global num_errs
    num_bits = 0
    num_errs = 0

    def step_pwr_lvl():
        global num_bits
        global num_errs
        logging.info('')
        logging.info('Power: {: >3.1f} dBm  Bits sent: {: >8}  Bits Errors: {: >5}  log10(BER)= {:.02f}'\
        .format(lvls[0] + gain_offset, num_bits, num_errs, math.log10(num_errs + 0.001) - math.log10(num_bits + 0.001)))
        
        lvls[:] = lvls[1:]
        if not lvls:
            tb.stop()
            thread.interrupt_main()
            # Ugly, skips any finally block
            # Essentially crashes
            os._exit(0)

        logging.info('Stepping to power level: {} dBm'.format(lvls[0] + gain_offset))
        tb.set_sw_gain(lvls[0])

        num_bits = 0
        num_errs = 0

        for _ in range(num_pkts):
            source.transmit(pkt_data)

        global timer
        timer = Timer(num_pkts / 4.0, step_pwr_lvl)
        timer.start()

    timer = Timer(num_pkts / 4.0, step_pwr_lvl)
    
    logging.info('Starting at power level: {} dBm'.format(lvls[0] + gain_offset))
    tb.set_sw_gain(lvls[0])

    for _ in range(num_pkts):
        source.transmit(pkt_data)

    timer.start()

    while True:
        data = [ord(x) for x in lfr.rx()]

        for x, y in zip(data, pkt_data):
            xor = x ^ ord(y) 
            for _ in range(8):
                num_bits += 1
                if (xor & 1): 
                    num_errs += 1
                xor >>= 1


if __name__ == '__main__':
    main()
