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
from com_radio_2 import Radio

class auto_ber_test(gr.top_block):

    def __init__(self, samp_rate, tune_offset, source, sink, sw_gain=0, sym_rate=10e3, deviation=20e3):
        gr.top_block.__init__(self, "Auto BER Test")

        self.source = source
        self.sink = sink

        ##################################################
        # Variables
        ##################################################
        self.sym_rate = sym_rate
        self.samp_per_sym = samp_per_sym = int(samp_rate / sym_rate)
        self.bt = bt = 0.5
        self.gaussian_taps = gaussian_taps = firdes.gaussian(1.0,samp_per_sym, bt, 4 * samp_per_sym)
        self.tx_taps = tx_taps = numpy.convolve(numpy.array(gaussian_taps),numpy.array((1,) * samp_per_sym))
        self.tx_deviation = tx_deviation = deviation
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
#                  Main                #
########################################

def main():
    ########################################
    #              Configuration           #
    ########################################

    port = "/dev/tty.SLAB_USBtoUART"
    baud = 115200

    freq = 434.000e6
    tune_offset = 250e3
    samp_rate = 4e6
    sym_rate = 40e3
    deviation = 10e3

    usrp = True
    lime = False
    lime_serial = '1D3AC9891D2233'

    num_pkts = 100

    hw_gain = 25
    gain_offset = -100

    lvls = numpy.linspace(-7.0, -0.0, 8).tolist()

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

    tb = auto_ber_test(samp_rate, tune_offset, source, sink, 0, sym_rate=sym_rate, deviation=deviation)
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

    pkt_delay = 2500 / sym_rate

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
        timer = Timer(num_pkts * pkt_delay, step_pwr_lvl)
        timer.start()

    timer = Timer(num_pkts * pkt_delay, step_pwr_lvl)
    
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
