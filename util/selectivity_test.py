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
from gnuradio import digital
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
import si446x
from optparse import OptionParser
import math
import numpy
import time
import sys
import pmt
import logging
import kiss
from threading import Timer 
import thread
import os
import json


from com_radio_2 import Radio

class auto_ber_test(gr.top_block):

    def __init__(self, samp_rate, source, sink, sw_gain=0, interfering_gain=-100, interfering_offset=50e3):
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


        self.interfering_offset = interfering_offset
        self.interfering_gain = interfering_gain

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
        self.freq_mod = analog.frequency_modulator_fc(2 * math.pi * tx_deviation / samp_rate)

        self.interfering_src = analog.sig_source_c(samp_rate, analog.GR_COS_WAVE, interfering_offset, 10**(interfering_gain / 20.0), 0)
        self.multiply_sw_gain = blocks.multiply_const_vcc((10**(sw_gain / 20.0), ))
        self.blocks_add = blocks.add_vcc(1)


        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.var_len_packet_creator, 'out'), (self.pdu_to_tagged_stream, 'pdus'))    
        self.msg_connect((self.source, 'out'), (self.var_len_packet_creator, 'in'))    
        self.connect((self.blocks_add, 0), (self.sink, 0))
        self.connect((self.freq_mod, 0), (self.multiply_sw_gain, 0))
        self.connect((self.gaussian_filter, 0), (self.freq_mod, 0))
        self.connect((self.interfering_src, 0), (self.blocks_add, 1))
        self.connect((self.mult_two, 0), (self.gaussian_filter, 0))
        self.connect((self.multiply_sw_gain, 0), (self.blocks_add, 0))
        self.connect((self.pdu_to_tagged_stream, 0), (self.uchar_to_float, 0))
        self.connect((self.sub_one_half, 0), (self.mult_two, 0))
        self.connect((self.uchar_to_float, 0), (self.sub_one_half, 0))

    def get_sw_gain(self):
        return self.sw_gain

    def set_sw_gain(self, sw_gain):
        self.sw_gain = sw_gain
        self.multiply_sw_gain.set_k((10**(self.sw_gain / 20.0), ))

    def get_interfering_offset(self):
        return self.interfering_offset

    def set_interfering_offset(self, interfering_offset):
        self.interfering_offset = interfering_offset
        self.interfering_src.set_frequency(self.interfering_offset)

    def get_interfering_gain(self):
        return self.interfering_gain

    def set_interfering_gain(self, interfering_gain):
        self.interfering_gain = interfering_gain
        self.interfering_src.set_amplitude(10**(self.interfering_gain / 20.0))


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
    lfr_freq = 434.000e6
    tune_offset = 10e6
    samp_rate = 1e6 * 2

    usrp = True

    num_pkts = 20
    ms_per_packet = 250

    # hw_gain = 20
    # gain_offset = -46 - 60

    hw_gain = 68
    gain_offset = -67 - 60 + hw_gain

    interfering_offset = 75e3
    # Can't be larger than sw_gain
    interfering_rel_gains = numpy.linspace(0, 50, 6).tolist()

    sw_gain = -50

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
        sink.set_center_freq(uhd.tune_request(freq, tune_offset), 0)
        sink.set_gain(hw_gain, 0)
        sink.set_antenna('TX/RX', 0)
    else:
        logging.error("ERROR: Select a radio type!")
        sys.exit(1)

    source = msg_source()

    tb = auto_ber_test(samp_rate, source, sink, sw_gain, -100, interfering_offset)
    tb.start()

    pkt_data = numpy.random.bytes(255)

    lfr = Radio(port, baud)
    
    cfg = lfr.get_cfg()
    cfg['freq'] = int(lfr_freq)
    cfg['flags'] &= ~(1) # ~CRC_ENABLE
    lfr.set_cfg(cfg)
    print(json.dumps(lfr.get_cfg(), indent=4))

    # Grr... Python scoping...
    global timer
    timer = None

    global num_bits
    global num_errs
    num_bits = 0
    num_errs = 0

    def step():
        global num_bits
        global num_errs
        logging.info('')
        logging.info('Interfering Power: {: >3.1f} dBc  Bits sent: {: >8}  Bits Errors: {: >5}  log10(BER)= {:.02f}'\
        .format(interfering_rel_gains[0], num_bits, num_errs, math.log10(num_errs + 0.001) - math.log10(num_bits + 0.001)))
        
        interfering_rel_gains[:] = interfering_rel_gains[1:]
        if not interfering_rel_gains:
            tb.stop()
            thread.interrupt_main()
            # Ugly, skips any finally block
            # Essentially crashes
            os._exit(0)

        logging.info('Stepping to power level: {} dBc'.format(interfering_rel_gains[0]))
        tb.set_interfering_gain(interfering_rel_gains[0] + sw_gain)

        num_bits = 0
        num_errs = 0

        for _ in range(num_pkts):
            source.transmit(pkt_data)

        global timer
        timer = Timer(num_pkts * ms_per_packet / 1000.0, step)
        timer.start()

    timer = Timer(num_pkts * ms_per_packet / 1000.0, step)
    
    logging.info('Starting at power level: {} dBc'.format(interfering_rel_gains[0]))
    tb.set_interfering_gain(interfering_rel_gains[0] + sw_gain)

    for _ in range(num_pkts):
        source.transmit(pkt_data)

    timer.start()

    while True:
        data = [ord(x) for x in lfr.rx()]

        if len(data) == len(pkt_data):
            for x, y in zip(data, pkt_data):
                xor = x ^ ord(y) 
                for _ in range(8):
                    num_bits += 1
                    if (xor & 1): 
                        num_errs += 1
                    xor >>= 1


if __name__ == '__main__':
    main()
