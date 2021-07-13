#!/usr/bin/env python3

import sys
import numpy
import serial
import time
from com_radio import Radio
import vxi11
import greatfet
import ina219

def dbm2mw(dbm):
    return 10**(dbm/10.0)

if __name__ == "__main__":
    
    num_samples = 1
    ext_atten_db = 40.0
    int_atten_db = 10
    freq = 432.5E+6


    drain_voltages = numpy.linspace(5000, 10000, num=11)
    gate_voltages = numpy.linspace(2800, 3300, num=6)

    vbatt = 16.000

    ### END CONFIG ###

    # Connect to LFR on serial port argv[1]
    radio = Radio(sys.argv[1])

    cfg = radio.get_cfg()
    orig_cfg = dict(cfg)

    cfg['freq'] = int(freq)

    # Connect to E4406A. Must be on UBNL-SOF network
    instr = vxi11.Instrument('TCPIP::192.168.100.52::INSTR')

    # Set center freqency
    instr.write(':FREQ:CENT {}'.format(freq))
    # Set channel width
    instr.write(':CHP:BWID:INT 50E+3')
    # Set averaging count
    instr.write(':CHP:AVER:COUN 50')
    # Set display span
    instr.write(':CHP:FREQ:SPAN 200E+3')
    # Set internal attenuation
    instr.write(':POW:ATT {}'.format(int_atten_db))

    # Connect to GreatFET over USB
    gf = greatfet.GreatFET()
    cur_sense = ina219.INA219(gf, 0.1)
    cur_sense.configure()

    # print(cur_sense.current(), 'mA')

    noise_floor, psd = map(float, instr.ask(':READ:CHP?').split(','))
    idle_current = cur_sense.current()

    for drain_mv in drain_voltages:
        for gate_mv in gate_voltages:

            cfg['tx_vdd'] = int(drain_mv)
            cfg['tx_gate_bias'] = int(gate_mv)
            radio.set_cfg(cfg)

            rf_power = numpy.zeros(num_samples)
            amp_power = numpy.zeros(num_samples)

            for i in range(num_samples):
                radio.tx_psr()
                time.sleep(0.100)
                current = cur_sense.current() # mA
                pwr_db, psd_db_hz = map(float, instr.ask(':READ:CHP?').split(','))

                rf_power[i] = pwr_db + ext_atten_db
                amp_power[i] = current * vbatt
                radio.tx_abort()

            med_rf_pwr_dBm = numpy.median(rf_power)
            med_amp_pwr = numpy.median(amp_power)
            med_rf_pwr_mW = dbm2mw(med_rf_pwr_dBm)
            print("{:.0f},{:.0f},{:.1f},{:.0f},{:.0f},{:.1f}".format(drain_mv, gate_mv, med_rf_pwr_dBm, med_rf_pwr_mW, med_amp_pwr, 100*med_rf_pwr_mW/med_amp_pwr))

    radio.set_cfg(orig_cfg)
