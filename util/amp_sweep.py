#!/usr/bin/env python3.7

import sys
import numpy
import serial
import time
from com_radio import Radio

def ad8318_mv2dbm(mv):
    return (mv / -24.5) + 25.0

def ina169_mv2ma(mv):
    # 1 V per A
    return mv

def dbm2mw(dbm):
    return 10**(dbm/10.0)

if __name__ == "__main__":
    
    num_samples = 5
    atten_db = 40.0

    drain_voltages = numpy.linspace(5000, 10000, num=11)
    gate_voltages = numpy.linspace(2800, 3300, num=6)


    radio = Radio(sys.argv[1])
    arduino = serial.Serial(sys.argv[2], 115200)

    cfg = radio.get_cfg()
    orig_cfg = dict(cfg)

    for drain_mv in drain_voltages:
        for gate_mv in gate_voltages:

            cfg['tx_vdd'] = int(drain_mv)
            cfg['tx_gate_bias'] = int(gate_mv)
            radio.set_cfg(cfg)

            rf_power = numpy.zeros(num_samples)
            amp_power = numpy.zeros(num_samples)

            for i in range(num_samples):
                radio.tx('TEST!' * 25)
                time.sleep(0.050)
                arduino.write(b'r')
                fields = arduino.readline().decode('utf-8').split(',')
                rf_power[i] = ad8318_mv2dbm(int(fields[0])) + atten_db
                amp_power[i] = ina169_mv2ma(int(fields[2]))
                time.sleep(0.100)

            print("{:.0f},{:.0f},{:.1f},{:.0f}".format(drain_mv, gate_mv, numpy.median(rf_power), numpy.median(amp_power)))

    radio.set_cfg(orig_cfg)

















# // Arduino (Teensy) Code
# void setup()
# {                
#   Serial.begin(115200);
#   analogReadResolution(16);
# }
#
# void loop()                     
# {
#   if (Serial.available() > 0) {
#     char c = Serial.read();
#
#     if (c == 'r') {
#       int ad8318_mv = map(analogRead(0), 0, 65535, 0, 3300);
#       int ina169_mv = map(analogRead(1), 0, 65535, 0, 3300); 
#       Serial.print(ad8318_mv);
#       Serial.print(",mV,");
#       Serial.print(ina169_mv);
#       Serial.println(",mV");
#     }
#   }
# }