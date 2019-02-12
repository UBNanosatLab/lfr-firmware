#!/usr/bin/env python3

import sys
import numpy
import time
from com_radio import Radio

if __name__ == "__main__":

    radio = Radio(sys.argv[1])    
    num_pkts = int(sys.argv[2])
    delay_s = int(sys.argv[3]) / 1000.0

    radio.reset()
    time.sleep(0.5)

    for i in range(num_pkts):
        print('Sending packet {:03d}...'.format(i))
        radio.tx('TEST!' * 25)
        time.sleep(delay_s)
    

