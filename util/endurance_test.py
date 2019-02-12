#!/usr/bin/env python3

import sys
import numpy
import time
from com_radio import Radio

radio = Radio(sys.argv[1])
time.sleep(0.5)

large_pkt_delay = 0.250
small_pkt_delay = 0.040

num_iterations = 0

while True:

    num_iterations += 1
    print('Starting stress iteration', num_iterations)

    # Send some NOPs
    for i in range(1000):
        print('Sending NOP {:03d}...'.format(i))
        radio.nop()

    # Send a bunch of big packets
    # Fill the buffer
    for i in range(100):
        print('Sending big packet {:03d}...'.format(i))
        radio.tx('KC2QOL {:03d} '.format(i).ljust(255, 'x'))
    
    time.sleep(large_pkt_delay * 50) # Let about half send

    # Send some more big packets
    # Fill the buffer
    for i in range(50):
        print('Sending big packet {:03d}...'.format(i))
        radio.tx('KC2QOL {:03d} '.format(i).ljust(255, 'x'))

    time.sleep(large_pkt_delay * 100) # Let all send

    # Send a bunch of small packets
    # Fill the buffer
    for i in range(200):
        print('Sending small packet {:03d}...'.format(i))
        radio.tx('KC2QOL {:03d} '.format(i).ljust(25, 'x'))

    time.sleep(small_pkt_delay * 100) # Let about half send

    # Send some more big packets
    # Fill the buffer
    for i in range(50):
        print('Sending big packet {:03d}...'.format(i))
        radio.tx('KC2QOL {:03d} '.format(i).ljust(255, 'x'))

    time.sleep(large_pkt_delay * 100) # Let all send

    # Send a ton of small packets
    for i in range(1000):
        print('Sending small packet {:03d}...'.format(i))
        radio.tx('KC2QOL {:03d} '.format(i).ljust(25, 'x'))

    time.sleep(small_pkt_delay * 1000) # Let all send

    # Wait a while before repeating
    print('Idling for 15 seconds')
    time.sleep(15)
    

