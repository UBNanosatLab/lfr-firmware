#!/usr/bin/env python3

from com_radio import Radio
import unittest
import sys
import time
import timeout_decorator
import random

class Communication_Tests(unittest.TestCase):

    def setUp(self):
        self.radio_uart = sys.argv[1]
        self.radio = Radio(self.radio_uart)
        self.radio.flush_serial()
        pass
    
    @timeout_decorator.timeout(2)
    def tearDown(self):
        self.radio.cfg_default()
        self.radio.save_cfg()
        self.radio.reset()

    @timeout_decorator.timeout(5)
    def test_reset(self):
        self.radio.reset()

    @timeout_decorator.timeout(3)
    def test_nop(self):
        self.radio.nop()

    @timeout_decorator.timeout(3)
    def test_get_cfg(self):
        self.radio.get_cfg()

    @timeout_decorator.timeout(3)
    def test_set_cfg(self):
        test_callsign = 'TESTER{:02d}'.format(random.randrange(100))
        cfg = self.radio.get_cfg()
        cfg['callsign'] = test_callsign
        self.radio.set_cfg(cfg)
        new_cfg = self.radio.get_cfg()
        self.assertEqual(cfg, new_cfg)

    @timeout_decorator.timeout(3)
    def test_default_cfg(self):
        test_callsign = 'TESTER{:02d}'.format(random.randrange(100))
        cfg = self.radio.get_cfg()
        cfg['callsign'] = test_callsign
        self.radio.set_cfg(cfg)
        self.radio.cfg_default()
        new_cfg = self.radio.get_cfg()
        self.assertNotEqual(cfg, new_cfg)

    @timeout_decorator.timeout(3)
    def test_save_cfg(self):
        test_callsign = 'TESTER{:02d}'.format(random.randrange(100))
        cfg = self.radio.get_cfg()
        cfg['callsign'] = test_callsign
        
        self.radio.set_cfg(cfg)
        self.radio.reset()
        new_cfg = self.radio.get_cfg()
        self.assertNotEqual(cfg, new_cfg)

        self.radio.set_cfg(cfg)
        self.radio.save_cfg()
        self.radio.reset()
        new_cfg = self.radio.get_cfg()
        self.assertEqual(cfg, new_cfg)

    @timeout_decorator.timeout(3)
    def test_set_freq(self):
        test_freq = random.randrange(430000000, 440000000)
        self.radio.set_freq(test_freq)
        cfg = self.radio.get_cfg()
        self.assertEqual(cfg['freq'], test_freq)

    @timeout_decorator.timeout(30)
    def test_set_txpwr(self):
        test_pwr = random.randrange(0x001, 0xFFF)
        self.radio.set_txpwr(test_pwr)
        cfg = self.radio.get_cfg()
        self.assertEqual(cfg['tx_gate_bias'], test_pwr)



    
def suite(tests=None):
    suite = unittest.TestSuite()
    if tests:
        for test in tests:
            suite.addTest(Communication_Tests(test))
    else:
        suite.addTest(unittest.defaultTestLoader.loadTestsFromTestCase(Communication_Tests))
    
    return suite

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: ' + sys.argv[0] + '/dev/<RADIO_UART>')
        sys.exit(1)

    tests = None

    if len(sys.argv) > 2:
        tests = sys.argv[2:]

    runner = unittest.TextTestRunner()
    runner.run(suite(tests))
