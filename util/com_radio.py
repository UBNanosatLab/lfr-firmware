#!/usr/bin/env python3

import sys
import socket
from time import sleep
import threading

import serial
import struct
import json

from enum import Enum, auto

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
    SET_FREQ = 0x05

    GET_CFG = 0x08
    SET_CFG = 0x09
    SAVE_CFG = 0x0A
    CFG_DEFAULT = 0x0B

    RXDATA = 0x10

    ERROR = 0x7F

    REPLY = 0x80

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
            self.msg = 'An unknown error occurred (' + str(code) + ')'

    def __str__(self):
        return 'RadioException(' + self.error + '): ' + self.msg

    pass


class Radio:

    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baud)
        self.state = ParseState.SYNC_H

    def flush_serial(self):
        self.ser.reset_input_buffer()

    def checksum(self, data):
        def feltcher(chksum, byte):
            lsb = chksum & 0xFF
            msb = chksum >> 8
            msb += byte
            msb &= 0xFF
            lsb += msb
            lsb &= 0xFF
            return (msb << 8) | lsb

        chksum = 0
        for x in data:
            chksum = feltcher(chksum, x)
        return chksum

    def send_pkt(self, cmd, data=bytes()):
    
        pkt = bytes([cmd.value, len(data)])
        pkt += data
        chksum = self.checksum(pkt)
        pkt += bytes([chksum >> 8, chksum & 0xFF])
        pkt = b'\xBE\xEF' + pkt

        self.ser.write(pkt)

    def reset(self):
        self.send_pkt(Command.RESET)
        (flags, cmd, pay) = self.recv()

        if cmd  == Command.ERROR.value | Command.REPLY.value:
            err = RadioException(pay[0])
            raise err
        elif Command.RESET.value | Command.REPLY.value:
            return
        else:
            raise Exception('Unexpected response: ' + str((hex(flags), hex(cmd), pay)))

    def nop(self):
        self.send_pkt(Command.NOP)
        (flags, cmd, pay) = self.recv()

        if cmd  == Command.ERROR.value | Command.REPLY.value:
            err = RadioException(pay[0])
            raise err
        elif Command.NOP.value | Command.REPLY.value:
            return
        else:
            raise Exception('Unexpected response: ' + str((hex(flags), hex(cmd), pay)))

    def set_freq(self, freq):
        self.send_pkt(Command.SET_FREQ, struct.pack("!I", freq))
        (flags, cmd, pay) = self.recv()

        if cmd  == Command.ERROR.value | Command.REPLY.value:
            err = RadioException(pay[0])
            raise err
        elif Command.SET_FREQ.value | Command.REPLY.value:
            return
        else:
            raise Exception('Unexpected response: ' + str((hex(flags), hex(cmd), pay)))

    def set_txpwr(self, pwr):
        self.send_pkt(Command.SET_TXPWR, struct.pack("!H", pwr))
        (flags, cmd, pay) = self.recv()

        if cmd  == Command.ERROR.value | Command.REPLY.value:
            err = RadioException(pay[0])
            raise err
        elif Command.SET_TXPWR.value | Command.REPLY.value:
            return
        else:
            raise Exception('Unexpected response: ' + str((hex(flags), hex(cmd), pay)))

    def tx(self, data):
        self.send_pkt(Command.TXDATA, data.encode('utf-8'))
        (flags, cmd, pay) = self.recv()

        if cmd  == Command.ERROR.value | Command.REPLY.value:
            err = RadioException(pay[0])
            if err.error == 'EBUSY':
                self.tx(data)
            else:
                raise err

        elif Command.TXDATA.value | Command.REPLY.value:
            return
        else:
            raise Exception('Unexpected response: ' + str((hex(flags), hex(cmd), pay)))

    def rx(self):
        (flags, cmd, pay) = self.recv()

        if cmd  == Command.ERROR.value | Command.REPLY.value:
            raise RadioException(pay[0])
        elif Command.RXDATA.value | Command.REPLY.value:
            return pay.decode("utf-8")
        else:
            raise Exception('Unexpected response: ' + str((hex(flags), hex(cmd), pay)))

    def recv(self):

        payload = b''

        while True:
            c = self.ser.read(1)[0]

            if self.state is ParseState.SYNC_H:
                if c == SYNCWORD_H:
                    self.state = ParseState.SYNC_L
            elif self.state is ParseState.SYNC_L:
                if c == SYNCWORD_L:
                    self.state = ParseState.FLAGS
                elif c == SYNCWORD_H:
                    self.state = ParseState.SYNC_L
                else:
                    self.state = ParseState.SYNC_H
            elif self.state is ParseState.FLAGS:
                flags = c
                self.state = ParseState.CMD
            elif self.state is ParseState.CMD:
                cmd = c
                self.state = ParseState.PAYLOAD_LEN
            elif self.state is ParseState.PAYLOAD_LEN:
                length = c
                # TODO: Validate len for cmd
                if (length):
                    self.state = ParseState.PAYLOAD
                else:
                    chksum = self.checksum(bytes([flags , cmd, 0]))
                    self.state = ParseState.CHKSUM_H

            elif self.state is ParseState.PAYLOAD:
                payload += bytes([c])
                length -= 1
                self.state = ParseState.PAYLOAD
                if (length == 0):
                    chksum = self.checksum(bytes([flags , cmd, len(payload)]) + payload)
                    self.state = ParseState.CHKSUM_H
            elif self.state is ParseState.CHKSUM_H:
                if (c == chksum >> 8):
                    self.state = ParseState.CHKSUM_L
                else:
                    # TODO: Handle error
                    pass
                    self.state = ParseState.SYNC_H
                    break
            elif self.state is ParseState.CHKSUM_L:
                if (c != chksum & 0xFF):
                    # TODO: Handle error
                    pass
                self.state = ParseState.SYNC_H
                break


        return (flags, cmd, payload)

    def get_cfg(self):
        self.send_pkt(Command.GET_CFG)

        (flags, cmd, pay) = self.recv()

        if cmd  == (Command.ERROR.value | Command.REPLY.value):
            err = RadioException(pay[0])
            raise err

        elif cmd == (Command.GET_CFG.value | Command.REPLY.value):

            cfg = {}
            cfg['cfg_ver'], cfg['freq'], cfg['modem_config'], \
            cfg['txco_vpull'], cfg['tx_gate_bias'], cfg['tx_vdd'], \
            cfg['pa_ilimit'], cfg['tx_vdd_delay'], cfg['flags'],\
            cfg['callsign'] = struct.unpack("!BIBHHHHHH8s", pay)

            cfg['callsign'] = cfg['callsign'].decode("utf-8")

            return cfg
        else:
            raise Exception('Unexpected response: ' + str((hex(flags), hex(cmd), pay)))

    def set_cfg(self, cfg_in):

        cfg = dict(cfg_in)

        cfg['callsign'] = cfg['callsign'].encode("utf-8")

        data = struct.pack("!BIBHHHHHH8s", \
            cfg['cfg_ver'], cfg['freq'], cfg['modem_config'], \
            cfg['txco_vpull'], cfg['tx_gate_bias'], cfg['tx_vdd'], \
            cfg['pa_ilimit'], cfg['tx_vdd_delay'], cfg['flags'], \
            cfg['callsign'])

        self.send_pkt(Command.SET_CFG, data)

        (flags, cmd, pay) = self.recv()

        if cmd  == Command.ERROR.value | Command.REPLY.value:
            err = RadioException(pay[0])
            raise err

        elif Command.SET_CFG.value | Command.REPLY.value:
            return
        else:
            raise Exception('Unexpected response: ' + str((hex(flags), hex(cmd), pay)))


    def save_cfg(self):
        self.send_pkt(Command.SAVE_CFG)

        (flags, cmd, pay) = self.recv()

        if cmd  == Command.ERROR.value | Command.REPLY.value:
            err = RadioException(pay[0])
            raise err

        elif Command.GET_CFG.value | Command.REPLY.value:
            return
        else:
            raise Exception('Unexpected response: ' + str((hex(flags), hex(cmd), pay)))

    def cfg_default(self):
        self.send_pkt(Command.CFG_DEFAULT)

        (flags, cmd, pay) = self.recv()

        if cmd  == Command.ERROR.value | Command.REPLY.value:
            err = RadioException(pay[0])
            raise err

        elif Command.GET_CFG.value | Command.REPLY.value:
            return
        else:
            raise Exception('Unexpected response: ' + str((hex(flags), hex(cmd), pay)))

if __name__ == '__main__':
    radio = Radio(sys.argv[1])

    if (len(sys.argv) == 2 or sys.argv[2] == 'rx'):
        numPackets = 0
        while True:
            pkt = radio.rx()
            print('RX>', pkt)
            numPackets += 1
            print('Received ' + str(numPackets) + ' packet(s)')

    elif (sys.argv[2] == 'tx' and len(sys.argv) == 4):
        sleep(1)
        for i in range(int(sys.argv[3])):
            data = ('KC2QOL ' + str(i + 1) + ' ').ljust(255, 'x')
            print('TX>', data)
            radio.tx(data)
            print('Sent ' + str(len(data)) + ' byte(s)')
            # Look ma, no sleep!
    
    elif (sys.argv[2] == 'get-cfg' and len(sys.argv) == 3):
        print(json.dumps(radio.get_cfg(), indent=4))

    elif (sys.argv[2] == 'load-cfg' and len(sys.argv) == 4): 
        with open(sys.argv[3], 'r') as f:
            cfg = json.load(f)
        
        radio.set_cfg(cfg)
        new_cfg = json.dumps(radio.get_cfg(), indent=4)
        print("Successfully loaded config:", sys.argv[3])
        print(new_cfg)
     
    elif (sys.argv[2] == 'save-cfg' and len(sys.argv) == 3):
        radio.save_cfg()

    elif (sys.argv[2] == 'default-cfg' and len(sys.argv) == 3):
        radio.cfg_default()

    else:
        print('Usage: python3', sys.argv[0], '/dev/<RADIO_UART> [rx | tx n | get-cfg | load-cfg | svae-cfg | default-cfg]')
