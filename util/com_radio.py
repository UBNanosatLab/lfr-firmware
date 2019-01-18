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

def create_tx_pkt(data):
    pkt = '\x02'
    pkt += chr(len(data))
    pkt += data
    chksum = compute_chksum(pkt)
    pkt += chr(chksum >> 8)
    pkt += chr(chksum & 0xFF)
    pkt = '\xBE\xEF' + pkt
    return pkt

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


    def tx(self, data):
        pkt = chr(Command.TXDATA.value)
        pkt += chr(len(data))
        pkt += data
        chksum = compute_chksum(pkt)
        pkt += chr(chksum >> 8)
        pkt += chr(chksum & 0xFF)
        pkt = '\xBE\xEF' + pkt

        self.ser.write(bytes([ord(x) for x in pkt]))

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
            raise RadioException(ord(pay[0]))
        elif Command.RXDATA.value | Command.REPLY.value:
            return pay
        else:
            raise Exception('Unexpected response: ' + str((flags, cmd, pay)))

    def recv(self):

        payload = ''

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
                    chksum = compute_chksum(''.join([chr(flags), chr(cmd), chr(0)]))
                    self.state = ParseState.CHKSUM_H

            elif self.state is ParseState.PAYLOAD:
                payload += chr(c)
                length -= 1
                self.state = ParseState.PAYLOAD
                if (length == 0):
                    chksum = compute_chksum(''.join([chr(flags), chr(cmd), chr(len(payload))]) + payload)
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
        pkt = chr(Command.GET_CFG.value)
        pkt += chr(0)
        chksum = compute_chksum(pkt)
        pkt += chr(chksum >> 8)
        pkt += chr(chksum & 0xFF)
        pkt = '\xBE\xEF' + pkt

        self.ser.write(bytes([ord(x) for x in pkt]))

        (flags, cmd, pay) = self.recv()

        if cmd  == Command.ERROR.value | Command.REPLY.value:
            err = RadioException(pay[0])
            raise err

        elif Command.GET_CFG.value | Command.REPLY.value:

            cfg = {}

            cfg['cfg_ver'], cfg['freq'], cfg['deviation'], cfg['data_rate'], \
            cfg['txco_vpull'], cfg['tx_gate_bias'], cfg['tx_vdd'], \
            cfg['pa_ilimit'], cfg['tx_vdd_delay'], cfg['flags'],\
            cfg['callsign'] = struct.unpack("!BIHHHHHHHH8s", bytes([ord(x) for x in pay]))

            cfg['callsign'] = cfg['callsign'].decode("utf-8")

            return cfg
        else:
            raise Exception('Unexpected response: ' + str((flags, cmd, pay)))

    def set_cfg(self, cfg_in):

        cfg = dict(cfg_in)

        cfg['callsign'] = cfg['callsign'].encode("utf-8")

        data = struct.pack("!BIHHHHHHHH8s", \
            cfg['cfg_ver'], cfg['freq'], cfg['deviation'], cfg['data_rate'], \
            cfg['txco_vpull'], cfg['tx_gate_bias'], cfg['tx_vdd'], \
            cfg['pa_ilimit'], cfg['tx_vdd_delay'], cfg['flags'], \
            cfg['callsign'])

        pkt = chr(Command.SET_CFG.value)
        pkt += chr(len(data))
        pkt += ''.join([chr(x) for x in data])
        chksum = compute_chksum(pkt)
        pkt += chr(chksum >> 8)
        pkt += chr(chksum & 0xFF)
        pkt = '\xBE\xEF' + pkt

        self.ser.write(bytes([ord(x) for x in pkt]))

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

        self.ser.write(bytes([ord(x) for x in pkt]))

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

        self.ser.write(bytes([ord(x) for x in pkt]))

        (flags, cmd, pay) = self.recv()

        if cmd  == Command.ERROR.value | Command.REPLY.value:
            err = RadioException(ord(pay[0]))
            raise err

        elif Command.GET_CFG.value | Command.REPLY.value:
            return
        else:
            raise Exception('Unexpected response: ' + str((flags, cmd, pay)))

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
        print('Usage: python3', sys.argv[0], '/dev/<RADIO_UART> [rx | tx n]')
