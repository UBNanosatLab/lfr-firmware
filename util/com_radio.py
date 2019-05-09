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
    CMD = 3
    PAYLOAD_LEN = 4
    PAYLOAD = 5
    CHKSUM_H = 6
    CHKSUM_L = 7

class Command(Enum):
    NOP = 0x00
    RESET = 0x01

    TXDATA = 0x10
    RXDATA = 0x11
    TX_ABORT = 0x12
    TX_PSR = 0x13

    GET_CFG = 0x20
    SET_CFG = 0x21
    SAVE_CFG = 0x22
    CFG_DEFAULT = 0x23
    SET_FREQ = 0x24
    GET_TXPWR = 0x25
    SET_TXPWR = 0x26

    CMD_GET_QUEUE_DEPTH = 0x32

    INTERNALERROR = 0x7E
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
            self.error = 'ESILICON'
            self.msg = 'Si446x silicon bug (zero len bug?); Try again'
        elif code == 9:
            self.error = 'ERESETSI'
            self.msg = 'Si446x silicon bug; Reset and try again'
        elif code == 11:
            self.error = 'EOVERFLOW'
            self.msg = 'Buffer full'
        elif code == 12:
            self.error = 'EUNDERFLOW'
            self.msg = 'Buffer empty'
        elif code == 19:
            self.error = 'ECMDINVAL'
            self.msg = 'Not a valid command'
        elif code == 22:
            self.error = 'ECMDBADSUM'
            self.msg = 'Bad command checksum'
        elif code == 127:
            self.error = 'ENOTIMPL'
            self.msg = 'Feature not implemented'
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

    def nop_flush(self, num):
        for _ in range(num):
            self.send_pkt(Command.NOP)

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
        (cmd, pay) = self.recv()

        if cmd  == Command.ERROR.value | Command.REPLY.value:
            err = RadioException(pay[0])
            raise err
        elif cmd == Command.RESET.value | Command.REPLY.value:
            return
        else:
            raise Exception('Unexpected response: ' + str((hex(cmd), pay)))

    def nop(self):
        self.send_pkt(Command.NOP)
        (cmd, pay) = self.recv()

        if cmd  == Command.ERROR.value | Command.REPLY.value:
            err = RadioException(pay[0])
            raise err
        elif cmd == Command.NOP.value | Command.REPLY.value:
            return
        else:
            raise Exception('Unexpected response: ' + str((hex(cmd), pay)))

    def tx_psr(self):
        self.send_pkt(Command.TX_PSR)
        (cmd, pay) = self.recv()

        if cmd  == Command.ERROR.value | Command.REPLY.value:
            err = RadioException(pay[0])
            raise err
        elif cmd == Command.TX_PSR.value | Command.REPLY.value:
            return
        else:
            raise Exception('Unexpected response: ' + str((hex(cmd), pay)))

    def tx_abort(self):
        self.send_pkt(Command.TX_ABORT)
        (cmd, pay) = self.recv()

        if cmd  == Command.ERROR.value | Command.REPLY.value:
            err = RadioException(pay[0])
            raise err
        elif cmd == Command.TX_ABORT.value | Command.REPLY.value:
            return
        else:
            raise Exception('Unexpected response: ' + str((hex(cmd), pay)))

    def set_freq(self, freq):
        self.send_pkt(Command.SET_FREQ, struct.pack("!I", freq))
        (cmd, pay) = self.recv()

        if cmd  == Command.ERROR.value | Command.REPLY.value:
            err = RadioException(pay[0])
            raise err
        elif cmd == Command.SET_FREQ.value | Command.REPLY.value:
            return
        else:
            raise Exception('Unexpected response: ' + str((hex(cmd), pay)))

    def set_txpwr(self, pwr):
        self.send_pkt(Command.SET_TXPWR, struct.pack("!H", pwr))
        (cmd, pay) = self.recv()

        if cmd  == Command.ERROR.value | Command.REPLY.value:
            err = RadioException(pay[0])
            raise err
        elif cmd == Command.SET_TXPWR.value | Command.REPLY.value:
            return
        else:
            raise Exception('Unexpected response: ' + str((hex(cmd), pay)))

    def tx(self, data):
        self.send_pkt(Command.TXDATA, data if isinstance(data, bytes) else data.encode('utf-8'))
        (cmd, pay) = self.recv()

        if cmd  == Command.ERROR.value | Command.REPLY.value:
            err = RadioException(pay[0])
            if err.error == 'EOVERFLOW':
                sleep(0.05)
                self.tx(data)
            else:
                raise err

        elif cmd == Command.TXDATA.value | Command.REPLY.value:
            return
        else:
            raise Exception('Unexpected response: ' + str((hex(cmd), pay)))

    def rx(self):
        (cmd, pay) = self.recv()

        if cmd  == Command.ERROR.value | Command.REPLY.value:
            raise RadioException(pay[0])
        elif cmd == Command.RXDATA.value | Command.REPLY.value:
            return pay.decode("utf-8")
        else:
            raise Exception('Unexpected response: ' + str((hex(cmd), pay)))

    def recv(self):

        payload = b''

        while True:
            c = self.ser.read(1)[0]

            if self.state is ParseState.SYNC_H:
                if c == SYNCWORD_H:
                    self.state = ParseState.SYNC_L
            elif self.state is ParseState.SYNC_L:
                if c == SYNCWORD_L:
                    self.state = ParseState.CMD
                elif c == SYNCWORD_H:
                    self.state = ParseState.SYNC_L
                else:
                    self.state = ParseState.SYNC_H
            elif self.state is ParseState.CMD:
                cmd = c
                self.state = ParseState.PAYLOAD_LEN
            elif self.state is ParseState.PAYLOAD_LEN:
                length = c
                # TODO: Validate len for cmd
                if (length):
                    self.state = ParseState.PAYLOAD
                else:
                    chksum = self.checksum(bytes([cmd, 0]))
                    self.state = ParseState.CHKSUM_H

            elif self.state is ParseState.PAYLOAD:
                payload += bytes([c])
                length -= 1
                self.state = ParseState.PAYLOAD
                if (length == 0):
                    chksum = self.checksum(bytes([cmd, len(payload)]) + payload)
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


        return (cmd, payload)

    def get_cfg(self):
        self.send_pkt(Command.GET_CFG)

        (cmd, pay) = self.recv()

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
            raise Exception('Unexpected response: ' + str((hex(cmd), pay)))

    def set_cfg(self, cfg_in):

        cfg = dict(cfg_in)

        cfg['callsign'] = cfg['callsign'].encode("utf-8")

        data = struct.pack("!BIBHHHHHH8s", \
            cfg['cfg_ver'], cfg['freq'], cfg['modem_config'], \
            cfg['txco_vpull'], cfg['tx_gate_bias'], cfg['tx_vdd'], \
            cfg['pa_ilimit'], cfg['tx_vdd_delay'], cfg['flags'], \
            cfg['callsign'])

        self.send_pkt(Command.SET_CFG, data)

        (cmd, pay) = self.recv()

        if cmd  == Command.ERROR.value | Command.REPLY.value:
            err = RadioException(pay[0])
            raise err

        elif cmd == Command.SET_CFG.value | Command.REPLY.value:
            return
        else:
            raise Exception('Unexpected response: ' + str((hex(cmd), pay)))


    def save_cfg(self):
        self.send_pkt(Command.SAVE_CFG)

        (cmd, pay) = self.recv()

        if cmd  == Command.ERROR.value | Command.REPLY.value:
            err = RadioException(pay[0])
            raise err

        elif cmd == Command.GET_CFG.value | Command.REPLY.value:
            return
        else:
            raise Exception('Unexpected response: ' + str((hex(cmd), pay)))

    def cfg_default(self):
        self.send_pkt(Command.CFG_DEFAULT)

        (cmd, pay) = self.recv()

        if cmd  == Command.ERROR.value | Command.REPLY.value:
            err = RadioException(pay[0])
            raise err

        elif cmd == Command.GET_CFG.value | Command.REPLY.value:
            return
        else:
            raise Exception('Unexpected response: ' + str((hex(cmd), pay)))

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

    elif (sys.argv[2] == 'tx-psr' and len(sys.argv) == 3):
        radio.tx_psr()

    elif (sys.argv[2] == 'tx-abort' and len(sys.argv) == 3):
        radio.tx_abort()

    elif (sys.argv[2] == 'reset' and len(sys.argv) == 3):
        radio.nop_flush(60)
        radio.flush_serial()
        radio.reset()

    else:
        print('Usage: python3', sys.argv[0], '/dev/<RADIO_UART> [rx | tx n | get-cfg | load-cfg | save-cfg | default-cfg | tx-psr | tx-abort | reset]')
