import serial
import struct
from enum import Enum

########################################
#         LFR Command Handling         #
########################################

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
    GET_TXPWR = 0x25
    SET_TXPWR = 0x26

    GET_CFG = 0x20
    SET_CFG = 0x21
    SAVE_CFG = 0x22
    CFG_DEFAULT = 0x23

    RXDATA = 0x11

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
            self.msg = 'An unknown error occurred'

    def __str__(self):
        return 'RadioException(' + self.error + '): ' + self.msg

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

        self.ser.write(pkt)

        (cmd, pay) = self.recv()

        if cmd  == Command.ERROR.value | Command.REPLY.value:
            err = RadioException(ord(pay[0]))
            if err.error == 'EBUSY':
                self.tx(data)
            else:
                raise err

        elif Command.TXDATA.value | Command.REPLY.value:
            return
        else:
            raise Exception('Unexpected response: ' + str((cmd, pay)))

    def rx(self):
        (cmd, pay) = self.recv()

        if cmd  == Command.ERROR.value | Command.REPLY.value:
            raise RadioException(pay[0])
        elif Command.RXDATA.value | Command.REPLY.value:
            return pay
        else:
            raise Exception('Unexpected response: ' + str((cmd, pay)))

    def recv(self):

        payload = ''

        while True:
            c = ord(self.ser.read(1))

            if self.state == ParseState.SYNC_H:
                if c == SYNCWORD_H:
                    self.state = ParseState.SYNC_L
            elif self.state == ParseState.SYNC_L:
                if c == SYNCWORD_L:
                    self.state = ParseState.CMD
                elif c == SYNCWORD_H:
                    self.state = ParseState.SYNC_L
                else:
                    self.state = ParseState.SYNC_H
            elif self.state == ParseState.CMD:
                cmd = c
                self.state = ParseState.PAYLOAD_LEN
            elif self.state == ParseState.PAYLOAD_LEN:
                length = c
                # TODO: Validate len for cmd
                if (length):
                    self.state = ParseState.PAYLOAD
                else:
                    chksum = compute_chksum(''.join([chr(cmd), chr(0)]))
                    self.state = ParseState.CHKSUM_H

            elif self.state == ParseState.PAYLOAD:
                payload += chr(c)
                length -= 1
                self.state = ParseState.PAYLOAD
                if (length == 0):
                    chksum = compute_chksum(''.join([chr(cmd), chr(len(payload))]) + payload)
                    self.state = ParseState.CHKSUM_H
            elif self.state == ParseState.CHKSUM_H:
                if (c == chksum >> 8):
                    self.state = ParseState.CHKSUM_L
                else:
                    # TODO: Handle error
                    pass
                    self.state = ParseState.SYNC_H
                    break
            elif self.state == ParseState.CHKSUM_L:
                if (c != chksum & 0xFF):
                    # TODO: Handle error
                    pass
                self.state = ParseState.SYNC_H
                break

        return (0, cmd, payload)

    def get_cfg(self):
        pkt = chr(Command.GET_CFG.value)
        pkt += chr(0)
        chksum = compute_chksum(pkt)
        pkt += chr(chksum >> 8)
        pkt += chr(chksum & 0xFF)
        pkt = '\xBE\xEF' + pkt

        self.ser.write(pkt)

        (cmd, pay) = self.recv()

        if cmd  == Command.ERROR.value | Command.REPLY.value:
            err = RadioException(pay[0])
            raise err

        elif Command.GET_CFG.value | Command.REPLY.value:

            cfg = {}

            cfg['cfg_ver'], cfg['freq'], cfg['modem_config'], \
            cfg['txco_vpull'], cfg['tx_gate_bias'], cfg['tx_vdd'], \
            cfg['pa_ilimit'], cfg['tx_vdd_delay'], cfg['flags'],\
            cfg['callsign'] = struct.unpack("!BIBHHHHHH8s", pay)

            return cfg
        else:
            raise Exception('Unexpected response: ' + str((cmd, pay)))

    def set_cfg(self, cfg_in):

        cfg = dict(cfg_in)

        data = struct.pack("!BIBHHHHHH8s", \
            cfg['cfg_ver'], cfg['freq'], cfg['modem_config'], \
            cfg['txco_vpull'], cfg['tx_gate_bias'], cfg['tx_vdd'], \
            cfg['pa_ilimit'], cfg['tx_vdd_delay'], cfg['flags'], \
            cfg['callsign'])

        pkt = chr(Command.SET_CFG.value)
        pkt += chr(len(data))
        pkt += data
        chksum = compute_chksum(pkt)
        pkt += chr(chksum >> 8)
        pkt += chr(chksum & 0xFF)
        pkt = '\xBE\xEF' + pkt

        self.ser.write(pkt)

        (cmd, pay) = self.recv()

        if cmd  == Command.ERROR.value | Command.REPLY.value:
            err = RadioException(ord(pay[0]))
            raise err

        elif Command.SET_CFG.value | Command.REPLY.value:
            return
        else:
            raise Exception('Unexpected response: ' + str((cmd, pay)))


    def save_cfg(self):
        pkt = chr(Command.SAVE_CFG.value)
        pkt += chr(0)
        chksum = compute_chksum(pkt)
        pkt += chr(chksum >> 8)
        pkt += chr(chksum & 0xFF)
        pkt = '\xBE\xEF' + pkt

        self.ser.write(pkt)

        (cmd, pay) = self.recv()

        if cmd  == Command.ERROR.value | Command.REPLY.value:
            err = RadioException(ord(pay[0]))
            raise err

        elif Command.GET_CFG.value | Command.REPLY.value:
            return
        else:
            raise Exception('Unexpected response: ' + str((cmd, pay)))

    def cfg_default(self):
        pkt = chr(Command.CFG_DEFAULT.value)
        pkt += chr(0)
        chksum = compute_chksum(pkt)
        pkt += chr(chksum >> 8)
        pkt += chr(chksum & 0xFF)
        pkt = '\xBE\xEF' + pkt

        self.ser.write(pkt)

        (cmd, pay) = self.recv()

        if cmd  == Command.ERROR.value | Command.REPLY.value:
            err = RadioException(ord(pay[0]))
            raise err

        elif Command.GET_CFG.value | Command.REPLY.value:
            return
        else:
            raise Exception('Unexpected response: ' + str((cmd, pay)))
