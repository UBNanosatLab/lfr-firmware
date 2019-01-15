#!/usr/bin/env python3

import sys
import socket
from time import sleep
import threading

import serial

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
    pass

class Radio:

    def __init__(self, port, baud=9600):
        self.ser = serial.Serial(port, baud)
        self.state = ParseState.SYNC_H


    def send(self, data):
        pkt = chr(Command.TXDATA.value)
        pkt += chr(len(data))
        pkt += data
        chksum = compute_chksum(pkt)
        pkt += chr(chksum >> 8)
        pkt += chr(chksum & 0xFF)
        pkt = '\xBE\xEF' + pkt

        self.ser.write(bytes([ord(x) for x in pkt]))

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


import curses

pkt = ''

def recv_loop(radio, msgwin):
    global pkt
    while True:
        (flags, cmd, pay) = radio.recv()
        cmd = Command(cmd ^ Command.REPLY.value)
        if cmd is Command.TXDATA:
            pkt = 'TX_ACK'
        elif cmd is Command.ERROR:
            pkt = 'ERR: ' + str(ord(pay[0]))
        elif cmd is Command.RXDATA:
            pkt = 'RXDATA: ' + payload
        elif cmd is Command.RESET:
            pkt = 'RESET'
        elif cmd is Command.NOP:
            pkt = 'NOP'
        else:
            pkt = 'UNIMPL: ' + str(cmd)

def main(stdscr):

    msg = ''
    radio = Radio(sys.argv[1])
    stdscr.clear()
    height = curses.LINES
    width = curses.COLS

    curses.start_color()
    curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_GREEN)
    curses.init_pair(2, curses.COLOR_BLACK, curses.COLOR_WHITE)

    msgwin = curses.newwin(height - 2, width, 0, 0)
    editwin = curses.newwin(1, width, height - 2, 0)
    statwin = curses.newwin(1, width, height - 1, 0)

    msgwin.scrollok(True)
    msgwin.clearok(True)
    editwin.nodelay(True)

    status = 'Starting up...'

    statwin.attron(curses.color_pair(1))
    statwin.addstr(0, 0, status)
    statwin.addstr(0, len(status), " " * (width - len(status) - 1))
    statwin.attroff(curses.color_pair(1))
    statwin.refresh()

    threading.Thread(target=recv_loop, args=[radio, msgwin]).start()
    global pkt

    status = 'Ready!'

    while True:

        statwin.attron(curses.color_pair(1))
        statwin.addstr(0, 0, status)
        statwin.addstr(0, len(status), " " * (width - len(status) - 1))
        statwin.attroff(curses.color_pair(1))
        statwin.refresh()

        editwin.clear()
        editwin.addstr(0, 0, msg)

        while True:

            if pkt:
                msgwin.addstr(pkt + '\n')
                msgwin.refresh()
                pkt = ''

            c = editwin.getch()
            if (c == ord('\n')):
                msgwin.addstr('TX: ' + msg + '\n')
                radio.send(msg)
                msgwin.refresh()
                msg = ''
                break
            elif (c == 127):
                if msg:
                    msg = msg[:-1]
                    break
            elif (c != curses.ERR):
                msg += chr(c)
                break


if __name__ == '__main__':
    curses.wrapper(main)
