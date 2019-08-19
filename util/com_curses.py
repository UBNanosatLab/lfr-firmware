#!/usr/bin/env python3

import sys
import socket
from time import sleep
import threading
from com_radio import Radio, Command
import curses

pkt = ''

def recv_loop(radio, msgwin):
    global pkt
    while True:
        (cmd, pay) = radio.recv()
        cmd = Command(cmd ^ Command.REPLY.value)
        if cmd is Command.TXDATA:
            pkt = 'TX_ACK'
        elif cmd is Command.ERROR:
            pkt = 'ERR: ' + str(ord(pay[0]))
        elif cmd is Command.RXDATA:
            pkt = 'RXDATA: ' + pay.decode("utf-8")
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
                # tx(msg.encode('ascii'))
                radio.send_pkt(Command.TXDATA, msg.encode('utf-8'))
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
