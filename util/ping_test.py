#!/usr/bin/env python3

from com_radio import Radio

#def crc16(data: bytes, poly=0x8408):
def crc16(data: bytes, poly=0x8408):
    '''
    CRC-16-CCITT Algorithm
    '''
    data = bytearray(data)
    crc = 0xFFFF
    for b in data:
        cur_byte = 0xFF & b
        for _ in range(0, 8):
            if (crc & 0x0001) ^ (cur_byte & 0x0001):
                crc = (crc >> 1) ^ poly
            else:
                crc >>= 1
            cur_byte >>= 1
    crc = (~crc & 0xFFFF)
    crc = (crc << 8) | ((crc >> 8) & 0xFF)
    
    return crc & 0xFFFF


def crc( data):
    reg = 0xFFFF
    for x in data:
        for _ in range(0, 8):
            if(((reg & 0x8000) >> 8) ^ (x & 0x80)):
                reg = ((reg << 1) ^ 0x1021 ) & 0xffff ;
            else:
                reg = (reg << 1) & 0xffff;
            x = (x << 1) & 0xff;
    return reg

data = b'\xDBLinkSat=BestSat\x00\x00'
#data = b'LinkSat=BestSat'

lfr=Radio('/dev/ttyUSB0')
cksum = lfr.checksum(data[16:])
print(hex(cksum))
data = data + bytes([cksum>>8,cksum& 0xff])

lfr.tx(data)
#data =bytes([len(data)]) + data
print(data.hex())
print(hex(crc16(data,0x1021)))
print(hex(crc(data)))
