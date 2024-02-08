import uasyncio as asyncio
import struct

import esp32

from binascii import hexlify
from machine import UART


def normalize_resistance(resistance):
    """
    ask the nvs manager for the resistance data (or make that a global, not sure yet)
    and use that to make a scaled resistance value
    probably have to make a new func to generate the polynomials for smoothing this data
    """
    pass


def calc_speed(power, resistances_array):
    """
    from power, back out the speed based on an coefficients in a resistance polynomial
    probably store that as a global too
    """
    pass


def flip_le_ascii(s):
    """
    decode a bytestring as ascii, then reverse it and return it as an int
    maybe there's a more efficient way to be unpacking the structs?
    """
    r = ""
    for c in s.decode("ascii"):
        r = c + r

    return int(r)


async def serial_mirror(buf, uartA, uartB, packetready_evt):
    """
    takes a buffer, two UARTs, and a flag to set when it's read from either of them
    I suppose I could set events based on where the message came from,
    but I can already parse that out 
    """
    len = 0
    
    while True:
        lenA = uartA.any()
        lenB = uartB.any()
        
        if uartA.any():
            len = uartA.readinto(buf)
            uartB.write(buf[:len])
            packetready_evt.set()
        if uartB.any() and not packetready_evt.is_set():
            len = uartB.readinto(buf)
            uartA.write(buf[:len])
            packetready_evt.set()
        
        len = 0
        await asyncio.sleep_ms(1)


async def nvs_manager():
    """
    stash values when asked by the packet parser, checking if they've changed
    that would mean we're on a different peloton
    """

async def packet_parse(buf, packetready_evt):
    """
    honestly kind of the mainloop
    responding to new serial packets is the main driver of most of the state changes here
    """
    is_query = 0
    is_resp = 0
    
    cal_i = 0
    cal_n = 0
    
    cur_cadence = 0
    cur_power = 0
    cur_resistance = 0
    cur_resistance_normalized = 0
    
    while True:
        await packetready_evt.wait()
        
        # print(hexlify(buf[:2], "-")) # show what packet header we're dealing with
        
        if buf[0] == 0xF1:
            if buf[1] == 0xFE:
                print("bike bootup response")
            
            elif buf[1] == 0xFB:
                print("bike ID response")
            
            elif buf[1] == 0xF7:
                print("bike resistance cal value")
                print(flip_le_ascii(struct.unpack("<4s", buf[3:7])[0]))
            
            elif buf[1] == 0x41:
                print("bike cadence response")
                print(flip_le_ascii(struct.unpack("<3s", buf[3:6])[0]))
            
            elif buf[1] == 0x44:
                print("bike power response")
                print(flip_le_ascii(struct.unpack("<5s", buf[3:8])[0]))
            
            elif buf[1] == 0x4A:
                print("bike raw resistance response")
                print(flip_le_ascii(struct.unpack("<4s", buf[3:7])[0]))
        
        elif buf[0] == 0xFE:
            print("head bootup message")
        
        elif buf[0] == 0xF5:
            if buf[1] == 0x41:
                print("head getting cadence")
            
            elif buf[1] == 0x44:
                print("head getting output")
            
            elif buf[1] == 0x4A:
                print("head getting raw resistance")
            
            else:
                print("head getting bike ID")
        
        elif buf[0] == 0xF7:
            print("head getting resistance cal value")            

        else:
            raise Exception("Unknown message" + str(buf))

        packetready_evt.clear()
        await asyncio.sleep(0)


async def main():
    """
    create some UARTs that get handed to the serial mirror
    maybe I'll move these to globals, since that's how they're used
    
    also making an event to control the packet parser
    ...that should probably also be a global
    """
    buf = bytearray(20)
    head_uart = UART(1, baudrate=19200, tx=33, rx=26, timeout=5)
    bike_uart = UART(2, baudrate=19200, tx=32, rx=25, timeout=5)
    
    packetready = asyncio.Event()

    asyncio.create_task(serial_mirror(buf, head_uart, bike_uart, packetready))
    asyncio.create_task(packet_parse(buf, packetready))
    while True:
        await asyncio.sleep(0)

asyncio.run(main())