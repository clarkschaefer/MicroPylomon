import uasyncio as asyncio
import struct

import esp32

from binascii import hexlify
from machine import UART


def normalize_resistance(resistance):
    pass
    # make this use the nvs values eventually


def calc_speed(power, resistances_array):
    pass
    # from power, back out the speed based on an coefficients in a resistance polynomial


def flip_le_ascii(s):
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
    pass
    # 

async def packet_parse(buf, packetready_evt):
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
        
        # print(hexlify(buf[:2], "-"))
        
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
    buf = bytearray(20)
    head_uart = UART(1, baudrate=19200, tx=33, rx=26, timeout=5)
    bike_uart = UART(2, baudrate=19200, tx=32, rx=25, timeout=5)
    
    packetready = asyncio.Event()

    asyncio.create_task(serial_mirror(buf, head_uart, bike_uart, packetready))
    asyncio.create_task(packet_parse(buf, packetready))
    while True:
        await asyncio.sleep(0)

asyncio.run(main())