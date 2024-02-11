"""
fully sniff a Peloton Bike (original)'s level-shifted RS-232 and spit that data out as if we're a bike computer
"""

import uasyncio as asyncio
import aioble
import bluetooth
import esp32 # we don't need this until I do the NVS stuff technically but whatever

from struct import pack
from binascii import hexlify
from machine import UART, Pin
from micropython import const
from neopixel import NeoPixel

# ===== BLUETOOTH =====

# ----- SERVICES -----
# cycling power service; needs notify/read/write
_UUID_CYCLING_POWER_SERVICE = bluetooth.UUID(0x1818)
# cycling speed and cadence service; needs notify/read/write
_UUID_CYCLING_CADENCE_SPEED_SERVICE = bluetooth.UUID(0x1816)


# ----- CHARACTERISTICS -----
# cycling power feature characteristic, mandatory read
_UUID_CYCLING_POWER_FEATURE = bluetooth.UUID(0x2A65)
# cycling power feature characteristic cycling power feature field flags, we use these later to set other flags too
# we'll shift them over later before we put them in the structs
_PEDAL_POWER_BALANCE_SUPPORTED                                      = const(False)
_ACCUMULATED_TORQUE_SUPPORTED                                       = const(False)
_CP_WHEEL_REVOLUTION_DATA_SUPPORTED                                 = const(False)
_CP_CRANK_REVOLUTION_DATA_SUPPORTED                                 = const(False)
_EXTREME_MAGNITUDES_SUPPORTED                                       = const(False)
_EXTREME_ANGLES_SUPPORTED                                           = const(False)
_TOP_BOTTOM_DEAD_SPOT_ANGLES_SUPPORTED                              = const(False)
_ACCUMULATED_ENERGY_SUPPORTED                                       = const(False)
_OFFSET_COMPSENATION_INDICATOR_SUPPORTED                            = const(False)
_CYCLING_POWER_MEASUREMENT_CHARACTERISTIC_CONTENT_MASKING_SUPPORTED = const(False)
_CP_MULTIPLE_SENSOR_LOCATIONS_SUPPORTED                             = const(False)
_CRANK_LENGTH_ADJUSTMENT_SUPPORTED                                  = const(False)
_CHAIN_LENGTH_ADJUSTMENT_SUPPORTED                                  = const(False)
_CHAIN_WEIGHT_ADJUSTMENT_SUPPORTED                                  = const(False)
_SPAN_LENGTH_ADJUSTMENT_SUPPORTED                                   = const(False)
_SENSOR_MEASUREMENT_CONTEXT                                         = const(0b0) # 0 = force based, 1 = torque based
_INSTANTANEOUS_MEASUREMENT_DIRECTION_SUPPORTED                      = const(False)
_FACTORY_CALIBRATION_DATE_SUPPORTED                                 = const(False)
_ENHANCED_OFFSET_CALIBRATION_PROCEDURE_SUPPORTED                    = const(False)
_DISTRIBUTED_SYSTEM_SUPPORTED                                       = const(0b00) # 0 = legacy, 1 = not distributed, 2 = yes distributed, 3 = RFU

# cycling power measurement characteristic, mandatory notify
_UUID_CYCLING_POWER_MEASUREMENT = bluetooth.UUID(0x2A63)
# cycling power measurement characteristic cycling power measurement field flags
# we don't support using software at all for any reason, and it's important to commit that to memory
_PEDAL_POWER_BALANCE_PRESENT       = const(False and _PEDAL_POWER_BALANCE_SUPPORTED)
_PEDAL_POWER_BALANCE_REFERENCE     = const(False and _PEDAL_POWER_BALANCE_SUPPORTED)
_ACCUMULATED_TORQUE_PRESENT        = const(False and _ACCUMULATED_TORQUE_SUPPORTED)
_ACCUMULATED_TORQUE_SOURCE         = const(0b1 and _ACCUMULATED_TORQUE_SUPPORTED) # 0 = wheel based, 1 = crank based
_CP_WHEEL_REVOLUTION_DATA_PRESENT  = const(False and _CP_WHEEL_REVOLUTION_DATA_SUPPORTED)
_CP_CRANK_REVOLUTION_DATA_PRESENT  = const(False and _CP_CRANK_REVOLUTION_DATA_SUPPORTED)
_EXTREME_FORCE_MAGNITUDES_PRESENT  = const(False and _EXTREME_MAGNITUDES_SUPPORTED
                                           and (_SENSOR_MEASUREMENT_CONTEXT is True))
_EXTREME_TORQUE_MAGNITUDES_PRESENT = const(False and _EXTREME_MAGNITUDES_SUPPORTED
                                           and (_SENSOR_MEASUREMENT_CONTEXT is False))
_EXTREME_ANGLES_PRESENT            = const(False and _EXTREME_ANGLES_SUPPORTED)
_TOP_DEAD_SPOT_ANGLE_PRESENT       = const(False and _TOP_BOTTOM_DEAD_SPOT_ANGLES_SUPPORTED)
_BOTTOM_DEAD_SPOT_ANGLE_PRESENT    = const(False and _TOP_BOTTOM_DEAD_SPOT_ANGLES_SUPPORTED)
_ACUMULATED_ENERGY_PRESENT         = const(False and _ACCUMULATED_ENERGY_SUPPORTED)
_OFFSET_COMPENSATION_INDICATOR     = const(False and _OFFSET_COMPSENATION_INDICATOR_SUPPORTED)

# sensor location characteristic, mandatory read
_UUID_SENSOR_LOCATION = bluetooth.UUID(0x2A5D)
# sensor location characteristic sensor location field
# thankfully only one value
_SENSOR_LOCATION_FIELD = const(0) # 0 = other, not listing the rest since technically we don't know where the sensor is
# if you wanted to be a pedant you could say const(4) "front wheel"

# csc feature characteristic, mandatory read
_UUID_CSC_FEATURE = bluetooth.UUID(0x2A5C)
# csc feature characteristic feature field flags
# hypothetically cadence/speed data could be measured from a different location than the power data, so I'll make them distinct
_CSC_WHEEL_REVOLUTION_DATA_SUPPORTED     = const(False)
_CSC_CRANK_REVOLUTION_DATA_SUPPORTED     = const(False)
_CSC_MULTIPLE_SENSOR_LOCATIONS_SUPPORTED = const(False)

# csc measurement characteristic, mandatory notify
_UUID_CSC_MEASUREMENT = bluetooth.UUID(0x2A5B)
# csc measurement characteristic flags field
# keeping things separate still
_CSC_WHEEL_REVOLUTION_DATA_PRESENT = const(False and _CSC_WHEEL_REVOLUTION_DATA_SUPPORTED)
_CSC_CRANK_REVOLUTION_DATA_PRESENT = const(False and _CSC_CRANK_REVOLUTION_DATA_SUPPORTED)

# ----- OTHER STUFF -----
# advertising interval
_ADV_INTERVAL_MS = const(250000)
# appearance
_ADV_APPEARANCE_CYCLING_COMPUTER = const(0x0481)
# ----- SETUP -----
# aioble POWER service setup


# ===== RGB LED STUFF =====
n = NeoPixel(Pin(48), 1)


def _encode_power_feature():
    return pack("<4s", bytes(
                _PEDAL_POWER_BALANCE_SUPPORTED                                      << 0  |
                _ACCUMULATED_TORQUE_SUPPORTED                                       << 1  |
                _CP_WHEEL_REVOLUTION_DATA_SUPPORTED                                 << 2  |
                _CP_CRANK_REVOLUTION_DATA_SUPPORTED                                 << 3  |
                _EXTREME_MAGNITUDES_SUPPORTED                                       << 4  |
                _EXTREME_ANGLES_SUPPORTED                                           << 5  |
                _TOP_BOTTOM_DEAD_SPOT_ANGLES_SUPPORTED                              << 6  |
                _ACCUMULATED_ENERGY_SUPPORTED                                       << 7  |
                _OFFSET_COMPSENATION_INDICATOR_SUPPORTED                            << 8  |
                _CYCLING_POWER_MEASUREMENT_CHARACTERISTIC_CONTENT_MASKING_SUPPORTED << 9  |
                _CP_MULTIPLE_SENSOR_LOCATIONS_SUPPORTED                             << 10 |
                _CRANK_LENGTH_ADJUSTMENT_SUPPORTED                                  << 11 |
                _CHAIN_LENGTH_ADJUSTMENT_SUPPORTED                                  << 12 |
                _CHAIN_WEIGHT_ADJUSTMENT_SUPPORTED                                  << 13 |
                _SPAN_LENGTH_ADJUSTMENT_SUPPORTED                                   << 14 |
                _SENSOR_MEASUREMENT_CONTEXT                                         << 15 |
                _INSTANTANEOUS_MEASUREMENT_DIRECTION_SUPPORTED                      << 16 |
                _FACTORY_CALIBRATION_DATE_SUPPORTED                                 << 17 |
                _ENHANCED_OFFSET_CALIBRATION_PROCEDURE_SUPPORTED                    << 18 |
                _DISTRIBUTED_SYSTEM_SUPPORTED                                       << 19 ))


def _encode_power_measurement(raw_power):
    """
    encode all the flags that say we don't support anything, and also what power the Peloton is reading
    """
    total_size = 4
    flags = bytes(_PEDAL_POWER_BALANCE_PRESENT       << 0  |
                  _PEDAL_POWER_BALANCE_REFERENCE     << 1  |
                  _ACCUMULATED_TORQUE_PRESENT        << 2  |
                  _ACCUMULATED_TORQUE_SOURCE         << 3  |
                  _CP_WHEEL_REVOLUTION_DATA_PRESENT  << 4  |
                  _CP_CRANK_REVOLUTION_DATA_PRESENT  << 5  |
                  _EXTREME_FORCE_MAGNITUDES_PRESENT  << 6  |
                  _EXTREME_TORQUE_MAGNITUDES_PRESENT << 7  |
                  _EXTREME_ANGLES_PRESENT            << 8  |
                  _TOP_DEAD_SPOT_ANGLE_PRESENT       << 9  |
                  _BOTTOM_DEAD_SPOT_ANGLE_PRESENT    << 10 |
                  _ACUMULATED_ENERGY_PRESENT         << 11 |
                  _OFFSET_COMPENSATION_INDICATOR     << 12)
    instantaneous_power = int(raw_power / 10)
    # I'm not going to go to the effort of adding functionality to encode the other values here
    # really I shouldn't have gone to the effort of properly writing these flags
    return pack("<2sh", flags, instantaneous_power)


def _encode_sensor_location():
    """
    encode sensor location for the cycling power measurement sensor location characteristic
    """
    # we're using pack() here in case the target system is big-endian and does bytes() weird
    # I have no idea if there even are micropython targets that aren't little-endian
    return pack("<2s", bytes(_UUID_SENSOR_LOCATION))


def _encode_csc_feature():
    """
    encode csc feature flags for the feature characteristic
    """
    return pack("<2s", bytes(
                _CSC_WHEEL_REVOLUTION_DATA_SUPPORTED     << 0 |
                _CSC_CRANK_REVOLUTION_DATA_SUPPORTED     << 1 |
                _CSC_MULTIPLE_SENSOR_LOCATIONS_SUPPORTED << 2))


def _encode_csc_measurement(raw_cadence, raw_power):
    """
    encode cadence and speed data as well as required feature information
    """
    pass


def quickrgb(r,g,b): # TODO: make this an async task that manages light to indicate state
    global n
    n[0] = (r,g,b)
    n.write()


def normalize_resistance(resistance):
    """
    ask the nvs manager for the resistance data (or make that a global, not sure yet)
    and use that to make a scaled resistance value
    probably have to make a new func to generate the polynomials for smoothing this data
    """
    pass


def calc_speed(raw_power, resistances_array):
    """
    from raw_power, back out the speed based on an coefficients in a resistance polynomial
    probably store that as a global too
    """
    pass


def reverse_bytes_to_int(s):
    """
    decode a bytestring as ascii, then reverse it and return it as an int
    maybe there's a more efficient way to be unpacking the structs?
    """
    r = ""
    for c in s.decode("ascii"):
        r = c + r

    return int(r)


async def serial_mirror_task(buf, uartA, uartB, packetready_evt):
    """
    takes a buffer, two UARTs, and a flag to set when it's read from either of them
    I suppose I could set events based on where the message came from,
    but I can already parse that out 
    """
    len = 0
    
    while True:
        if uartA.any():
            len = uartA.readinto(buf)
            uartB.write(buf[:len])
            packetready_evt.set()
            
        if uartB.any() and not packetready_evt.is_set(): # make sure we don't overwrite
            len = uartB.readinto(buf)
            uartA.write(buf[:len])
            packetready_evt.set()
            
        await asyncio.sleep_ms(0) # yield, but don't wait too long to check for the response packet


async def nvs_manager():
    """
    stash values when asked by the packet parser, checking if they've changed
    that would mean we're on a different peloton
    """

async def bluetooth_task():
    """
    initialize and manage the bluetooth state
    """
    power_service = aioble.Service(_UUID_CYCLING_POWER_SERVICE)
    csc_service = aioble.Service(_UUID_CYCLING_CADENCE_SPEED_SERVICE)
    
    power_feature_characteristic = aioble.Characteristic(
        power_service,
        _UUID_CYCLING_POWER_FEATURE,
        read=True)
    power_measurement_characteristic = aioble.Characteristic(
        power_service,
        _UUID_CYCLING_POWER_MEASUREMENT,
        notify=True)
    sensor_location_characteristic = aioble.Characteristic(
        power_service,
        _UUID_SENSOR_LOCATION,
        read=True)
    csc_feature_characteristic = aioble.Characteristic(
        csc_service,
        _UUID_CSC_FEATURE,
        read=True)
    csc_measurement_characteristic = aioble.Characteristic(
        csc_service,
        _UUID_CSC_MEASUREMENT,
        notify=True)
    
    
    csc_feature_characteristic.write(_encode_csc_feature())
    csc_measurement_characteristic.write(_encode_csc_measurement(0,0)) # we're not moving or pedaling to start
    
    while True:
        quickrgb(5,0,0) # start LED red
        async with await aioble.advertise(
            _ADV_INTERVAL_MS,
            name="mpylomon",
            services=[_UUID_CYCLING_POWER_SERVICE,
                      _UUID_CYCLING_CADENCE_SPEED_SERVICE],
            appearance=_ADV_APPEARANCE_CYCLING_COMPUTER,
        ) as connection:
            quickrgb(0,0,5) # we're connected, go blue
            print(dir(connection))
            await connection.disconnected()

            
            

async def packet_parse_task(buf, packetready_evt):
    """
    honestly kind of the mainloop
    responding to new serial packets is the main driver of most of the state changes here
    """
    is_query = 0
    is_resp = 0
    
    cal_i = 0
    cal_n = 0
    
    cur_cadence = 0
    cur_POWER = 0
    cur_resistance = 0
    cur_resistance_normalized = 0
    
    while True:
        await packetready_evt.wait()
        
        # print(hexlify(buf[:2], "-")) # show what packet header we're dealing with
        
        if buf[0] == 0xF1: # message from bike
            if buf[1] == 0xFE: # bike responding to bootup
                pass
            
            elif buf[1] == 0xFB: # bike responding with ID
                pass
            
            elif buf[1] == 0xF7: # bike responding with resistance cal value
                resistance = reverse_bytes_to_int(buf[3:7])
                print(reverse_bytes_to_int(buf[3:7]))
            
            elif buf[1] == 0x41: # bike responding with cadence
                pass
                print(reverse_bytes_to_int(buf[3:6]))
            
            elif buf[1] == 0x44: # bike responding with power
                pass
                print(reverse_bytes_to_int(buf[3:8]))
            
            elif buf[1] == 0x4A: # bike responding with raw resistance
                pass
                print(reverse_bytes_to_int(buf[3:7]))
        
        elif buf[0] == 0xFE: # head bootup message 
            pass
        
        elif buf[0] == 0xF5: # head requests
            if buf[1] == 0x41: # head requesting cadence
                pass
            
            elif buf[1] == 0x44: # head requesting output
                pass
            
            elif buf[1] == 0x4A: # head requesting raw resistance
                pass
            
            else: # head requesting bike ID
                pass
        
        elif buf[0] == 0xF7: # head requesting resistance cal value
            pass

        else:
            raise Exception("Unknown message", str(buf))

        packetready_evt.clear()
        await asyncio.sleep(0) # I think I might not need this line? since I start by waiting for the event


async def main():
    """
    create some UARTs that get handed to the serial mirror
    maybe I'll move these to globals, since that's how they're used
    
    also making an event to control the packet parser
    ...that should probably also be a global
    """
    buf = bytearray(20)
    head_uart = UART(1, baudrate=19200, tx=18, rx=17, timeout=5)
    bike_uart = UART(2, baudrate=19200, tx=16, rx=15, timeout=5)
    
    packetready = asyncio.Event()

    asyncio.create_task(serial_mirror_task(buf, head_uart, bike_uart, packetready))
    asyncio.create_task(packet_parse_task(buf, packetready))
    asyncio.create_task(bluetooth_task())
    while True:
        await asyncio.sleep(0)

asyncio.run(main())

