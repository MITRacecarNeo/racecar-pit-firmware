import serial
import struct
import numpy as np
import time
import collections

## ---------------- Formats and Schemas ------------------- ##
# '>' - Big Endian
# '<' - Little Endian
type_map_lil_endian = {
    "uint8":  '<u1',
    "int8":   '<i1',
    "uint16": '<u2',
    "int16":  '<i2',
    "uint32": '<u4',
    "int32":  '<i4',
    "float": '<f', 
    "bool": '<?' 
}

# CMD_Packet_SCHEMA = np.dtype([
#     ('deltaStr', type_map_lil_endian['uint16']),
#     ('deltaThr', type_map_lil_endian['uint16'])
# ])

HEADER_PACKET_SCHEMA = np.dtype([
    ('MagicNum', type_map_lil_endian['uint32']), 
    ('VersionNum', type_map_lil_endian['uint8']), 
    ('IDNumber', type_map_lil_endian['uint8']), 
    ('PacketBytes', type_map_lil_endian['uint16']),
    ('FrameBytes', type_map_lil_endian['uint32']), 
    ('PacketIndex', type_map_lil_endian['uint16']), 
    ('PacketsPerFrame', type_map_lil_endian['uint16'])
])

HEADER_PACKET_NO_MAGIC_SCHEMA = np.dtype([ 
    ('VersionNum', type_map_lil_endian['uint8']), 
    ('IDNumber', type_map_lil_endian['uint8']), 
    ('PacketBytes', type_map_lil_endian['uint16']),
    ('FrameBytes', type_map_lil_endian['uint32']), 
    ('PacketIndex', type_map_lil_endian['uint16']), 
    ('PacketsPerFrame', type_map_lil_endian['uint16'])
])

RXPACKET_PACKET_SCHEMA = np.dtype([
    ('header', HEADER_PACKET_NO_MAGIC_SCHEMA), 
    ('Timestamp', type_map_lil_endian['int32']), 
    ('VoltAndCurr', type_map_lil_endian['uint16'], 2), 
    ('RxReceiver', type_map_lil_endian['uint16'], 8),
    ('IMU', type_map_lil_endian['float'], 9), 
    ('Encoder', type_map_lil_endian['int16']), 
    ('EKF', type_map_lil_endian['float'], 3),
    ('Checksum', type_map_lil_endian['uint16'])
])

TXPACKET_PACKET_SCHEMA = np.dtype([
    ('header', HEADER_PACKET_SCHEMA), 
    ('ServoAng', type_map_lil_endian['float']), 
    ('MotorAng', type_map_lil_endian['float']), 
    ('DotMatrix', type_map_lil_endian['bool'], 192),
    ('LED', type_map_lil_endian['uint8'], 255), 
    ('SystemState', type_map_lil_endian['uint8']),
    ('Checksum', type_map_lil_endian['uint16'])
])

# steer = int(input("Enter Servo Microseconds : 1025 : 1500 : 1975\n"))
# throt = int(input("Enter Throttle Microseconds : 750 : 1500 : 2250\n"))

rxPacketPong = np.zeros(1, dtype = RXPACKET_PACKET_SCHEMA)

txPacketPing = np.zeros(1, dtype = TXPACKET_PACKET_SCHEMA)
txPacketPong = np.zeros(1, dtype = TXPACKET_PACKET_SCHEMA)

# CMD_Packet = np.zeros(1, dtype=CMD_Packet_SCHEMA)
# CMD_Packet['deltaStr'] = steer
# CMD_Packet['deltaThr'] = throt

ser = serial.Serial('/dev/ttyAMA0', 230400)

# ser.write(CMD_Packet.tobytes())

window = collections.deque(maxlen=4)  # creates a "window" where can slide along the array for 0xDEADBEEF which is 4 bytes long

while True:
    rxPacket = ser.read(1)
    if not rxPacket:
        continue
    window.append(rxPacket[0])
    if(len(window) == 4):
        rxDat = struct.unpack('<L', bytes(window))[0] # outputs tuple, and then extracts the first element
        # print(f'{rxDat}')
        if(rxDat == 0xDEADBEEF):
            # print("yay")
            
            rxPacket = ser.read(rxPacketPong.itemsize)
            rxPacketPing = np.frombuffer(rxPacket, dtype = RXPACKET_PACKET_SCHEMA)[0]
            print(f'Length of rxPacketPing: {rxPacketPing[3]}')   
    
    time.sleep(0.001)
# print(f'Sending Servo: {CMD_Packet['deltaStr']} Throt: {CMD_Packet['deltaThr']}')
