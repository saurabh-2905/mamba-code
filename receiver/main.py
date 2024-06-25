# from network import LoRa
from network import LoRa
import usocket
import time
import sys
import pycom
import ustruct


# def read_config(path="config"):
#     """
#     Reads the config file, to get the relevant board ids.
#     """
#     with open("config", "r") as f:
#         config = f.read()
#     config = config.split('\n')[0].split(',')
#     for i in range(len(config)):
#         config[i] = int(config[i])
#     return config

# def lora_init():
#     """
#     Initialises the SX1276 with 868MHz and a SF of 7.
#     Default BW: .
#     Default CR: .
#     """
#     lora.init(868000000, 7)


# def receive():
#     """
#     Waits for a message and returns it.
#     """
#     lora.changemode(1)
#     while True:
#         msg = lora.recv()[0]
#         if msg:
#             return msg


# def send(msg):
#     """
#     Sends a given bytestring, or formats a string and then sends it.
#     """
#     lora.changemode(0)
#     if isinstance(msg, str):
#         msg = msg.encode()
#     lora.send(msg)


def crc32(crc, p, len):
    """
    Returns crc32 for a given length.
    """
    crc = 0xffffffff & ~crc
    for i in range(len):
        crc = crc ^ p[i]
        for j in range(8):
            crc = (crc >> 1) ^ (0xedb88320 & -(crc & 1))
    return 0xffffffff & ~crc



# # board_ids based on the manuall numbering of the boards (to map to old ids)
# board_ids = read_config()


# Connect WIFI and MQTT
MESSAGE_LENGTH = 66  # 58+4+4
_pkng_frmt = '>12f3HI'

# lora_init()

# Start of loop
all_values = []


# Please pick the region that matches where you are using the device

pycom.heartbeat(False)
pycom.rgbled(0x008B00) # green

try:
    lora = LoRa(mode=LoRa.LORA, region=LoRa.EU868)
    s = usocket.socket(usocket.AF_LORA, usocket.SOCK_RAW)
    s.setblocking(False)
    i = 0
    while True:
        # recv_msg = receive()
        recv_msg = s.recv(80)
        print(recv_msg)
        if len(recv_msg) == MESSAGE_LENGTH:  # to differentiate between heartbeat and msg
            if ustruct.unpack(">L", recv_msg[-4:])[0] != crc32(0, recv_msg[:-4], 62):
                print('Invalid CRC32 in msg')
            else:
                # exclude timstamp and crc (8 bytes) to get msg
                values = ustruct.unpack(_pkng_frmt, recv_msg[:-8])
                timestamp = list(ustruct.unpack(">L", recv_msg[-8:-4]))
                receiver_timestamp = time.localtime()
                # send ACK
                s.send(str(values[15]) + ',' + str(timestamp[0]))
                # send ACK
                # send(str(values[15]) + ',' + str(timestamp[0]))

                print('ACK sent', str(values[15]))

        else:
            print('Corrupted messge with length:', len(recv_msg))
        time.sleep(0.5)
except Exception as e:
    pycom.heartbeat(True)
    print(e)
    sys.print_exception(e)