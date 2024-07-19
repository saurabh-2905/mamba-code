
from network import LoRa
import socket
import time
import pycom
import sys
# import random
from machine import Pin


pycom.heartbeat(False)
lora = LoRa(mode=LoRa.LORA, region=LoRa.EU868, sf=7, bandwidth=LoRa.BW_125KHZ, coding_rate=LoRa.CODING_4_5, )
s = socket.socket(socket.AF_LORA, socket.SOCK_RAW)
s.setblocking(False)
i = 0

ext_button = Pin('P10', mode=Pin.IN, pull=Pin.PULL_UP)

try:
    while True:
        ################# Auto send data every 2 seconds #################
        # s.send('Hello World, This is noise packet for Testing')
        # ### blikn the led for 500 ms
        # pycom.rgbled(0x002200) # green
        # time.sleep(0.1)
        # pycom.rgbled(0x000000)  # off
        # print('Ping {}'.format(i))
        # i= i+1
        # # if i%3 < 3:
        # #     print('Sending after 4 sec')
        # #     time.sleep(1)
        # # elif random.random() < 0.8:
        # #     print('Sending after 6 sec')
        # #     time.sleep(3)
        # # else:
        # #     print('Sending after 8 sec')
        # #     time.sleep(5)
        
        # time.sleep(2)
        # pycom.rgbled(0x000022) # blue
        # time.sleep(0.2)
        # pycom.rgbled(0x000000)  # off
        # time.sleep(1)

        ################# Manual send data #################

        if ext_button() == 0:
            print('Button pressed, send msg')
            s.send('Hello World, This is noise packet for Testing')
            # print('Ping {}'.format(i))
            pycom.rgbled(0x002200) # green
            time.sleep(0.1)
            pycom.rgbled(0x000000)  # off
            time.sleep(0.2)
        else:
            print('Button not pressed')
            pycom.rgbled(0x000022) # blue
            time.sleep(0.2)
            pycom.rgbled(0x000000)  # off
            time.sleep(1)

except:
    s.close()
    print('Socket closed')
    pycom.rgbled(0x550000) # red
    time.sleep(1)
    pycom.heartbeat(True)
    sys.exit()

