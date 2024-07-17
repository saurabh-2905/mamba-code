# from lora import LoRa
# import time
# import utime
# import micropython
# from machine import Pin, I2C, SoftSPI, Timer

# def cb_30():
#     global cb_30_done
#     cb_30_done = True

# ####### variables - global #######
# cb_30_done = False

# ################## Initialize LoRa ##################
# SPI_BUS = SoftSPI(baudrate=10000000, sck=Pin(18, Pin.OUT),
#                       mosi=Pin(23, Pin.OUT), miso=Pin(19, Pin.IN))
    
# lora = LoRa(SPI_BUS, True, cs=Pin(5, Pin.OUT), rx=Pin(2, Pin.IN))

# # Create Timers
# timer0 = Timer(0)
# timer0.init(period=3, mode=Timer.PERIODIC, callback=cb_30)

# while True:
#     if cb_30_done:
#         cb_30_done = False
#         lora.send('Hello, This is noise packet for testing')
#         print('Sent')
#     time.sleep(1)


from network import LoRa
import socket
import time
import pycom
import sys


pycom.heartbeat(False)
lora = LoRa(mode=LoRa.LORA, region=LoRa.EU868, sf=7, bandwidth=LoRa.BW_125KHZ, coding_rate=LoRa.CODING_4_5, )
s = socket.socket(socket.AF_LORA, socket.SOCK_RAW)
s.setblocking(False)
i = 0

try:
    while True:
        s.send('Hello World, This is noise packet for Testing')
        print('Ping {}'.format(i))
        i= i+1
        ### blikn the led for 500 ms
        pycom.rgbled(0x002200) # green
        time.sleep(0.1)
        pycom.rgbled(0x000000)  # off
        time.sleep(2)
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

