# -------------------------------------------------------------------------------
# author: Malavika Unnikrishnan, Florian Stechmann, Saurabh Band
# date: 20.04.2022
# function: code for esp32 board with lora module.
# -------------------------------------------------------------------------------

from machine import Pin, I2C, SoftSPI, Timer
import machine
import micropython
import ustruct, ubinascii, uhashlib
import time
import utime
import random

from scd30 import SCD30
from lora import LoRa
from mcp3221 import MCP3221
from bmp180 import BMP180
from am2301 import AM2301
from lib.varlogger import VarLogger as vl
import _thread
import gc

#vl.log(var='0', fun=_fun_name, clas=_cls_name, th=_thread_id)     ### standard logging format

# ------------------------ function declaration -------------------------------
gc.collect()

def measure_scd30():
    """
    Takes CO2 reading.
    """
    _thread_id = _thread.get_ident()
    _fun_name = 'measure_scd30'
    _cls_name = '0'

    vl.log(var='0', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=None)
    if scd30.get_status_ready() == 1:
        return scd30.read_measurement()
    else:
        return (-1, -1, -1)


def measure_co():
    """
    Takes CO reading.
    """
    _thread_id = _thread.get_ident()
    _fun_name = 'measure_co'
    _cls_name = '0'

    vl.log(var='0', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=None)
    return MCP_CO.read_measurement_co()


def measure_o2():
    """
    Takes O2 reading.
    """
    _thread_id = _thread.get_ident()
    _fun_name = 'measure_o2'
    _cls_name = '0'

    vl.log(var='0', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=None)
    return MCP_O2.read_measurement_o2()


def measure_bmp():
    """
    Takes pressure reading.
    """
    _thread_id = _thread.get_ident()
    _fun_name = 'measure_bmp'
    _cls_name = '0'

    vl.log(var='0', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=None)
    return BMP.pressure/100


def measure_am1(stat):
    """
    Temp & humidity sensor 1 reading.
    """
    _thread_id = _thread.get_ident()
    _fun_name = 'measure_am1'
    _cls_name = '0'

    global am_temp, am_hum
    try:
        am_temp, am_hum = AM2301_1.read_measurement()
        vl.log(var='am_temp', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=am_temp)    ### logging
        vl.log(var='am_hum', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=am_hum)    ### logging
    except Exception:
        CONNECTION_VAR[stat] = 0
        vl.log(var='CONNECTION_VAR', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=CONNECTION_VAR[stat])    ### logging
        


def measure_am2(stat):
    """
    Temp & humidity sensor 2 reading.
    """
    _thread_id = _thread.get_ident()
    _fun_name = 'measure_am2'
    _cls_name = '0'

    global am_temp, am_hum
    try:
        am_temp, am_hum = AM2301_2.read_measurement()
        vl.log(var='am_temp', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=am_temp)    ### logging
        vl.log(var='am_hum', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=am_hum)    ### logging

    except Exception:
        CONNECTION_VAR[stat] = 0
        vl.log(var='CONNECTION_VAR', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=CONNECTION_VAR[stat])    ### logging



def measure_am3(stat):
    """
    Temp & humidity sensor 3 reading.
    """
    _thread_id = _thread.get_ident()
    _fun_name = 'measure_am3'
    _cls_name = '0'

    global am_temp, am_hum
    try:
        am_temp, am_hum = AM2301_3.read_measurement()
        vl.log(var='am_temp', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=am_temp)    ### logging
        vl.log(var='am_hum', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=am_hum)    ### logging
    except Exception:
        CONNECTION_VAR[stat] = 0
        vl.log(var='CONNECTION_VAR', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=CONNECTION_VAR[stat])    ### logging


def measure_am4(stat):
    """
    Temp & humidity sensor 4 reading.
    """
    _thread_id = _thread.get_ident()
    _fun_name = 'measure_am4'
    _cls_name = '0'

    global am_temp, am_hum
    try:
        am_temp, am_hum = AM2301_4.read_measurement()
        vl.log(var='am_temp', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=am_temp)    ### logging
        vl.log(var='am_hum', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=am_hum)    ### logging
    except Exception:
        CONNECTION_VAR[stat] = 0
        vl.log(var='CONNECTION_VAR', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=CONNECTION_VAR[stat])    ### logging


def cb_30(p):
    """
    Callback for the sending of msgs every btw 20s-40s.
    """
    _thread_id = _thread.get_ident()
    _fun_name = 'cb_30'
    _cls_name = '0'

    global cb_30_done
    cb_30_done = True
    vl.log(var='cb_30_done', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=cb_30_done)    ### logging


def cb_retrans(p):
    """
    Callback for resending msgs.
    """
    _thread_id = _thread.get_ident()
    _fun_name = 'cb_retrans'
    _cls_name = '0'

    global cb_retrans_done
    cb_retrans_done = True
    vl.log(var='cb_retrans_done', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=cb_retrans_done)    ### logging



def lora_scheduled(r_msg):
    """
    Scheduled lora callback.
    """
    _thread_id = _thread.get_ident()
    _fun_name = 'lora_scheduled'
    _cls_name = '0'

    global cb_lora_recv, rcv_msg
    cb_lora_recv = True
    vl.log(var='cb_lora_recv', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=cb_lora_recv)    ### logging
    rcv_msg.append(r_msg)
    vl.log(var='rcv_msg', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=r_msg)    ### logging



def cb_lora(p):
    """
    Schedules lora callback.
    """
    _thread_id = _thread.get_ident()
    _fun_name = 'cb_lora'
    _cls_name = '0'

    micropython.schedule(lora_scheduled, p)
    vl.log(var='0', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=None)    ### logging



def crc32(crc, p, len):
    """
    crc = 0
    p = message
    len = length of msg
    """
    _thread_id = _thread.get_ident()
    _fun_name = 'crc32'
    _cls_name = '0'

    crc = 0xffffffff & ~crc
    vl.log(var='crc', fun=_fun_name, clas=_cls_name, th=_thread_id)    ### logging
    for i in range(len):
        crc = crc ^ p[i]
        # vl.log(var='crc', fun=_fun_name, clas=_cls_name, th=_thread_id)    ### logging
        for j in range(8):
            crc = (crc >> 1) ^ (0xedb88320 & -(crc & 1))
            # vl.log(var='crc', fun=_fun_name, clas=_cls_name, th=_thread_id)    ### logging

    return 0xffffffff & ~crc


def write_to_log(msg, timestamp):
    """
    Write a given Message to the file log.txt.
    """
    _thread_id = _thread.get_ident()
    _fun_name = 'write_to_log'
    _cls_name = '0'

    vl.log(var='0', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=None)    ### logging
    with open("log", "a") as f:
        f.write(msg + "\t" + timestamp + "\n")


def add_to_que(msg, current_time):
    """
    Adds given msg to the que with a given timestamp
    """
    _thread_id = _thread.get_ident()
    _fun_name = 'add_to_que'
    _cls_name = '0'

    global que
    print('Queue', len(que))
    if len(que) >= MAX_QUEUE:
        # pop the packet from the end of que (the oldest packet)
        que.pop()
        # add the newest msg at the front of que
        que = [(msg, current_time)] + que
        vl.log(var='que', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=(msg, current_time))    ### logging

    else:
        que = [(msg, current_time)] + que
        vl.log(var='que', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=(msg, current_time))    ### logging



def get_nodename():
    """
    Retuns the unique_id of the esp32
    """
    _thread_id = _thread.get_ident()
    _fun_name = 'get_nodename'
    _cls_name = '0'

    uuid = ubinascii.hexlify(machine.unique_id()).decode()
    vl.log(var='uuid', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=uuid)    ### logging
    node_name = "ESP_" + uuid
    vl.log(var='node_name', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=node_name)    ### logging
    return node_name


def get_node_id(hex=False):
    """
    Get node id, which consists of four bytes unsigned int.
    Return as hex, according to parameter.
    """
    _thread_id = _thread.get_ident()
    _fun_name = 'get_node_id'
    _cls_name = '0'

    node_id = ubinascii.hexlify(uhashlib.sha1(
        machine.unique_id()).digest()).decode("utf-8")[-8:]
    vl.log(var='node_id', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=node_id)    ### logging
    
    if hex:
        return node_id
    else:
        return int(node_id, 16)


def lora_rcv_exec(p):
    """
    lora receive msg processing
    """
    _thread_id = _thread.get_ident()
    _fun_name = 'lora_rcv_exec'
    _cls_name = '0'

    global cb_lora_recv, rcv_msg
    if cb_lora_recv:
        cb_lora_recv = False
        vl.log(var='cb_lora_recv', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=cb_lora_recv)    ### logging
        for i in range(len(rcv_msg)):
            msg = rcv_msg[i]
            vl.log(var='msg', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=msg)
            try:
                recv_msg = msg.decode()
                vl.log(var='recv_msg', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=recv_msg)
                # print('received msg', recv_msg)
                board_id, timestamp = recv_msg.split(',')
                vl.log(var='board_id', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=board_id)
                vl.log(var='timestamp', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=timestamp)
                if int(board_id) == SENSORBOARD_ID:
                    for each_pkt in que:
                        if each_pkt[1] == int(timestamp):
                            print('ACK received:', each_pkt[1])
                            que.remove(each_pkt)
            except Exception:
                pass
                # write_to_log('callback lora: {}'.format(e),
                # str(time.mktime(time.localtime())))
        rcv_msg = []
        vl.log(var='rcv_msg', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=rcv_msg)    ### logging

gc.collect()
################################## Start of the main code ##################################
_thread_id = _thread.get_ident()
_fun_name = '0'
_cls_name = '0'

vl.thread_status(_thread_id, 'active')     #### MUST

# Allcoate emergeny buffer for interrupt signals
micropython.alloc_emergency_exception_buf(100)

# packing format
_pkng_frmt = '>12f3HI'
vl.log(var='_pkng_frmt', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=_pkng_frmt)    ### logging

SENSORBOARD_ID = get_node_id()
vl.log(var='SENSORBOARD_ID', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=SENSORBOARD_ID)    ### logging


# ------------------------ constants and variables ----------------------------
# addresses of sensors
O2_ADRR = const(0x48)
vl.log(var='O2_ADRR', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=O2_ADRR)    ### logging
CO_ADRR = const(0x49)
vl.log(var='CO_ADRR', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=CO_ADRR)    ### logging
SCD30_ADRR = const(0x61)
vl.log(var='SCD30_ADRR', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=SCD30_ADRR)    ### logging
AM2301_1_ADRR = const(0)
vl.log(var='AM2301_1_ADRR', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=AM2301_1_ADRR)    ### logging
AM2301_2_ADRR = const(4)
vl.log(var='AM2301_2_ADRR', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=AM2301_2_ADRR)    ### logging
AM2301_3_ADRR = const(17)
vl.log(var='AM2301_3_ADRR', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=AM2301_3_ADRR)    ### logging
AM2301_4_ADRR = const(16)
vl.log(var='AM2301_4_ADRR', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=AM2301_4_ADRR)    ### logging

# connection_variables init for sensors
FAILED_LORA = 1
vl.log(var='FAILED_LORA', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=FAILED_LORA)    ### logging
CONNECTION_CO2 = 1
vl.log(var='CONNECTION_CO2', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=CONNECTION_CO2)    ### logging
CONNECTION_CO = 1
vl.log(var='CONNECTION_CO', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=CONNECTION_CO)    ### logging
CONNECTION_O2 = 1
vl.log(var='CONNECTION_O2', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=CONNECTION_O2)    ### logging
CONNECTION_BMP = 1
vl.log(var='CONNECTION_BMP', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=CONNECTION_BMP)    ### logging
CONNECTION_A1 = 1
vl.log(var='CONNECTION_A1', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=CONNECTION_A1)    ### logging
CONNECTION_A2 = 1
vl.log(var='CONNECTION_A2', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=CONNECTION_A2)    ### logging
CONNECTION_A3 = 1
vl.log(var='CONNECTION_A3', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=CONNECTION_A3)    ### logging
CONNECTION_A4 = 1
vl.log(var='CONNECTION_A4', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=CONNECTION_A4)    ### logging
MAX_QUEUE = const(10)
vl.log(var='MAX_QUEUE', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=MAX_QUEUE)    ### logging
scd_co2 = 0
vl.log(var='scd_co2', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=scd_co2)    ### logging
scd_temp = 0
vl.log(var='scd_temp', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=scd_temp)    ### logging
scd_hum = 0
vl.log(var='scd_hum', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=scd_hum)    ### logging
am_temp = 0
vl.log(var='am_temp', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=am_temp)    ### logging
am_hum = 0
vl.log(var='am_hum', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=am_hum)    ### logging

# list for measurements values
que = []
vl.log(var='que', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=None)    ### logging

# init cb booleans
cb_30_done = False
vl.log(var='cb_30_done', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=cb_30_done)    ### logging
cb_retrans_done = False
vl.log(var='cb_retrans_done', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=cb_retrans_done)    ### logging
cb_lora_recv = False
vl.log(var='cb_lora_recv', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=cb_lora_recv)    ### logging

# init msg intervals
msg_interval = 30000  # 30 sec
vl.log(var='msg_interval', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=msg_interval)    ### logging
retx_interval = 5000  # 5 sec
vl.log(var='retx_interval', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=retx_interval)    ### logging

# ------------------------ establish connections ------------------------------
# establish I2c Bus
try:
    I2CBUS = I2C(1, sda=Pin(21), scl=Pin(22), freq=100000)
    vl.log(var='I2CBUS', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=I2CBUS)    ### logging
except Exception:
    # raise  # TODO:set conn_variables to sensors zero
    write_to_log('I2C failed', str(time.mktime(time.localtime())))

# establish SPI Bus and LoRa (SX1276)
try:
    SPI_BUS = SoftSPI(baudrate=10000000, sck=Pin(18, Pin.OUT),
                      mosi=Pin(23, Pin.OUT), miso=Pin(19, Pin.IN))
    vl.log(var='SPI_BUS', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=SPI_BUS)    ### logging
    lora = LoRa(SPI_BUS, True, cs=Pin(5, Pin.OUT), rx=Pin(2, Pin.IN))
    vl.log(var='lora', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=lora)    ### logging
except Exception:
    FAILED_LORA = 0
    vl.log(var='FAILED_LORA', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=FAILED_LORA)    ### logging
    write_to_log('Lora failed', str(time.mktime(time.localtime())))

# create sensorobjects
try:
    scd30 = SCD30(I2CBUS, SCD30_ADRR)
    vl.log(var='scd30', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=scd30)    ### logging
    scd30.start_continous_measurement()
except Exception:
    CONNECTION_CO2 = 0
    vl.log(var='CONNECTION_CO2', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=CONNECTION_CO2)    ### logging
    write_to_log('co2 failed', str(time.mktime(time.localtime())))

try:
    MCP_CO = MCP3221(I2CBUS, CO_ADRR)
    vl.log(var='MCP_CO', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=MCP_CO)    ### logging
except Exception:
    CONNECTION_CO = 0
    vl.log(var='CONNECTION_CO', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=CONNECTION_CO)    ### logging
    write_to_log('co failed', str(time.mktime(time.localtime())))

try:
    MCP_O2 = MCP3221(I2CBUS, O2_ADRR)
    vl.log(var='MCP_O2', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=MCP_O2)    ### logging
except Exception:
    CONNECTION_O2 = 0
    vl.log(var='CONNECTION_O2', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=CONNECTION_O2)    ### logging
    write_to_log('O2 failed', str(time.mktime(time.localtime())))

try:
    BMP = BMP180(I2CBUS)
    vl.log(var='BMP', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=BMP)    ### logging
except Exception:
    CONNECTION_BMP = 0
    vl.log(var='CONNECTION_BMP', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=CONNECTION_BMP)    ### logging
    write_to_log('pressure failed', str(time.mktime(time.localtime())))

try:
    AM2301_1 = AM2301(AM2301_1_ADRR)
    vl.log(var='AM2301_1', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=AM2301_1)    ### logging
except Exception:
    CONNECTION_A1 = 0
    vl.log(var='CONNECTION_A1', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=CONNECTION_A1)    ### logging
    write_to_log('AM1 failed', str(time.mktime(time.localtime())))

try:
    AM2301_2 = AM2301(AM2301_2_ADRR)
    vl.log(var='AM2301_2', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=AM2301_2)    ### logging
except Exception:
    CONNECTION_A2 = 0
    vl.log(var='CONNECTION_A2', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=CONNECTION_A2)    ### logging
    write_to_log('AM2 failed', str(time.mktime(time.localtime())))

try:
    AM2301_3 = AM2301(AM2301_3_ADRR)
    vl.log(var='AM2301_3', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=AM2301_3)    ### logging
except Exception:
    CONNECTION_A3 = 0
    vl.log(var='CONNECTION_A3', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=CONNECTION_A3)    ### logging
    write_to_log('AM3 failed', str(time.mktime(time.localtime())))

try:
    AM2301_4 = AM2301(AM2301_4_ADRR)
    vl.log(var='AM2301_4', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=AM2301_4)    ### logging
except Exception:
    CONNECTION_A4 = 0
    vl.log(var='CONNECTION_A4', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=CONNECTION_A4)    ### logging
    write_to_log('AM4 failed', str(time.mktime(time.localtime())))


# Thresshold limits
THRESHOLD_LIMITS = ((0.0, 3000.0), (0.0, 20.0), (0, 23.0), (950.0, 1040.0),
                    (18.0, 30.0, 0.0, 100.0))
vl.log(var='THRESHOLD_LIMITS', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=THRESHOLD_LIMITS)    ### logging

# connectionvaribles for each sensor
CONNECTION_VAR = [CONNECTION_CO2, CONNECTION_CO,
                  CONNECTION_O2, CONNECTION_BMP,
                  CONNECTION_A1, CONNECTION_A2,
                  CONNECTION_A3, CONNECTION_A4]
vl.log(var='CONNECTION_VAR', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=CONNECTION_VAR)    ### logging

SENSORS_LIST = ['CO2', 'CO', 'O2', 'BMP', 'AM1', 'AM2', 'AM3', 'AM4']
vl.log(var='SENSORS_LIST', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=SENSORS_LIST)    ### logging

# functions for taking sensor readings
FUNC_VAR = (measure_scd30, measure_co, measure_o2, measure_bmp,
            measure_am1, measure_am2, measure_am3, measure_am4)
vl.log(var='FUNC_VAR', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=FUNC_VAR)    ### logging

# Create Timers
timer0 = Timer(0)
vl.log(var='timer0', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=None)    ### logging
timer1 = Timer(1)
vl.log(var='timer1', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=None)    ### logging

# Set callback for LoRa (recv as IR)
lora.on_recv(cb_lora)

# sensor readings list init
SENSOR_DATA = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
vl.log(var='SENSOR_DATA', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=SENSOR_DATA)    ### logging

# ------------------------ infinite loop execution ----------------------------
msg = ""  # msg init
vl.log(var='msg', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=msg)    ### logging
rcv_msg = []  # rcv_msg init
vl.log(var='rcv_msg', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=None)    ### logging

# initialize timers
# Timer for sending msgs with measurement values + timestamp + crc
timer0.init(period=msg_interval, mode=Timer.ONE_SHOT, callback=cb_30)

# get the start time of the script in seconds wrt the localtime
start_time = time.mktime(time.localtime())
vl.log(var='start_time', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=start_time)    ### logging
retransmit_count = 0
vl.log(var='retransmit_count', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=retransmit_count)    ### logging


gc_start_time = utime.ticks_ms()
gc.collect()
print('gc.collect duration:', utime.ticks_ms()-gc_start_time)

try:
    while True:
        gc.collect()
        # get the current time of the script in seconds wrt the localtime
        current_time = time.mktime(time.localtime())
        vl.log(var='current_time', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=current_time)    ### logging
        SENSOR_STATUS = 0
        vl.log(var='SENSOR_STATUS', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=SENSOR_STATUS)    ### logging
        LIMITS_BROKEN = 0
        vl.log(var='LIMITS_BROKEN', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=LIMITS_BROKEN)    ### logging
        # print('start of the loop:', current_time, SENSOR_STATUS, LIMITS_BROKEN)
        j = 6
        vl.log(var='j', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=j)    ### logging
        for i in range(len(CONNECTION_VAR)):
            # Sensor Data is available & sensor is working
            func_call = FUNC_VAR[i]
            vl.log(var='func_call', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=func_call)    ### logging
            try:
                if i == 0:
                    # SCD30 sensor readings (involves three values)
                    reading_co2 = func_call()
                    vl.log(var='reading_co2', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=reading_co2)    ### logging
                    if not reading_co2[0] == -1:
                        scd_co2, scd_temp, scd_hum = reading_co2
                        vl.log(var='scd_co2', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=scd_co2)    ### logging
                        vl.log(var='scd_temp', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=scd_temp)    ### logging
                        vl.log(var='scd_hum', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=scd_hum)    ### logging
                        if not (THRESHOLD_LIMITS[i][0] <= scd_co2 <= THRESHOLD_LIMITS[i][1]):
                            print('Thresh broken', i)
                            LIMITS_BROKEN = 1
                            vl.log(var='LIMITS_BROKEN', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=LIMITS_BROKEN)    ### logging
                    SENSOR_DATA[0] = round(scd_co2, 2)
                    vl.log(var='SENSOR_DATA', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=SENSOR_DATA)    ### logging
                    SENSOR_DATA[1] = round(scd_temp, 2)
                    vl.log(var='SENSOR_DATA', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=SENSOR_DATA)    ### logging
                    SENSOR_DATA[2] = round(scd_hum, 2)
                    vl.log(var='SENSOR_DATA', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=SENSOR_DATA)    ### logging
                elif 1 <= i <= 3:
                    # MCP3221, BMP180 sensor reading
                    var = func_call()
                    vl.log(var='var', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=var)    ### logging
                    if not (THRESHOLD_LIMITS[i][0] <= var <= THRESHOLD_LIMITS[i][1]):
                        print('Thresh broken', i, var)
                        LIMITS_BROKEN = 1
                        vl.log(var='LIMITS_BROKEN', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=LIMITS_BROKEN)    ### logging
                    SENSOR_DATA[i+2] = round(var, 2)
                    vl.log(var='SENSOR_DATA', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=SENSOR_DATA)
                else:
                    # AM2301 readings(involves 2 values)
                    micropython.schedule(func_call, i)
                    if not (THRESHOLD_LIMITS[4][0] <= am_temp <= THRESHOLD_LIMITS[4][1]):
                        LIMITS_BROKEN = 1
                        vl.log(var='LIMITS_BROKEN', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=LIMITS_BROKEN)    ### logging
                        print('Thresh broken', i)
                    if not (THRESHOLD_LIMITS[4][2] <= am_hum <= THRESHOLD_LIMITS[4][3]):
                        LIMITS_BROKEN = 1
                        vl.log(var='LIMITS_BROKEN', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=LIMITS_BROKEN)    ### logging
                        print('Thresh broken', i)
                    SENSOR_DATA[j] = am_temp
                    vl.log(var='SENSOR_DATA', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=SENSOR_DATA)    ### logging
                    SENSOR_DATA[j+1] = am_hum
                    vl.log(var='SENSOR_DATA', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=SENSOR_DATA)
                    j += 2
                    vl.log(var='j', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=j)    ### logging
                if CONNECTION_VAR[i] == 0:
                    CONNECTION_VAR[i] = 1
                    vl.log(var='CONNECTION_VAR', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=CONNECTION_VAR)    ### logging
            except Exception as e:
                CONNECTION_VAR[i] = 0
                vl.log(var='CONNECTION_VAR', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=CONNECTION_VAR)    ### logging
                write_to_log('failed {}: {}'.format(SENSORS_LIST[i], e),
                            str(current_time))

            if not CONNECTION_VAR[i]:
                # Sensor failed
                if i == 0:
                    SENSOR_STATUS = 2**(i)
                    vl.log(var='SENSOR_STATUS', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=SENSOR_STATUS)    ### logging
                elif 1 <= i <= 3:
                    SENSOR_STATUS += 2**(i)
                    vl.log(var='SENSOR_STATUS', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=SENSOR_STATUS)
                else:
                    SENSOR_STATUS += 2**(i)
                    vl.log(var='SENSOR_STATUS', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=SENSOR_STATUS)
        print('sensor data:', SENSOR_DATA)
        # prepare the packted to be sent
        msg = ustruct.pack(_pkng_frmt, SENSOR_DATA[0], SENSOR_DATA[3],
                        SENSOR_DATA[4], SENSOR_DATA[5], SENSOR_DATA[6],
                        SENSOR_DATA[7], SENSOR_DATA[8], SENSOR_DATA[9],
                        SENSOR_DATA[10], SENSOR_DATA[11], SENSOR_DATA[12],
                        SENSOR_DATA[13], SENSOR_STATUS,
                        LIMITS_BROKEN, 0, SENSORBOARD_ID)  # current Sensorreadings
        vl.log(var='msg', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=msg)    ### logging
        msg += ustruct.pack(">L", current_time)  # add timestamp to the msg
        vl.log(var='msg', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=msg)    ### logging
        msg += ustruct.pack(">L", crc32(0, msg, 62))  # add 32-bit crc to the msg
        vl.log(var='msg', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=msg)    ### logging

        micropython.schedule(lora_rcv_exec, 0)  # process received msgs

        if LIMITS_BROKEN:
            add_to_que(msg, current_time)
            lora.send(msg)  # Sends imidiately if threshold limits are broken.
            print('send immediately')
            lora.recv()
        elif cb_30_done:  # send the messages every 30 seconds
            try:
                add_to_que(msg, current_time)
                lora.send(que[0][0])
                print('cb_30_done:', que[0][0])
                lora.recv()
            except Exception as e:
                write_to_log('callback 30: {}'.format(e), str(current_time))
            start_time = current_time
            vl.log(var='start_time', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=start_time)    ### logging
            timer1.init(period=retx_interval, mode=Timer.PERIODIC, callback=cb_retrans)
            timer0.init(period=msg_interval, mode=Timer.ONE_SHOT, callback=cb_30)

            # randomize the msg interval to avoid continous collision of packets
            if random.random() >= 0.4:
                # select time randomly with steps of 1000ms, because the max on
                # air time is 123ms and 390ms for SF7 and SF9 resp.
                msg_interval = random.randrange(20000, 40000, 1000)
                vl.log(var='msg_interval', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=msg_interval)    ### logging
                # select random time interval with step size of 1 sec
                retx_interval = random.randrange(2000, 10000, 1000)
                vl.log(var='retx_interval', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=retx_interval)

            # reset timer booleans
            cb_30_done = False
            vl.log(var='cb_30_done', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=cb_30_done)
        elif cb_retrans_done:  # retransmit every 5 seconds for piled up packets with no ack
            cb_retrans_done = False
            vl.log(var='cb_retrans_done', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=cb_retrans_done)
            retransmit_count += 1
            vl.log(var='retransmit_count', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=retransmit_count)
            if que != []:
                lora.send(que[0][0])
                print('cb_retrans_done:', que[0][0])
                lora.recv()
            if retransmit_count >= 2:
                timer1.deinit()
                retransmit_count = 0
                vl.log(var='retransmit_count', fun=_fun_name, clas=_cls_name, th=_thread_id)#, val=retransmit_count)
except Exception as e:
    vl.save()
    print('main thread error:', e)  
    #/// log the traceback message
    vl.traceback(e)