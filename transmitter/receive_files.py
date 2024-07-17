import serial

mamba_board_5 = '/dev/tty.usbserial-D309CQ8U'

ser = serial.Serial(mamba_board_5, 115200)
while True:
    line = ser.readline()
    if line:
        print(line.decode('utf-8').strip())