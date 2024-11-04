import serial
import time
import csv

# Replace 'COM3' with the appropriate port name for your device
# '/dev/ttyUSB0' for Linux, '/dev/ttyS0' for older serial ports on Linux, 'COM3' for Windows
ser = serial.Serial('COM4', baudrate=9600, timeout=1)
# Open the serial port
if ser.is_open:
    print(f"Connected to {ser.name}")

current_list = []

 # # Prepare the command
command_mixed_mode = "MX1" + chr(0x0D)  # Add carriage return (0x0D) at the end of the command
# command_mixed_mode = "RM0" + chr(0x0D)  # Add carriage return (0x0D) at the end of the command
# # Send the command
ser.write(command_mixed_mode.encode('utf-8'))
print(f"Sent command: {command_mixed_mode}")
        

try:
    while True:
        # Prepare the command
        start_time = time.time_ns()
        command_current_read = "MI2" + chr(0x0D)  # Add carriage return (0x0D) at the end of the command
        # Send the command
        ser.write(command_current_read.encode('utf-8'))
        print(f"Sent command: {command_current_read}")

        # Optionally, read the response
        time.sleep(0.1)  # Wait a moment for the device to process and respond
        response = ser.readline().decode('utf-8').strip()
        print('time per reading:', (time.time_ns() - start_time)/10**6)
        current_list += [response]
        print(f"Current: {response}")


except KeyboardInterrupt:
    print('Writing Current Readings to file')

    # Open a CSV file to save the received data
    with open("current_readings.csv", "w", newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        
        # Optional: Write a header row
        csvwriter.writerow(["Current (A)"])

        for current_raw in current_list:
            # print(current_raw[4:-1], type(current_raw))
            current_val = float(current_raw[4:-1])
            current_unit = current_raw[-1]

            # print(current_val, current_unit)
            csvwriter.writerow([current_val])

    print("Stopped by User")

finally:
    # Close the serial port when done
    # Prepare the command
    command_current_read = "RM1" + chr(0x0D)  # Add carriage return (0x0D) at the end of the command
    # Send the command
    ser.write(command_current_read.encode('utf-8'))
    print(f"Sent command: {command_current_read}")

    ser.close()
    print("Serial port closed.")
