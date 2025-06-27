# Mamba LoRa testbench with Varlogger
This repository contains the LoRa based sensor system to test and debug issues. It can read data from multiple sensors and sends it to a receiver. And incase of any discrepencies, it will log the error and tries to resend the data. The objective is to recreate real-world problems faced in earlier deployments and improve system reliability through controlled testing.

## 📁 Project Structure
```bash
├── receivers/
│   └── main.py        
├── transmitters/
│   └── main.py       
│   ├── lib/
│   │   └── varlogger.py    # varlogger library
│   │   └── lora.py 
└── README.md
```

## Requirements
1. Pycom board
2. LoRa compatible transceiver.
3. Flashing tools like Pymakr or rshell

## Setup
1. Flash main.py from both receiver and transmitter to corresponding Pycom boards.
2. Ensure sensors are wired correctly.
3. Power on device.
4. Use a serial monitor to observe Data transmission, ACK status and error logs.

## Logging and debugging
1. All logs are written to log.txt with timestamps.
