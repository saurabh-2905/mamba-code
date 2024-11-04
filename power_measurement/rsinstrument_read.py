""""
Find the instruments in your environment
"""

# from RsInstrument import *

# # Use the instr_list string items as resource names in the RsInstrument constructor
# instr_list = RsInstrument.list_resources("?*", "rs")
# print(instr_list)

############# session initialization #######################

from RsInstrument import *
import serial.tools.list_ports

# List available serial ports
print("Available serial ports:")
ports = serial.tools.list_ports.comports()
for port in ports:
    print(f"Port: {port.device} - Description: {port.description}")

# Resource string for your device
resource_string = 'ASRL3::INSTR'
# resource_string = 'USB::0x0403::0xED74::COM3::INSTR'
# resource_string = 'USB::0x3593::0x0445K02::101044::nq'



# Initializing the session
try:
    # Create an instance of RsInstrument with the correct resource string
    instr = RsInstrument(resource_string)

    # # Set a longer timeout if necessary
    # instr.timeout = 20000  # 10 seconds

    # Query the instrument's identification
    idn = instr.query_str('ID')
    print(f"\nHello, I am: '{idn}'")
    # print(f'RsInstrument driver version: {instr.driver_version}')
    # print(f'Visa manufacturer: {instr.visa_manufacturer}')
    # print(f'Instrument full name: {instr.full_instrument_model_name}')
    # print(f'Instrument installed options: {",".join(instr.instrument_options)}')

except Exception as e:
    print("Error communicating with the instrument:", e)

finally:
    # Close the session
    instr.close()
