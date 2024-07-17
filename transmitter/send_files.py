import os

# files_onbard = os.listdir()


def read_file(file_name):
    with open(file_name, 'r') as file:
        return file.read()

def send_file_over_serial(file_name):
    content = read_file(file_name)
    print(content)  # Send file content over UART (serial)

send_file_over_serial('filename.txt')