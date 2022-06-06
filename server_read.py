import serial

serial = serial.Serial("/dev/ttyACM0", 115200)  # Port and Baud rate

while True:
    line = serial.readline()  # Blocks until EOL is received
    print(line.decode("utf-8"))
