import serial

serial = serial.Serial("/dev/ttyACM0", 115200)

while True:
    line = serial.readline()
    print(line.decode("utf-8").strip())
