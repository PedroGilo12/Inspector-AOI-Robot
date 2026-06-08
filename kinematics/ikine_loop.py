import serial
import time

inspector = serial.Serial("/dev/ttyUSB0", 115200)

cmd = "G0 X0 Y0 Z131 A-90 F3000\r\n G0 X0 Y0 Z131 A0 F3000\r\n G0 X0 Y0 Z131 A90 F3000\r\n G0 X0 Y0 Z131 A0 F3000\r\n"
for i in range(10):
    inspector.write(cmd.encode("ascii"))
    time.sleep(10)