import serial
import time
def recevie(a,b):
	t=serial.Serial("/dev/ttyUSB0",115200);
    return t.read(3)
	
