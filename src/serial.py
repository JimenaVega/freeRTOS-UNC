
import numpy as np
import matplotlib.pyplot as plt
import time
import serial

ser = serial.Serial(
	port='/dev/ttyUSB1',		#Configurar con el puerto a usar 
	baudrate=115200,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS
)
 
ser.isOpen()
ser.timeout=None
print(ser.timeout)