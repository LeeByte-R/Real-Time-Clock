import serial
import time

uart = serial.Serial('COM8', 9600, timeout=2000)

bstring = bytes(time.strftime("%Y-%m-%d-%w-%H-%M-%S", time.localtime())+"\0", 'ascii')
uart.write(bstring)
print(bstring)
uart.close()