
import RPi.GPIO as GPIO      
import time
import serial
GPIO.setmode(GPIO.BOARD) 


# Enable Serial Communication
port = serial.Serial("/dev/ttyTHS1", baudrate=9600, timeout=1)


rcv = port.read(10)
print(rcv)
time.sleep(1)



