
import RPi.GPIO as GPIO      
import time
import serial
GPIO.setmode(GPIO.BOARD) 


# Enable Serial Communication
port = serial.Serial("/dev/ttyTHS1", baudrate=115200, timeout=1)

port.write("AT+CMGF=1\r\n".encode())
time.sleep(1)
port.write("AT+CSMP?\r\n".encode())
time.sleep(1)
port.write('AT+CMGS="+79221789077"\r\n'.encode())
time.sleep(1)
port.write("Hello\r\n".encode())
time.sleep(1)
port.write("\x1A".encode())

rcv = port.read(10)
print(rcv)
time.sleep(1)



