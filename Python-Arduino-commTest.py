import serial
import time


serialComm=serial.Serial(port='COM5', baudrate=115200, bytesize=8, timeout=2)

while True:
    i=input('Input angle of rotation: ')
    

    if i=='end':
        break
    
    i=int(i)
    
    serialComm.write(i)
    print(serialComm.readline())
    
    # print(serialComm.readline().decode('ascii'))
    # print(serialComm.readline().decode('ascii'))
    # print(serialComm.readline().decode('ascii'))

serialComm.close()

