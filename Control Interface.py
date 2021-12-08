import serial
import math

serialPort=serial.Serial(port='COM5', baudrate=115200, bytesize=8, timeout=2)


#constant link 1 and 2 lengths
L1=200
L2=200

while True:
    InvorFor=input("Would you like to use Forward or Inverse Kinetmatics (Forward/Inverse): ")

    if InvorFor=="Forward":
    
        theta1F=input("Input the angle Theta 1: ")
        theta2F=input("Input the angle Theta 2: ")

        xCoor = round(L1 * math.cos(theta1F) + L2 * math.cos(theta1F + theta2F))
        yCoor = round(L1 * math.sin(theta1F) + L2 * math.sin(theta1F + theta2F))

        print(xCoor)
        print(yCoor)
    



