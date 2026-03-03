import serial
import time
import tkinter as tk


#a=('0xF2')
#print(a)
#bytes.fromhex(a)

a=b'\x02\x43\xB0\x01\x03\xF2'

ser = serial.Serial('COM2', 9600)

while True:

    ser.write(a)
    data = ser.read(6)
    #print(data)
    b = str(hex(data[2]))
    c = str(hex(data[3]))
    #print(b, c)
    if (len(c)>3):
        d = (b+c[2]+c[3])
        #print(d)
        val = (int(d, 16))
        #print(len(b))
        if (len(str(val))>4):
            print('out if range')
        else:
            print(val)

    time.sleep(0.1)




