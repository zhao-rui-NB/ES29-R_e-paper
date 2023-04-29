import serial
from img import *
import time
cmd_write = 'A0'
cmd_update = 'A1'

ser = serial.Serial('com5', baudrate=115200, bytesize=8, parity='N', stopbits=1)

str = 'FF FF FF FF A1 00F0 FFFFFFFF 00000000 FFFFFFFF 00000000\r\n'

data = ''
UID = 'FFFFFFFF'
func = cmd_write
reg_addr = 0
ti = time.time()
for i in range(296):
    data = ''
    reg_addr = '{:04x}'.format(16*i+0x8000)
    for j in range(16):
        data = data + '{:02x}'.format(  0xFF^img[i*16+j])
    print(UID + func + reg_addr + data)
    packet = (UID + func + reg_addr + data + '\r\n').encode('utf-8')
    ser.write(packet)
    
    while True:
        line = ser.readline().decode()
        print(line)
        if 'ok' in line:
            break


packet = 'FF FF FF FF A1\r\n'.encode('utf-8')
ser.write(packet)




ser.close()

print(time.time() - ti)