import time
from pyb import UART

uart = UART(3, 115200)

while(True):
    uart.write('1234')
    time.sleep(1000)//延时
    if uart.any():
        a=uart.readline().decode()
      //https://singtown.com/learn/50240/
        print(a)  

