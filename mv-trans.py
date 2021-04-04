THRESHOLD = (70, 100, -15, 15,-20,0)
import sensor, image, time
from pyb import UART

sensor.reset()#初始化
sensor.set_pixformat(sensor.RGB565)#彩色
sensor.set_framesize(sensor.QVGA) #像素 320*240
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
#sensor.set_windowing([0,20,80,40])
sensor.skip_frames(time = 2000)     # WARNING: If you use QQVGA it may take seconds
clock = time.clock()                # to process a frame sometimes.
uart = UART(3, 9600)

while(True):
    clock.tick()
    #img1 = sensor.snapshot()
    img = sensor.snapshot().binary([THRESHOLD])
    blobs = img.find_blobs([THRESHOLD])
    line = img.get_regression([(70,100)], robust = False)
    img.draw_line(line.line(), color = 127)
    b = blobs[0]
    img.draw_rectangle(b[0:4]) # rect
    Lm = (b[2]+b[3])/2
    length = 1000/Lm
    if (line):
        print(line)
        img.draw_line(line.line(), color = 127)
        #list = [line.magnitude(),line.theta(),line.rho()]
        uart.write(str(img.width())+str(line.magnitude())+str(line.theta())+str(line.rho())+str(length)'\n')
    else:
        pass