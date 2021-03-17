#THRESHOLD = (5, 70, -23, 15, -57, 0) # Grayscale threshold for dark things...
#THRESHOLD = (70, 100, -15, 15, -57, 0)
THRESHOLD = (70, 100, -15, 15,-20,0)
import sensor, image, time
from pyb import LED
#import car
#from pid import PID
#rho_pid = PID(p=0.4, i=0)
#theta_pid = PID(p=0.001, i=0)

#LED(1).on()
#LED(2).on()
#LED(3).on()

sensor.reset()#chu shi hua
#sensor.set_vflip(True)#fanzhuan
#sensor.set_hmirror(True)
sensor.set_pixformat(sensor.RGB565)#cai she
sensor.set_framesize(sensor.QVGA) #xiangsu 320*240
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
#sensor.set_windowing([0,20,80,40])
#sensor.set_windowing([0,20,80,40])
sensor.skip_frames(time = 2000)     # WARNING: If you use QQVGA it may take seconds
clock = time.clock()                # to process a frame sometimes.

while(True):
    clock.tick()
    img1 = sensor.snapshot()
    print(img1)
    img2 = img1.binary([THRESHOLD])
    print(img2)
    #input()
    line = img2.get_regression([(70,100)], robust = False)
    #line = img.get_regression([(100,100,0,0,0,0)], robust = True)
    #sensor.skip_frames(time = 2000)
    print(line)
    #input()
    if (line):
        rho_err = abs(line.rho())-img2.width()/2
        if line.theta()>90:
            theta_err = line.theta()-180
        else:
            theta_err = line.theta()j
        #img.draw_line(line.line(), color = 127)
        print(rho_err,line.magnitude(),rho_err)
        if line.magnitude()>8:
            print(2)
            #if -40<b_err<40 and -30<t_err<30:
            #rho_output = rho_pid.get_pid(rho_err,1)
            #theta_output = theta_pid.get_pid(theta_err,1)
            #output = rho_output+theta_output
            #car.run(50+output, 50-output)
        else:
            print(0)
            #car.run(0,0)
    else:
        print(5)
        #car.run(50,-50)
        pass
    #print(clock.fps())
