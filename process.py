#!/usr/bin/python3
import time
import serial
from pid import PID

ser = serial.Serial("/dev/ttyAMA0",9600)
rho_pid = PID(p=0.4, i=0)
theta_pid = PID(p=0.001, i=0)

def PROCESS:
    feedback = ser.readline()
    list = [float(s) for s in feedback.decode().strip().split(',')]
    rho_err = abs(list[4])-list[0]/2
    if list[2]>90:
        theta_err = list[2]-180
    else:
        theta_err = list[2]
        
    if list[1]<4:
        #if -40<b_err<40 and -30<t_err<30:
        rho_output = rho_pid.get_pid(rho_err,1)
        theta_output = theta_pid.get_pid(theta_err,1)
        output = rho_output+theta_output
        ser.flushInput()
        return output
            # uart.write(str(output)+'\n')
            #car.run(50+output, 50-output)
    else:
        ser.flushInput()
        return 0.01
            # uart.write(str(50)+'\n')
            #car.run(0,0)
    #  else:
    #      pass