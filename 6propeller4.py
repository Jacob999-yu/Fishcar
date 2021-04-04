#!/usr/bin/python3
#-*- coding: utf-8 -*-
import RPi.GPIO as GPIO
import time as ttime
import threading
import serial
#from pid import PID
from math import pi, isnan

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(11,GPIO.OUT)#L1
GPIO.setup(22,GPIO.OUT)#L2
GPIO.setup(15,GPIO.OUT)#L3
GPIO.setup(12,GPIO.OUT)#R1
GPIO.setup(16,GPIO.OUT)#R2
GPIO.setup(18,GPIO.OUT)#R3
#PropellerVelocity = [1.f] * 6

def process1(listmv):
    return 10
def process(listmv):
    # feedback = ser.readline()
    # list = [float(s) for s in feedback.decode().strip().split(',')]
    rho_err = abs(listmv[2])-156
    if listmv[2]>90:
        theta_err = listmv[1]-180
    else:
        theta_err = listmv[1]
        
    if listmv[0]<4:
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



class PID:
    _kp = _ki = _kd = _integrator = _imax = 0
    _last_error = _last_derivative = _last_t = 0
    _RC = 1/(2 * pi * 20)
    # def __init__(self, p=0, i=0, d=0, imax=0):
    def __init__(self, p=0.4,i=0.08, d=0.1, imax=20):
        self._kp = float(p)
        self._ki = float(i)
        self._kd = float(d)
        self._imax = abs(imax)
        self._last_derivative = float('nan')
    def reset_I(self):
        self._integrator = 0
        self._last_derivative = float('nan')  
    def get_pid(self, error, scaler):
        tnow = ttime.time()
        dt = tnow - self._last_t
        output = 0
        if self._last_t == 0 or dt > 1000:
            dt = 0
            self.reset_I()
        self._last_t = tnow
        delta_time = float(dt) / float(1000)
        output += error * self._kp
        if abs(self._kd) > 0 and dt > 0:
            if isnan(self._last_derivative):
                derivative = 0
                self._last_derivative = 0
            else:
                derivative = (error - self._last_error) / delta_time
            derivative = self._last_derivative + \
                                     ((delta_time / (self._RC + delta_time)) * \
                                        (derivative - self._last_derivative))
            self._last_error = error
            self._last_derivative = derivative
            output += self._kd * derivative
        output *= scaler
        if abs(self._ki) > 0 and dt > 0:
            self._integrator += (error * self._ki) * scaler * delta_time
            if self._integrator < -self._imax: self._integrator = -self._imax
            elif self._integrator > self._imax: self._integrator = self._imax
            output += self._integrator
        return output  



rho_pid = PID(p=0.4, i=0)
theta_pid = PID(p=0.001, i=0)


class PropellerThread(threading.Thread):
    def __init__(self, id, pin,nowDutyCycle):
        threading.Thread.__init__(self)
        self.pwm = GPIO.PWM(pin, 50)
        self.id = id
        self.nowDutyCycle = nowDutyCycle
        
    def run(self):
        self.pwm.start(0)
        self.pwm.ChangeDutyCycle(5)
        ttime.sleep(1)
        self.pwm.ChangeDutyCycle(7)
        ttime.sleep(1)
        self.pwm.ChangeDutyCycle(self.nowDutyCycle)
        ttime.sleep(3)
        while True:
            #ser = serial.Serial("/dev/ttyAMA0",9600)
            #feedback = ser.readline()
            #listmv = [float(s) for s in feedback.decode().strip().split(',')]
            listmv = [1,132,5]
            if listmv == False:
                listmv = [1,132,5]
            if len(listmv) != 3:
                listmv = [1,132,5]
            velocity = process1(listmv)
            #height = list0[4]
            if self.id == 1:
                self.nowDutyCycle = self.nowDutyCycle + 0.05*velocity
                if self.nowDutyCycle > 7.5:
                    self.nowDutyCycle = 7.5
                if self.nowDutyCycle < 3.8:
                    self.nowDutyCycle = 3.8
                self.pwm.ChangeDutyCycle(self.nowDutyCycle) 
            if self.id == 2:
                self.nowDutyCycle = self.nowDutyCycle + 0.05*velocity
                if self.nowDutyCycle > 7.5:
                    self.nowDutyCycle = 7.5
                if self.nowDutyCycle < 3.8:
                    self.nowDutyCycle = 3.8
                self.pwm.ChangeDutyCycle(self.nowDutyCycle)
            if self.id == 3:
                self.nowDutyCycle = self.nowDutyCycle - 0.05*velocity
                if self.nowDutyCycle > 7.5:
                    self.nowDutyCycle = 7.5
                if self.nowDutyCycle < 3.8:
                    self.nowDutyCycle = 3.8
                self.pwm.ChangeDutyCycle(self.nowDutyCycle)
            if self.id == 4:
                self.nowDutyCycle = self.nowDutyCycle - 0.05*velocity
                if self.nowDutyCycle > 7.5:
                    self.nowDutyCycle = 7.5
                if self.nowDutyCycle < 3.8:
                    self.nowDutyCycle = 3.8
                self.pwm.ChangeDutyCycle(self.nowDutyCycle)
            '''
            if height > 5.5 :
                if self.id == 5:
                    self.nowDutyCycle = self.nowDutyCycle - 3
                    if self.nowDutyCycle > 7.5:
                        self.nowDutyCycle = 7.5
                    if self.nowDutyCycle < 3.8:
                        self.nowDutyCycle = 3.8
                    self.pwm.ChangeDutyCycle(self.nowDutyCycle)   #不知道减多少，暂定减5吧
                if self.id == 6:
                    self.nowDutyCycle = self.nowDutyCycle - 3.2
                    if self.nowDutyCycle > 7.5:
                        self.nowDutyCycle = 7.5
                    if self.nowDutyCycle < 3.8:
                        self.nowDutyCycle = 3.8
                    self.pwm.ChangeDutyCycle(self.nowDutyCycle)   #两个电机可能减少的不同
            else:
                if self.id == 5:
                    self.nowDutyCycle = self.nowDutyCycle + 3  
                    if self.nowDutyCycle > 7.5:
                        self.nowDutyCycle = 7.5
                    if self.nowDutyCycle < 3.8:
                        self.nowDutyCycle = 3.8
                    self.pwm.ChangeDutyCycle(self.nowDutyCycle)
                if self.id == 6:
                    self.nowDutyCycle = self.nowDutyCycle + 3.2
                    if self.nowDutyCycle > 7.5:
                        self.nowDutyCycle = 7.5
                    if self.nowDutyCycle < 3.8:
                        self.nowDutyCycle = 3.8
                    self.pwm.ChangeDutyCycle(self.nowDutyCycle)
                    '''
            ttime.sleep(0.5)

            
         #   self.pwm.ChangeDutyCycle(PropellerVelocity[self.id])


# threadLock = threading.Lock()

L1 = PropellerThread(1,11,6)
L2 = PropellerThread(2,22,6)
R1 = PropellerThread(3,15,6)
R2 = PropellerThread(4,12,6)
M1 = PropellerThread(5,16,6)
M2 = PropellerThread(6,18,6)


try:

    L1.start()
    L2.start()
    R1.start()
    R2.start()
    M1.start()
    M2.start()
except KeyboardInterrupt:
    GPIO.cleanup()
