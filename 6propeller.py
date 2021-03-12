#!/usr/bin/python3

import RPi.GPIO as GPIO
import time
from threading import Thread

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(11,GPIO.OUT)#L1
GPIO.setup(13,GPIO.OUT)#L2
GPIO.setup(15,GPIO.OUT)#L3
GPIO.setup(12,GPIO.OUT)#R1
GPIO.setup(16,GPIO.OUT)#R2
GPIO.setup(18,GPIO.OUT)#R3


#PropellerVelocity = [1.f] * 6


class PropellerThread(Thread):
    def __init__(self, id, pin):
        self.pwm = GPIO.PWM(pin, 50)
        self.id = id
        
    def run(self):
        self.pwm.start(0)
        self.pwm.ChangeDutyCycle(5)
        time.sleep(1)
        self.pwm.ChangeDutyCycle(7)
        time.sleep(1)
        self.pwm.ChangeDutyCycle(6)
        time.sleep(3)
        #while True:
         #   self.pwm.ChangeDutyCycle(PropellerVelocity[self.id])

L1 = PropellerThread(1,11)
L2 = PropellerThread(2,13)
L3 = PropellerThread(3,15)
R1 = PropellerThread(4,12)
R2 = PropellerThread(5,16)
R3 = PropellerThread(6,18)

L1.start()
L2.start()
L3.start()
R1.start()
R2.start()
R3.start()