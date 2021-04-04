#!/usr/bin/python3

import RPi.GPIO as GPIO
import time 

GPIO.setmode(GPIO.BOARD)
GPIO.setup(18,GPIO.OUT)

p = GPIO.PWM(18,10)
p.start(0)

try:
    while 1:
        for i in range(0,100,5):
            p.ChangeDutyCycle(i)
            time.sleep(0.05)
except KeyboardInterrupt:
    p.ChangeDutyCycle(0)
    GPIO.cleanup()




