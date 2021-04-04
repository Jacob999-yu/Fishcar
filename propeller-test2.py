#!/usr/bin/python3

import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(18,GPIO.OUT)
p = GPIO.PWM(18,50)
p.start(0)

try:
    dc = 5
    p.ChangeFrequency(dc)
    time.sleep(1)
    
    dc = 7
    p.ChangeFrequency(dc)
    time.sleep(1)

    dc = 6
    p.ChangeFrequency(dc)
    time.sleep(3)

except KeyboardInterrupt:
    p.stop
    GPIO.cleanup()