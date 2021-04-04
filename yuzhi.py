# Untitled - By: jacob - 周六 4月 3 2021

import sensor, image, time

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)

clock = time.clock()

while(True):
    clock.tick()
    img = sensor.snapshot()
    print(clock.fps())
#4:(67, 100, -23, 114, -63, 114)----
#THRESHOLD = (64, 100, -38, 6,-7,15)
#3:(60, 100, -38, 30, 8, 71)
#1:(53, 100, -25, 30, -6, 47);
#1:(53, 100, -25, 30, -4, 47);
#final:(53, 100, -25, 30, -5, 47)
