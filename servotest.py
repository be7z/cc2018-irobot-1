import RPi.GPIO as GPIO
import time

servoPIN = 4
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN,GPIO.OUT)

p = GPIO.PWM(servoPIN,50)
p.start(7.5)

try:
    while False:
        print('Center')
        p.ChangeDutyCycle(7.5)
        time.sleep(2)
        print('+90')
        p.ChangeDutyCycle(2.5)
        time.sleep(2)
        print('Center')
        p.ChangeDutyCycle(7.5)
        time.sleep(2)
        print('-90')
        p.ChangeDutyCycle(12.5)
        time.sleep(2)
    while True:
        for i in range(20, 70, 5):
            print(i/10.0)
            p.ChangeDutyCycle(i/10.0)
            time.sleep(1)
            if i > 50:
                time.sleep(4)
except KeyboardInterrupt:
    p.stop()
    GPIO.cleanup()
