import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

servoPIN = 21
GPIO.setup(servoPIN, GPIO.OUT)
pwmGripper = GPIO.PWM(servoPIN, 50)
pwmGripper.start(0)

servoPIN = 4
GPIO.setup(servoPIN, GPIO.OUT)
pwmT = GPIO.PWM(servoPIN, 50)
pwmT.start(0)


while True:
    # maxGripper 2.5 --> min for bottle 9.5
    # maxTopJoint 17 for 100PMW
    pwmT.ChangeDutyCycle(9)
    time.sleep(0.75)
    pwmGripper.ChangeDutyCycle(2.5)
    time.sleep(0.75)
    pwmT.ChangeDutyCycle(5)
    time.sleep(0.75)
    pwmGripper.ChangeDutyCycle(10)
    time.sleep(0.75)
