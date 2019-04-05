import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
'''
servoPIN = 4
GPIO.setup(servoPIN, GPIO.OUT)
pwmL = GPIO.PWM(servoPIN, 50)
pwmL.start(0)

servoPIN = 18
GPIO.setup(servoPIN, GPIO.OUT)
pwmM = GPIO.PWM(servoPIN, 50)
pwmM.start(0)

servoPIN = 16
GPIO.setup(servoPIN, GPIO.OUT)
pwmRB = GPIO.PWM(servoPIN, 50)
pwmRB.start(0)

servoPIN = 21
GPIO.setup(servoPIN, GPIO.OUT)
pwmLB = GPIO.PWM(servoPIN, 50)
pwmLB.start(0)

'''
servoPIN = 21
GPIO.setup(servoPIN, GPIO.OUT)
pwmGripper = GPIO.PWM(servoPIN, 50)
pwmGripper.start(0)

servoPIN = 4
GPIO.setup(servoPIN, GPIO.OUT)
pwmT = GPIO.PWM(servoPIN, 50)
pwmT.start(0)


while True:
    #maxGripper 2.5 --> min for bottle 9.5
    #maxT 17 --> 
    pwmT.ChangeDutyCycle(9)
    time.sleep(0.75)
    pwmGripper.ChangeDutyCycle(2.5)
    time.sleep(0.75)
    pwmT.ChangeDutyCycle(5)
    time.sleep(0.75)
    pwmGripper.ChangeDutyCycle(10)
    time.sleep(0.75)

