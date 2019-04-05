### lib for CREATE2
from  pycreate2 import Create2
import time
import serial

### lib for Async func
import asyncio
import time
 
### lib for RPi-GPIO
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# servo for gripper
servoPIN = 21
GPIO.setup(servoPIN, GPIO.OUT)
pwmGripper = GPIO.PWM(servoPIN, 50)
pwmGripper.start(0)

# servo for topJoint
servoPIN = 4
GPIO.setup(servoPIN, GPIO.OUT)
pwmT = GPIO.PWM(servoPIN, 50)
pwmT.start(0)

def grabBottle():
    pwmT.ChangeDutyCycle(5)
    time.sleep(0.75)
    pwmGripper.ChangeDutyCycle(10)
    time.sleep(0.75)
    pwmT.ChangeDutyCycle(8.5)
    time.sleep(0.75)
    
def grabShirt():
    pwmT.ChangeDutyCycle(4)
    time.sleep(0.75)
    pwmGripper.ChangeDutyCycle(12.5)
    time.sleep(0.75)
    pwmT.ChangeDutyCycle(8.5)
    time.sleep(0.75)
    
def releaseBottle():
    pwmGripper.ChangeDutyCycle(2.5)
    time.sleep(0.75)
   
def driveDst(dstInput):
    if dstInput > 0:
        bot.drive_direct(400, 400)
    else:
        bot.drive_direct(-400, -400)
    moveTime = dstInput / 40
    time.sleep(moveTime)
    print()
    
def turn90():
    try:
        bot.turn_angle(45, 200)
    except Exception:
        pass
     
def turn_90():
    try:
        bot.turn_angle(45, -200)
    except Exception:
        pass
    
# setup
bot = Create2('/dev/ttyUSB0', 115200)
connection = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)
bot.start()
bot.safe()
connection.write(b'\x8c\x03\x01@\x16\x8d\x03') #beep!

def main():
    global x, y, w, h, dst
    while True:
        pwmT.ChangeDutyCycle(8.5)
        time.sleep(0.75)
        pwmGripper.ChangeDutyCycle(2.5)
        time.sleep(0.75)
        connection.write(b'\x90\x00\x00\x7f')
        
        colorDetect()
        print('%d  %d  %d  %d  %d' % (x, y, w, h, dst))
        
        driveDst(52)
        grabBottle()
        driveDst(-35)
        turn_90()
        driveDst(115)
        turn_90()
        driveDst(60)
        releaseBottle()
        
        driveDst(-180+137-35-15)
        turn90()
        turn90()
        grabShirt()
        driveDst(17)
        turn90()
        driveDst(65+50+80+50)
        turn90()
        driveDst(180-137+35+15+17)
        releaseBottle()
        turn_90()
        driveDst(30)
        turn_90()
        driveDst(103)
        grabShirt()
        turn_90()
        driveDst(30)
        turn_90()
        driveDst(103)
        releaseBottle()
        turn_90()
        driveDst(30)
        turn_90()
        driveDst(103+65)
        turn_90()
        driveDst(80)
        turn90()
        driveDst(65+65+65)
        turn_90()
        driveDst(54)
        grabShirt()
        turn_90()
        driveDst(65+65+65+65+72+8+57)
        turn_90()
        driveDst(54+16)
        releaseBottle()
        turn_90()
        driveDst(65+65+65+65+72+8+57)
        turn_90()
        driveDst(80+50+65+10)
        grabBottle()
        driveDst(0-(80+50+65+10))
        turn_90
        driveDst(65+65+65+65+72+8+57)
        turn90()
        driveDst(80+50+65+10+27)
        releaseBottle()
        driveDst(0-(80+50+65+10+27))
        turn90()
        driveDst(65+65+65+65+72+8+57)
        turn90()
        driveDst(57)
        grabBottle()
        driveDst(-57)
        turn90()
        driveDst(65+65+65+65+72+8+57)
        turn90()
        driveDst(80+50+65+10+27)
        releaseBottle(85)
        
        connection.write(b'\x8c\x03\x01@\x16\x8d\x03')
        time.sleep(1)
        connection.write(b'\x8c\x03\x01@\x16\x8d\x03')
        time.sleep(1)
        
        break

main()
