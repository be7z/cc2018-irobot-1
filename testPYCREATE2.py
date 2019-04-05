### lib for CREATE2
from  pycreate2 import Create2
import time
import serial

import asyncio
# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
 
### lib for RPi-GPIO
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (416, 416)
#camera.rotation = 180
camera.framerate = 25
rawCapture = PiRGBArray(camera, size=(416, 416))
# allow the camera to warmup
time.sleep(0.1)
# define range of blue color in HSV
lower_blue = np.array([45,50,50])
upper_blue = np.array([75,255,255])
x, y, w, h, dst = 0, 0, 0, 0, 0
# motor
servoPIN = 21
GPIO.setup(servoPIN, GPIO.OUT)
pwmGripper = GPIO.PWM(servoPIN, 50)
pwmGripper.start(0)

servoPIN = 4
GPIO.setup(servoPIN, GPIO.OUT)
pwmT = GPIO.PWM(servoPIN, 50)
pwmT.start(0)

# capture frames from the camera
def colorDetect():
    global x, y, w, h, dst
    camera.capture('ColorDetectIn.jpg')
    img = cv2.imread('ColorDetectIn.jpg')
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # Threshold the HSV image to get only green colors
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    x, y, w, h = 0, 0, 0, 0
    area = 0
    dst = 0
    for c in contours:
        rect = cv2.boundingRect(c)
        tmp_x,tmp_y,tmp_w,tmp_h = rect
        area = max(area, tmp_w * tmp_h)
        if area == tmp_w * tmp_h:
            x, y, w, h = rect
            epsilon = 0.08 * cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, epsilon, True)
    
    # calc distance ==> 40 * 60 = 2400 px @max=72cm
    if min(h, w) > 17:
        dst = 506.67*6/min(h, w)
    else:
        dst = 0
        
    if area > 120:
        # draw ocv-rect box
        '''try:
            if y >= int(h/21*3):
                crop = img[int(y-h/21*3):min(y+h+5, 416), max(x-5, 0):min(x+w+5, 416)]
                cv2.imwrite('predictInput.jpg', crop)
                #detectReq('predictInput.jpg')
                #motionChange()
        except Exception:
                pass
        '''
        cv2.drawContours(img, [approx], -1, (0, 0, 255), 2)
        cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.putText(img, '%.2f cm' % (dst),
            (img.shape[1] - 200, img.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX,
            2.0, (0, 255, 0), 3)
    gry = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.imshow('Frame', gry)
    cv2.imwrite('colorDetectOut.jpg', img)
        
    # if the `q` key was pressed, break from the loop
    key = cv2.waitKey(1) & 0xFF
    # clear the stream in preparation for the next frame
    #rawCapture.truncate(0)
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
bot=Create2('/dev/ttyUSB0', 115200)
connection = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)
bot.start()
bot.safe()
connection.write(b'\x8c\x03\x01@\x16\x8d\x03')

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
