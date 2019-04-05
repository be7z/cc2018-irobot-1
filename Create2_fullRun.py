# -------------------- library ------------------------#

# lib for CREATE-2
import sys
import glob  # for listing serial ports
import struct
try:
    import serial
except ImportError:
    print('Import error', 'Please install pyserial.')
    raise

# lib for CameraMod
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np

# lib for YOLOv3
from subprocess import Popen, PIPE
import threading
import os
import fcntl

# lib for async
import asyncio

# lib for CREATE2
from pycreate2 import Create2
import time

# lib for RPi-GPIO
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
# ------------------- globla var ----------------------#

# CREATE2 connect'n
bot = Create2('/dev/ttyUSB0', 115200)
connection = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)

# camera mod
camera = PiCamera()
camera.resolution = (416, 416)
#camera.rotation = 180
#camera.framerate = 60
rawCapture = PiRGBArray(camera, size=(416, 416))

# location of gr-bott
x, y, w, h, dst = 0, 0, 0, 0, 0
thisIsGRBTT = False
# YOLOv3 ready check (Loading, Ready, Reqst)
yoloStat = 'Ready'
yoloReqIMG = ''

# range of green color in HSV
lower_blue = np.array([45, 50, 50])
upper_blue = np.array([75, 255, 255])


# motor
servoPIN = 21
GPIO.setup(servoPIN, GPIO.OUT)
pwmGripper = GPIO.PWM(servoPIN, 50)
pwmGripper.start(0)

servoPIN = 4
GPIO.setup(servoPIN, GPIO.OUT)
pwmT = GPIO.PWM(servoPIN, 50)
pwmT.start(0)

# -------------------- function ----------------------#

# detect color from the camera


def colorDetect(imgFrame):
    global x, y, w, h, dst
    img = cv2.imread(imgFrame)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Threshold the HSV image to get only green colors
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    contours, hierarchy = cv2.findContours(
        mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    x, y, w, h = 0, 0, 0, 0
    area = 0
    dst = 0
    for c in contours:
        rect = cv2.boundingRect(c)
        tmp_x, tmp_y, tmp_w, tmp_h = rect
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

    if area > 100:
        # draw ocv-rect box
        try:
            if y >= int(h/21*3):
                crop = img[int(y-h/21*3):min(y+h+5, 416),
                           max(x-5, 0):min(x+w+5, 416)]
                cv2.imwrite('predictInput.jpg', crop)
                thisIsGRBTT = True
                #detectReq('predictInput.jpg')
        except Exception:
            pass

        cv2.drawContours(img, [approx], -1, (0, 0, 255), 2)
        cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.putText(img, '%.2f cm' % (dst),
                    (img.shape[1] - 200, img.shape[0] -
                     20), cv2.FONT_HERSHEY_SIMPLEX,
                    2.0, (0, 255, 0), 3)
    gry = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.imwrite('colorDetectOut.jpg', img)
    cv2.imshow('outFrame', gry)

    # if the `q` key was pressed, break from the loop
    key = cv2.waitKey(1) & 0xFF
    # clear the stream in preparation for the next frame
    # rawCapture.truncate(0)


def detectReq(inputName):
    global yoloStat, yoloReqIMG, thisIsGRBTT
    yoloReqIMG = '../Desktop/' + inputName
    yoloStat = 'Reqst'
    thisIsGRBTT = False


def motionChange():
    global x, y, w, h, dst
    moveDist = 10 #dst - 25
    moveTime = 1  #moveDist / 10
    if dst < 35:
        moveDist = dst - 25
        moveTime = moveDist / 10
        bot.drive_direct(100, 90)
        time.sleep(moveTime)
    if dst > 25 * 1.1:
        bot.drive_direct(100, 90)
        time.sleep(moveTime)
    if dst < 25 * 0.9:
        bot.drive_direct(-100, -90)
        time.sleep(moveTime)
    bot.drive_straight(0)
    time.sleep(0.1)
    print('move %.1f cm' % moveDist)


def rotateChange():
    global x, y, w, h, dst
    rotateAng = int(np.arctan((208-x+w/2)/dst/506.67*100)*180/np.pi)
    rotateTime = rotateAng / 90 * 1.5 * 0.4
    print(rotateTime)
    if rotateAng > 0:
        bot.drive_direct(100, -100)
        time.sleep(rotateTime)
    else:
        bot.drive_direct(-100, 100)
        time.sleep(rotateTime)
    bot.drive_stop()


async def main():
    global x, y, w, h, dst
    global yoloStat, yoloReqIMG
    connection.write(b'\x8c\x03\x01@\x16\x8d\x03')
    
    print('YOLOv3 model loading complete :D')

    bot.start()
    bot.safe()
    pwmT.ChangeDutyCycle(9)
    time.sleep(0.75)
    pwmGripper.ChangeDutyCycle(2.5)
    time.sleep(0.75)
    bot.turn_angle(0, 100)
    
    while True:
        await asyncio.sleep(0.04)
        if yoloStat == 'Ready':
            # Setting CREATE2
            camera.capture('ColorDetectIn.jpg')
            colorDetect('ColorDetectIn.jpg')
            while thisIsGRBTT:
                print('%d  %d  %d  %d  %.1f' % (x, y, w, h, dst))
                if dst > 25:
                    rotateChange()
                    motionChange()
                    camera.capture('ColorDetectIn.jpg')
                    colorDetect('ColorDetectIn.jpg')
                else:
                    pwmT.ChangeDutyCycle(5)
                    time.sleep(0.75)
                    pwmGripper.ChangeDutyCycle(10)
                    time.sleep(0.75)
                    pwmT.ChangeDutyCycle(8.5)
                    time.sleep(0.75)
                    bot.drive_turn(90,100)
                    
                    bot.drive_stop()
                    break
            '''
            # random
            rotateTime = rotateAng / 90 * 1.5 * 0.4
            print(rotateTime)
            if rotateAng > 0:
                bot.drive_direct(100, -100)
            else:
                bot.drive_direct(-100, 100)
            time.sleep(rotateTime)
            bot.drive_stop()
            '''


loop = asyncio.get_event_loop()
tasks = [
    asyncio.async(main())
    #asyncio.async(objDetect())
]
try:
    loop.run_until_complete(asyncio.gather(*tasks))
except KeyboardInterrupt:
    loop.close()
