# -------------------- library ------------------------#

# lib for CREATE-2
import sys, glob # for listing serial ports
import struct
import time
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

# lib for async
import asyncio

# lib for CREATE2
from  pycreate2 import Create2

# ------------------- globla var ----------------------#

# serial connect'n
connection = None

# camera mod
camera = PiCamera()
camera.resolution = (416, 416)
camera.rotation = 180
camera.framerate = 60
rawCapture = PiRGBArray(camera, size=(416, 416))
x, y, w, h, dst = 0, 0, 0, 0, 0

# range of green color in HSV
lower_blue = np.array([45,50,50])
upper_blue = np.array([75,255,255])

# -------------------- function ----------------------#
def motion(velo, rot):'''
    if velo > 0:
        # forward
        sendCommandASCII('145 0 200 0 200')
    elif velo < 0:
        # backward
        sendCommandASCII('145 255 56 255 56')
    elif rot > 0:
        # Turn R
        sendCommandASCII('145 255 106 0 150')
    elif rot < 0:
        # turn L
        sendCommandASCII('145 0 150 255 106')
    elif velo == 0 and rot == 0:
        sendCommandASCII('145 0 0 0 0')'''
    time.sleep(1.0/24.0)

# capture frames from the camera
@asyncio.coroutine
def camVar():
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        yield from asyncio.sleep(0.05)
        global x, y, w, h, dst
        img = frame.array
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
        if min(h, w) > 20:
            dst = 506.67*6/min(h, w)
        if area > 2400:
            # draw ocv-rect box
            '''if y >= h/21*3:
                crp = img[y-h/21*3:y+h, x:x+w]
                cv2.imshow("Mask", crp)'''
            cv2.drawContours(img, [approx], -1, (0, 0, 255), 2)
            cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(img, "%.2f cm" % (dst),
                (img.shape[1] - 200, img.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX,
                2.0, (0, 255, 0), 3)
        gry = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        cv2.imshow("Frame", gry)
        
        # if the `q` key was pressed, break from the loop
        key = cv2.waitKey(1) & 0xFF
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        if key == ord('q'):
            motion(0, 0)

@asyncio.coroutine
def main():
    global x, y, w, h, dst
    while True:
        yield from asyncio.sleep(0.01)
        print('%d  %d  %d  %d  %.1f' % (x, y, w, h, dst))
        if (x+w/2 >= 208-w and x+w/2<=208+w):
            motion(0, 0)
        else:
            if x+w/2 < 208-w:
                motion(0, -1)
            if x+w/2 > 208+w:
                motion(0, 1)
            '''if dst > 35:
                motion(1, 0)
            if dst < 25:
                motion(-1, 0)'''
         
@asyncio.coroutine
def main2():
    while True:
        yield from asyncio.sleep(0.5)
        sendCommandASCII('140 3 1 64 16 141 3')
        print(connection.read())
    

### Begin Connect to Create2 ###
AvailablePort()
ConnectPort('/dev/ttyUSB0')
'''
# Reset
sendCommandASCII('7')
time.sleep(5)
# Passive Mode
sendCommandASCII('128')
time.sleep(2)
# Full Mode
sendCommandASCII('132')
time.sleep(2)'''
# Beep !!
sendCommandASCII('140 3 1 64 16 141 3')
#bot = Create2('/dev/ttyUSB0', 115200)
#bot.turn_angle(45, 100)

#---------- ASYNC ----------#
loop = asyncio.get_event_loop()
tasks = [
    asyncio.async(camVar()),
    asyncio.async(main2())
]

try:
    loop.run_until_complete(asyncio.gather(*tasks))
except KeyboardInterrupt:
    loop.close()


bot=Create2('/dev/ttyUSB0', 115200)
bot.start()
bot.safe()

