from picamera import PiCamera
from picamera.array import PiRGBArray
from subprocess import Popen, PIPE
from time import sleep
import numpy as np
import threading
import os
import fcntl
import cv2

camera = PiCamera()
camera.resolution = (416, 416)
#camera.framerate = 1
rawCapture = PiRGBArray(camera, size=(416, 416))

sleep(0.1)

yolo_proc = Popen(['./darknet',
                    'detect',
                    './cfg/yolov3-tiny.cfg',
                    './yolov3-tiny.weights',
                    '-thresh', '0.1'],
                    stdin=PIPE, stdout=PIPE)



fcntl.fcntl(yolo_proc.stdout.fileno(), fcntl.F_SETFL, os.O_NONBLOCK)

# define range of blue color in HSV
lower_blue = np.array([45, 50, 50])
upper_blue = np.array([75, 255, 255])


stdout, errs = yolo_proc.communicate()
stdout=stdout.decode()
print(stdout)


while True:
    try:
        gr = 0
        if 'Enter Image Path' in stdout:
            try:
                # yolov3-tiny --> object dection
                image = cv2.imread('predictions.png')
                cam = cv2.imread('test_input.jpg')

                # opencv mask --> color detection
                hsv = cv2.cvtColor(cam, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, lower_blue, upper_blue)
                contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                
            except Exception:
                pass
                
            # cam capture --> input
            camera.capture('test_input.jpg')
            try:
                f = open(yolo_proc.stdin)
                
                yolo_proc.stdin.write(f, 'test_input.jpg\n')
                #stdout, errs = yolo_proc.communicate()
                #stdout=stdout.decode()
                
                print(stdout)
            except Exception as e:
                print(e)
                
            
            print(stdout)
            
            for c in contours:
                rect = cv2.boundingRect(c)
                x, y, w, h = rect
                area = w * h
                if area > 600:
                    gr = 1
                    cv2.drawContours(cam, [approx], -1, (0, 0, 255), 2)
                    cv2.rectangle(cam, (x, y), (x+w, y+h), (0, 255, 0), 2)
            #cv2.imshow('yolov3-tiny', image)
            #cv2.imshow('mask', mask)
            #cv2.imshow('cam', cam)
            key = cv2.waitKey(2)
            
        print('test4')
        # output analyse
        if len(stdout.strip()) > 0:
            print('get %s' % stdout)
            if gr == 1:
                print('---------- Green Bottle - %d ----------' % w)
    except Exception:
        print('ee')
        pass

