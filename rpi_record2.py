from picamera import PiCamera
from subprocess import Popen, PIPE
import threading
from time import sleep
import os, fcntl
import cv2
from shutil import copyfile


camera = PiCamera()
camera.resolution = (416, 416)
sleep(0.1)

yolo_proc = Popen(["./darknet",
                   "detect",
                   "./cfg/yolov3-tiny.cfg",
                   "./yolov3-tiny.weights",
                   "-thresh","0.5"],
                   stdin = PIPE, stdout = PIPE)

fcntl.fcntl(yolo_proc.stdout.fileno(), fcntl.F_SETFL, os.O_NONBLOCK)

while True:
    try:
        stdout = yolo_proc.stdout.read().decode()
        print(stdout)
        if 'Enter Image Path' in stdout:
            camera.capture('test.jpg')
            try:
                yolo_proc.stdin.write('test.jpg\n'.encode())
                yolo_proc.stdin.flush()
                im = cv2.imread('predictions.png')
                cv2.imshow('yolov3-tiny',im)
                key = cv2.waitKey(5)
            except Exception as e:
                print(e)
                pass
               
        if len(stdout.strip())>0:
            print('get %s' % stdout)
            
    except Exception:
        pass
