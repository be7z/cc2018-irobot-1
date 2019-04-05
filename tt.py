from picamera import PiCamera
from subprocess import Popen, PIPE
import threading
from time import sleep
import os, fcntl
import asyncio
sleep(0.1)

yolo_proc = Popen(["./darknet",
                   "detect",
                   "./cfg/yolov3-tiny.cfg",
                   "./yolov3-tiny.weights",
                   "-thresh","0.4"],
                   stdin = PIPE, stdout = PIPE)
fcntl.fcntl(yolo_proc.stdout.fileno(), fcntl.F_SETFL, os.O_NONBLOCK)

async def Che():
    global i, imgName
    stdout = ''
    while True:
        await asyncio.sleep(0.2)
        try:
            # YOLOv3 Model Ready?
            if not ('Enter Image Path' in stdout):
                stdout = yolo_proc.stdout.read().decode()
                print(stdout)
                if 'bottle' in stdout:
                    print('------- Green Bottle -------')
                i = 0
            try:
                if (i == 1):# and ('Enter Image Path' in stdout):
                    #print('-------------------# '+str(i)+'  '+stdout+imgName)
                    yolo_proc.stdin.write((imgName + '\n').encode())
                    yolo_proc.stdin.flush()
                    stdout = yolo_proc.stdout.read().decode()
                    stdout=''
                    imgName=''
                    i=0
                
            except Exception:
                pass
        except Exception:
            pass

def check1(inputName):
    global i, imgName
    imgName = inputName
    i = 1

async def main2():
    global i
    while True:
        await asyncio.sleep(0.2)
        if i==0:
            check1('../Desktop/pdictInput.png')
            break
    while True:
        await asyncio.sleep(0.2)
        if i==0:
            check1('data/dog.jpg')
            break

i=-1
imgName=''
#---------- ASYNC ----------#
loop = asyncio.get_event_loop()
tasks = [
    asyncio.async(Che()),
    asyncio.async(main2())
]

try:
    loop.run_until_complete(asyncio.gather(*tasks))
except KeyboardInterrupt:
    loop.close()
