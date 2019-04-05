# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
 
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (416, 416)
#camera.rotation = 180
camera.framerate = 24
rawCapture = PiRGBArray(camera, size=(416, 416))

# allow the camera to warmup
time.sleep(0.1)

# define range of blue color in HSV
lower_blue = np.array([45,50,50])
upper_blue = np.array([75,255,255])
dst = 0
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    img = frame.array
    
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
    # Threshold the HSV image to get only green colors
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    x, y, w, h = 0, 0, 0, 0
    area = 0
    for c in contours:
        rect = cv2.boundingRect(c)
        tmp_x,tmp_y,tmp_w,tmp_h = rect
        area = max(area, tmp_w * tmp_h)
        if area == tmp_w * tmp_h:
            x, y, w, h = rect
            epsilon = 0.08 * cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, epsilon, True)
    
    # 40 * 60 = 2400 px @max=72cm
    if area > 2400:
        dst = 506.67*6/min(h, w)
        if y >= h/21*3:
            crp = img[int(y-h/21*3):y+h, x:x+w]
            cv2.imshow("Mask", crp)
        cv2.drawContours(img, [approx], -1, (0, 0, 255), 2)
        cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.putText(img, "%.2fcm" % (dst),
            (img.shape[1] - 200, img.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX,
            2.0, (0, 255, 0), 3)
    gry = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.imshow("Frame", gry)
    print('%d  %d  %d  %d  %.1f' % (int(x+w/2), int(y+h/2), w, h, dst))

	# if the `q` key was pressed, break from the loop
    key = cv2.waitKey(1) & 0xFF
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
    if key == ord("q"):
        break
