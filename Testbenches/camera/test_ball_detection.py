from collections import deque
from imutils.video import VideoStream
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import cv2
import imutils
import time

green_lower = (40,50,50)
green_upper = (64,255,255)

blue_lower = (65,50,50)
blue_upper= (130,255,255)

red_lower = (0,50,50)
red_upper = (10,255,255)

yellow_lower = (10,50,50)
yellow_upper = (40,255,255)

white_sensitivity = 100
white_lower = (0,0,255-white_sensitivity)
white_upper = (255,white_sensitivity,255)
pts = deque(maxlen=64)

#initialize camera and get reference to raw camera feed
camera = PiCamera()
camera.resolution = (640,480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size = (640,480))

#allow camera to warmup
time.sleep(0.1)

for frame in camera.capture_continuous(rawCapture, format = "bgr", use_video_port=True):
    image = frame.array
    image = imutils.resize(image, width=600)
    blurred = cv2.GaussianBlur(image, (11,11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    green_mask = cv2.inRange(hsv, green_lower, green_upper)
    green_mask = cv2.erode(green_mask, None, iterations=2)
    green_mask = cv2.dilate(green_mask, None, iterations=2)

    blue_mask = cv2.inRange(hsv, blue_lower, blue_upper) 
    blue_mask = cv2.erode(blue_mask, None, iterations=2)
    blue_mask = cv2.dilate(blue_mask, None, iterations=2)

    red_mask = cv2.inRange(hsv, red_lower, red_upper)
    red_mask = cv2.erode(red_mask, None, iterations=2)
    red_mask = cv2.dilate(red_mask, None, iterations=2)

    yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper) 
    yellow_mask = cv2.erode(yellow_mask, None, iterations=2)
    yellow_mask = cv2.dilate(yellow_mask, None, iterations=2)

    white_mask = cv2.inRange(hsv, white_lower, white_upper) 
    white_mask = cv2.erode(white_mask, None, iterations=2)
    white_mask = cv2.dilate(white_mask, None, iterations=2)
   

    green_cnts = cv2.findContours(green_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blue_cnts = cv2.findContours(blue_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    red_cnts = cv2.findContours(red_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    yellow_cnts = cv2.findContours(yellow_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    white_cnts = cv2.findContours(white_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    green_cnts = imutils.grab_contours(green_cnts)
    blue_cnts = imutils.grab_contours(blue_cnts)
    red_cnts = imutils.grab_contours(red_cnts)
    yellow_cnts = imutils.grab_contours(yellow_cnts)
    white_cnts = imutils.grab_contours(white_cnts)

    green_center = None
    blue_center = None
    red_center = None
    yellow_center = None
    white_center = None

    if len(green_cnts) > 0:
        for c in green_cnts:
	    epsilon = 0.01*cv2.arcLength(c, True)
	    approx = cv2.approxPolyDP(c, epsilon, True)
	    if cv2.contourArea(approx) > 19000:
	        cv2.drawContours(image, [approx], -1, (0,0, 255), 2)
		if(cv2.contourArea(approx) > 50000):
			print("Green Tape")
		else:
			print("Green Debris")

    if len(blue_cnts) > 0:
        for c in blue_cnts:
	    epsilon = 0.01*cv2.arcLength(c, True)
	    approx = cv2.approxPolyDP(c, epsilon, True)
	    if cv2.contourArea(approx) > 19000:
	        cv2.drawContours(image, [approx], -1, (0,0, 255), 2)
	        if(cv2.contourArea(approx) > 50000):
			print("Blue Tape")
		else:
			print("Blue Debris")
	

    if len(red_cnts) > 0:
	for c in red_cnts:
	    epsilon = 0.01*cv2.arcLength(c, True)
	    approx = cv2.approxPolyDP(c, epsilon, True)
	    if cv2.contourArea(approx) > 19000:
	        cv2.drawContours(image, [approx], -1, (0,0, 255), 2)
	        if(cv2.contourArea(approx) > 50000):
			print("Red Tape")
		else:
			print("Red Debris")


    if len(yellow_cnts) > 0:
        for c in yellow_cnts:
	    epsilon = 0.01*cv2.arcLength(c, True)
	    approx = cv2.approxPolyDP(c, epsilon, True)
	    if cv2.contourArea(approx) > 19000:
	        cv2.drawContours(image, [approx], -1, (0,0, 255), 2)
	        if(cv2.contourArea(approx) > 50000):
			print("Yellow Tape")
		else:
			print("Yellow Debris")


    if len(white_cnts) > 0:
        for c in white_cnts:
	    epsilon = 0.01*cv2.arcLength(c, True)
	    approx = cv2.approxPolyDP(c, epsilon, True)
	    if cv2.contourArea(approx) > 19000:
	        cv2.drawContours(image, [approx], -1, (0,0, 255), 2)
	        print("White Tape")


    cv2.imshow("Frame", image)
    key = cv2.waitKey(1) & 0xFF

    rawCapture.truncate(0)

    if key == ord("q"):
        break

cv2.destroyAllWindows()
