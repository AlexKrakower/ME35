##WORKING CODE

# Core opencv code provided by Einsteinium Studios
# Revisions to work with Pi Camera v3 by Briana Bouchard

import board
import RPi.GPIO as GPIO
import digitalio
import numpy as np
import cv2
import cv2 as cv
from picamera2 import Picamera2
from libcamera import controls
import time

picam2 = Picamera2() # assigns camera variable
picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous}) # sets auto focus mode
picam2.start() # activates camera

time.sleep(1) # wait to give camera time to start up
 
#create boundary for red values as two arrays
low_blue = np.array([94, 80, 2])
high_blue = np.array([126, 255, 255])

low_green = np.array([25, 52, 72])
high_green = np.array([102, 255, 255])

low_red = np.array([161, 155, 84])
high_red = np.array([179, 255, 255])

low_black = np.array([0, 0, 0])
high_black = np.array([180, 255, 30])

low_purple = np.array([129, 50, 70])
high_purple = np.array([158, 255, 255])

ledPin1 = 23
ledPin2 = 24
ledPin3 = 25

GPIO.setup(ledPin1, GPIO.OUT)
GPIO.setup(ledPin2, GPIO.OUT)
GPIO.setup(ledPin3, GPIO.OUT)

GPIO.output(ledPin1, GPIO.HIGH)
GPIO.output(ledPin2, GPIO.HIGH)
GPIO.output(ledPin3, GPIO.HIGH)

# Initialize pins using BCM mode (GPIO pin numbers not board numbers)
yellowL = digitalio.DigitalInOut(board.D18)
yellowL.direction = digitalio.Direction.OUTPUT
redL = digitalio.DigitalInOut(board.D17)
redL.direction = digitalio.Direction.OUTPUT
grayL = digitalio.DigitalInOut(board.D27)
grayL.direction = digitalio.Direction.OUTPUT
greenL = digitalio.DigitalInOut(board.D22)
greenL.direction = digitalio.Direction.OUTPUT

# Initialize pins using BCM mode (GPIO pin numbers not board numbers)
yellowR = digitalio.DigitalInOut(board.D6)
yellowR.direction = digitalio.Direction.OUTPUT
redR = digitalio.DigitalInOut(board.D13)
redR.direction = digitalio.Direction.OUTPUT
grayR = digitalio.DigitalInOut(board.D19)
grayR.direction = digitalio.Direction.OUTPUT
greenR = digitalio.DigitalInOut(board.D26)
greenR.direction = digitalio.Direction.OUTPUT


# Define direction values
cw = 1
ccw = 0

# Define the steps per revolution for the motor
steps_rev = 200

def setMotorRight(current_step2, delay):
# This function provides the step sequence

    if current_step2 == 0:
        yellowR.value = True
        redR.value = False
        grayR.value = True
        greenR.value = False
        time.sleep(delay)

    elif current_step2 == 1:
        yellowR.value = False
        redR.value = True
        grayR.value = True
        greenR.value = False
        time.sleep(delay)

    elif current_step2 == 2:
        yellowR.value = False
        redR.value = True
        grayR.value = False
        greenR.value = True
        time.sleep(delay)
       
    elif current_step2 == 3:
        yellowR.value = True
        redR.value = False
        grayR.value = False
        greenR.value = True
        time.sleep(delay)
       
def setMotorLeft(current_step1, delay):
# This function provides the step sequence

    if current_step1 == 0:
        yellowL.value = True
        redL.value = False
        grayL.value = True
        greenL.value = False
        time.sleep(delay)

    elif current_step1 == 1:
        yellowL.value = False
        redL.value = True
        grayL.value = True
        greenL.value = False
        time.sleep(delay)

    elif current_step1 == 2:
        yellowL.value = False
        redL.value = True
        grayL.value = False
        greenL.value = True
        time.sleep(delay)
       
    elif current_step1 == 3:
        yellowL.value = True
        redL.value = False
        grayL.value = False
        greenL.value = True
        time.sleep(delay)


def moveStepsRight(input_steps, speed):    
# This function tracks the number of steps remaining based on the step input and the loop cycles

    current_step2 = 0
    input_steps2=round(input_steps)
    delay = 60/(steps_rev*speed)
   
    # Determines the direction based on sign of input_steps
    if input_steps2 > 0:
        direction = ccw
    if input_steps2 < 0:
        direction = cw
   
    for steps_remaining in range (abs(input_steps2), 0, -1):
        if direction == cw:
            if current_step2 >= 0 and current_step2 < 3:
                current_step2 = current_step2 + 1
            elif current_step2 == 3:
                current_step2 = 0
        if direction == ccw:
            if current_step2 <= 3 and current_step2 > 0:
                current_step2 = current_step2 - 1
            elif current_step2 == 0:
                current_step2 = 3     
   
        setMotorRight(current_step2, delay)
       
   
def moveStepsLeft(input_steps, speed):    
# This function tracks the number of steps remaining based on the step input and the loop cycles

    current_step1 = 0
    input_steps1=round(input_steps)
    print(input_steps1)
    delay = 60/(steps_rev*speed)
   
    # Determines the direction based on sign of input_steps
    if input_steps1 > 0:
        direction = ccw
    if input_steps1 < 0:
        direction = cw
   
    for steps_remaining in range (abs(input_steps1), 0, -1):
        if direction == cw:
            if current_step1 >= 0 and current_step1 < 3:
                current_step1 = current_step1 + 1
            elif current_step1 == 3:
                current_step1 = 0
        if direction == ccw:
            if current_step1 <= 3 and current_step1 > 0:
                current_step1 = current_step1 - 1
            elif current_step1 == 0:
                current_step1 = 3     
   
        setMotorLeft(current_step1, delay)
       
previous = 3
while(True):
       

	#Percent color detection
    img_name = 'image.jpg'
    picam2.capture_file(img_name) #take image 

    img = cv.imread("image.jpg") #read image with open cv, to get the bgr value of one pixel index using print(img[row][col])
	# Crop the image
    crop_img = img[50:130, 0:160]
    hsv = cv.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
    total_pixels = crop_img.shape #returns [2529, 4608] as the shape of the image

    #determine if the pixel in the image has bgr values within the range
    image_mask_blue = cv.inRange(hsv,low_blue,high_blue) #returns array of 0s & 255s, 255=white=within range, 0=black=not in range
    image_mask_green = cv.inRange(hsv,low_green,high_green)
    image_mask_red = cv.inRange(hsv,low_red,high_red)
    image_mask_black = cv.inRange(hsv,low_black,high_black)
    image_mask_purple = cv.inRange(hsv,low_purple,high_purple)

    in_range_blue = np.count_nonzero(image_mask_blue) #count the number of elements in the array that are not zero (in other words elements that are in the red range)
    in_range_green = np.count_nonzero(image_mask_green)
    in_range_red = np.count_nonzero(image_mask_red)
    in_range_black = np.count_nonzero(image_mask_black)
    in_range_purple = np.count_nonzero(image_mask_purple)


    total = total_pixels[0]*total_pixels[1]

    percent_blue = round((in_range_blue/total)*100)
    percent_green = round((in_range_green/total)*100)
    percent_red = round((in_range_red/total)*100)
    percent_black = round((in_range_black/total)*100)
    percent_purple = round((in_range_purple/total)*100)

    if percent_green>0:
        print("Green")
    elif percent_blue>0:
        print("Blue")
    elif percent_red>0:
        print("Red")
    elif percent_black>0:
        print("Black")
    elif percent_purple>0:
        print("Purple")
    else:
        print("White")
    
    # Convert to grayscale
    gray = cv.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
    # Gaussian blur
    blur = cv2.GaussianBlur(gray,(9,9),0)
 
    # Color thresholding
    input_threshold,comp_threshold = cv2.threshold(blur,185,255,cv2.THRESH_BINARY_INV)
    # Find the contours of the frame
    contours,hierarchy = cv2.findContours(comp_threshold.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    speedMotor = 30
    # Find the biggest contour (if detected)
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c) # determine moment - weighted average of intensities
        #print(M)
        if M["m00"] != 0:
            cx = int(M['m10']/M['m00']) # find x component of centroid location
            cy = int(M['m01']/M['m00']) # find y component of centroid location
     
            cv2.line(crop_img,(cx,0),(cx,720),(255,0,0),1) # display vertical line at x value of centroid
            cv2.line(crop_img,(0,cy),(1280,cy),(255,0,0),1) # display horizontal line at y value of centroid
     
            cv2.drawContours(crop_img, contours, -1, (0,255,0), 2) # display green lines for all contours
             
            # determine location of centroid in x direction and adjust steering recommendation
            if cx >= 120:
               print("Turn Left!")
               moveStepsRight(-4, speedMotor)
               #previous=moveStepsRight(-4, 30)
               previous = 2
               
     
            if cx < 120 and cx > 50:
               print("On Track!")
               moveStepsRight(-4, speedMotor)
               moveStepsLeft(4, speedMotor)
 
            if cx <= 50:
               print("Turn Right")
               moveStepsLeft(4, speedMotor)
               previous = 1
               #previous=moveStepsLeft(4, 30)
 
    else:
       print("I don't see the line")
       if previous == 2:
           moveStepsRight(-4, speedMotor)
       if previous == 1:
           moveStepsLeft(4, speedMotor)
       if previous == 3:
           print("First iteration")
 
    # Display the resulting frame
    #cv2.imshow('frame',crop_img)
   
    # Check for "q" key press to end program
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
