import time
import board
from adafruit_apds9960.apds9960 import APDS9960
from adafruit_apds9960 import colorutility
import RPi.GPIO as GPIO
import digitalio

i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
apds = APDS9960(i2c)
apds.enable_color = True

ledPin1 = 23
ledPin2 = 24
ledPin3 = 25
#GPIO.setmode(GPIO.BCM)
#GPIO.setwarnings(False)
GPIO.setup(ledPin1, GPIO.OUT)
GPIO.setup(ledPin2, GPIO.OUT)
GPIO.setup(ledPin3, GPIO.OUT)

GPIO.output(ledPin1, GPIO.HIGH)
GPIO.output(ledPin2, GPIO.HIGH)
GPIO.output(ledPin3, GPIO.HIGH)

color=[100]
error_current=[100]
error_integral=[]
error_derivative=[0]
Kp=0.0003
Ki=0.000000
Kd=0.000001

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
    input_steps2=round(input_steps-PID)
    print(input_steps2)
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
       
    print("Stepping complete! Left motor completed " + str(abs(input_steps2)) + " steps at " + str(speed)+ " revolutions per minute")
   
def moveStepsLeft(input_steps, speed):    
# This function tracks the number of steps remaining based on the step input and the loop cycles

    current_step1 = 0
    input_steps1=round(input_steps-PID)
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
       
    print("Stepping complete! Left motor completed " + str(abs(input_steps1)) + " steps at " + str(speed)+ " revolutions per minute")
   
   
stepcounter=0
try:
    j=0
    while True:
            # wait for color data to be ready
        while not apds.color_data_ready:
            time.sleep(0.005)
        # get the data and print the different channels
        r, g, b, c = apds.color_data
        color.append(r)
        error_current.append(color[-1]-12) #this will probably have to be a range - we will have to test the color temp values for each tape
        error_integral.append(sum(error_current))
        error_derivative.append((color[-1]-color[-2])/0.5)
        PID=error_current[-1]*Kp+error_integral[-1]*Ki+error_derivative[-1]*Kd
        print("red: ", r)
        print("green: ", g)
        print("blue: ", b)
        print("clear: ", c)
        print("Current Error: ", error_current[-1])
        print("Integral Error: ", error_integral[-1])
        print("Derivative Error: ", error_derivative[-1])
        print("color temp {}".format(colorutility.calculate_color_temperature(r, g, b)))
        print("light lux {}".format(colorutility.calculate_lux(r, g, b)))
        print("PID", PID)
        time.sleep(0.5)
        # Define the steps per revolution for the motor
        steps_rev = 200
        if j==0:
            PID=0
            moveStepsRight(2, 20)
            moveStepsLeft(2, 20)
        if j > 0:
            if 45000 < c and r > 16000:
                print("color is white")
                moveStepsLeft(0,30)
                moveStepsRight(0,30)
                stepcounter=stepcounter+1
                if stepcounter>6:
                    print("in loop")
                    PID=-2*PID
                    moveStepsLeft(0,30)
                    moveStepsRight(0, 30)
            elif 16000 > r > 10000:
                print("color is red")
                # Set the number of steps to move and the speed in revolutions per minute
                PID=0
                stepcounter=0
                moveStepsRight(-10, 20)
                moveStepsLeft(10, 20)    
            else:
                print("color is black")
                # Set the number of steps to move and the speed in revolutions per minute
                PID=0
                moveStepsRight(-20, 20)
                moveStepsLeft(20, 20)
            
 #       stepcounter=0        
        j=j+1
       
       

except KeyboardInterrupt:
    # Turn off GPIO pins
    GPIO.cleanup()
