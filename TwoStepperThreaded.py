# Messy code to run 2 stepper motors simultaneously with different step inputs per motor
# Written by Briana Bouchard 

import RPi.GPIO as GPIO
import time
import threading
#from Motors.ThreadStepperLib import Stepper
from adafruit_apds9960.apds9960 import APDS9960
from adafruit_apds9960 import colorutility
import digitalio
import board

i2c = board.I2C()  # uses board.SCL and board.SDA
apds = APDS9960(i2c)
apds.enable_color = True

color=[100]
error_current=[100]
error_integral=[]
error_derivative=[0]
Kp=0.1
Ki=0.0001
Kd=0.1

# Define the GPIO pins for the L298N motor driver

# Initialize pins using BCM mode (GPIO pin numbers not board numbers)
yellowL = digitalio.DigitalInOut(board.D18)
redL = digitalio.DigitalInOut(board.D17)
grayL = digitalio.DigitalInOut(board.D27)
greenL = digitalio.DigitalInOut(board.D22)

# Initialize pins using BCM mode (GPIO pin numbers not board numbers)
yellowR = digitalio.DigitalInOut(board.D6)
redR = digitalio.DigitalInOut(board.D13)
grayR = digitalio.DigitalInOut(board.D19)
greenR = digitalio.DigitalInOut(board.D26)

Motor1 = [yellowL,redL,grayL,greenL]
Motor2 = [yellowR,redR,grayR,greenR]
print(Motor1)


# Define direction values
cw = 1
ccw = 0


def setMotor(motor, current_step, delay):
# This function provides the step sequence

    if current_step == 0:
        GPIO.output(motor[0],GPIO.HIGH)
        GPIO.output(motor[1],GPIO.LOW)
        GPIO.output(motor[2],GPIO.HIGH)
        GPIO.output(motor[3],GPIO.LOW)
        time.sleep(delay)

    elif current_step == 1:
        GPIO.output(motor[0],GPIO.LOW)
        GPIO.output(motor[1],GPIO.HIGH)
        GPIO.output(motor[2],GPIO.HIGH)
        GPIO.output(motor[3],GPIO.LOW)
        time.sleep(delay)

    elif current_step == 2:
        GPIO.output(motor[0],GPIO.LOW)
        GPIO.output(motor[1],GPIO.HIGH)
        GPIO.output(motor[2],GPIO.LOW)
        GPIO.output(motor[3],GPIO.HIGH)
        time.sleep(delay)
        
    elif current_step == 3:
        GPIO.output(motor[0],GPIO.HIGH)
        GPIO.output(motor[1],GPIO.LOW)
        GPIO.output(motor[2],GPIO.LOW)
        GPIO.output(motor[3],GPIO.HIGH)
        time.sleep(delay)


def moveSteps(motor: list[int], input_steps, steps_rev, speed):    
# This function tracks the number of steps remaining based on the step input and the loop cycles

    current_step = 0
    delay = 60/(steps_rev*speed)
    
    # Determines the direction based on sign of input_steps 
    if input_steps > 0:
        direction = ccw
    if input_steps < 0:
        direction = cw
    
    # Track and set current step iteration
    for steps_remaining in range (abs(input_steps), 0, -1):
        if direction == cw: 
            if current_step >= 0 and current_step < 3:
                current_step = current_step + 1
            elif current_step == 3:
                current_step = 0
        if direction == ccw: 
            if current_step <= 3 and current_step > 0:
                current_step = current_step - 1
            elif current_step == 0:
                current_step = 3
                
        setMotor(motor, current_step, delay)
        
    print("Stepping complete! Your motor completed "  + str(abs(input_steps)) + " steps at " + str(speed)+ " revolutions per minute")

class Stepper (threading.Thread):
    def __init__(self, threadID, name, motor, steps, steps_rev, speed):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.motor = motor
        self.steps = steps
        self.steps_rev = steps_rev
        self.speed = speed

    def run(self):
        print("Starting " + str(self.name))

        # Set GPIO pins for motors as outputs
        pin = 0
        for pin in range(0,4,1):
            GPIO.setup(self.motor[pin], GPIO.OUT)
        
        moveSteps(self.motor,self.steps,self.steps_rev,self.speed)
        
        print("Exiting " + str(self.name))


try:
    while True:
            # wait for color data to be ready
        while not apds.color_data_ready:
            time.sleep(0.005)
        # get the data and print the different channels
        r, g, b, c = apds.color_data
        color.append(r)
        error_current.append(color[-1]-400) #this will probably have to be a range - we will have to test the color temp values for each tape
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
        steps1=round(steps_rev-PID)
        steps2=round(200+PID)
        
        # Set the thread number, thread ID, motor, number of steps to move, steps per revolution,
        # and the speed in revolutions per minute
        stepper1 = Stepper(1,"Motor #1",Motor1, steps1, steps_rev, 20)
        stepper2 = Stepper(2,"Motor #2",Motor2, steps2, steps_rev, 20)

        # Start the motor threads
        stepper1.start()
        stepper2.start()

        # Check to see if both threads are done and clean up GPIO pins when done
        while True:
            if stepper1.is_alive() == False and stepper2.is_alive() == False:
                GPIO.cleanup()
                break
except KeyboardInterrupt:
    GPIO.cleanup()