import sys
import time
import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BOARD)

mode=GPIO.getmode()

motor_pins=(11,12,13,15) #Yellow Red Green Gray

#Motor section
GPIO.setup(motor_pins, GPIO.OUT)

GPIO.output(motor_pins, (GPIO.LOW,GPIO.LOW,GPIO.LOW,GPIO.LOW))

def direction_control(step_number, inputdelay):
    for y in range(0,step_number,4):
        try:
            GPIO.output(motor_pins, (GPIO.HIGH,GPIO.LOW,GPIO.HIGH,GPIO.LOW))
            sleep(inputdelay)
            GPIO.output(motor_pins, (GPIO.LOW,GPIO.HIGH,GPIO.HIGH,GPIO.LOW))
            sleep(inputdelay)
            GPIO.output(motor_pins, (GPIO.LOW,GPIO.HIGH,GPIO.LOW,GPIO.HIGH))
            sleep(inputdelay)
            GPIO.output(motor_pins, (GPIO.HIGH,GPIO.LOW,GPIO.LOW,GPIO.HIGH))
            sleep(inputdelay)
            y=y+1
        except KeyboardInterrupt:
            GPIO.cleanup()


direction_control(200,0.01)
GPIO.cleanup()
