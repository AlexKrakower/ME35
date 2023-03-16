import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import requests
import json
import numpy as np
from rclpy.qos import ReliabilityPolicy, QoSProfile
import re
import ast
from irobot_create_msgs.action import AudioNoteSequence
from irobot_create_msgs.msg import AudioNote
from irobot_create_msgs.msg import AudioNoteVector
from builtin_interfaces.msg import Duration
from keras.models import load_model  # TensorFlow is required for Keras to work
import cv2  # Install opencv-python
import cv2 as cv
import numpy as np
from picamera2 import Picamera2
from libcamera import controls
import time

# Disable scientific notation for clarity
np.set_printoptions(suppress=True)

# Load the model
model = load_model("keras_model.h5", compile=False)

# Load the labels
class_names = open("labels.txt", "r").readlines()

picam2 = Picamera2() # assigns camera variable
picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous}) # sets auto focus mode
picam2.start() # activates camera

def CameraModel():
    img_name = 'image.jpg'
    picam2.capture_file(img_name) #take image 

    image = cv.imread("image.jpg") #read image with open cv, to get the bgr value of one pixel index using print(img[row][col])

    # Resize the raw image into (224-height,224-width) pixels
    image = cv2.resize(image, (224, 224), interpolation=cv2.INTER_AREA)

    # Make the image a numpy array and reshape it to the models input shape.
    image = np.asarray(image, dtype=np.float32).reshape(1, 224, 224, 3)

    # Normalize the image array
    image = (image / 127.5) - 1

    # Predicts the model
    prediction = model.predict(image)
    index = np.argmax(prediction)
    class_name = class_names[index]
    confidence_score = prediction[0][index]

    # Print prediction and confidence score
    print("Class:", class_name[2:], end="")
    print("Confidence Score:", str(np.round(confidence_score * 100))[:-2], "%")
    
    case1=str(class_name[2:]).strip()
    case=''.join(case1)
    CI1=str(np.round(confidence_score * 100))[:-2]
    CI=int(CI1)
    
    print(case)
    print(CI)
    
    return case, CI


#Motion Command Inputs
def GoLeft():

    linx=0
    liny=0
    linz=0
    angx=0
    angy=0
    angz=1
    
    return linx, liny, linz, angx, angy, angz

def GoRight():

    linx=0
    liny=0
    linz=0
    angx=0
    angy=0
    angz=-1
    
    return linx, liny, linz, angx, angy, angz

def GoStraight():

    linx=0.1
    liny=0
    linz=0
    angx=0
    angy=0
    angz=0
    
    return linx, liny, linz, angx, angy, angz
    
def Stop():

    linx=0
    liny=0
    linz=0
    angx=0
    angy=0
    angz=0
    
    return linx, liny, linz, angx, angy, angz

#Do Something Fun Publisher
class Sound():
    def __init__(self):
        print("Hi")
        self.note1=AudioNote(frequency=200, max_runtime=Duration(sec=1,nanosec=0))
        self.note2=AudioNote(frequency=250, max_runtime=Duration(sec=1,nanosec=0))
        self.note3=AudioNote(frequency=300, max_runtime=Duration(sec=1,nanosec=0))
        self.note4=AudioNote(frequency=350, max_runtime=Duration(sec=1,nanosec=0))

        
class SoundPublisher(Node):
    def __init__(self):
        
        super().__init__('Sound_Publisher')
        self.cp = Sound()
        self.Sound_Publisher = self.create_publisher(AudioNoteVector, '/cmd_audio', 10)

        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.noise = AudioNote()
        self.noisevector = AudioNoteVector()
       
    def timer_callback(self):
        [case, CI2] = CameraModel()
        print(case)
        CI=int(CI2)
        print(type(CI))
        if case=="Elephant": 
            if CI>90:
                self.noisevector.notes=[self.cp.note1, self.cp.note2, self.cp.note3, self.cp.note4]
                self.noisevector.append=True
                self.Sound_Publisher.publish(self.noisevector)
                print("playing note")
                time.sleep(3)
            else:
                print("No Fun Thing")
        else:
            print("No Fun Thing")
    def reset(self):
        self.Sound_Publisher.publish(self.noisevector)

#Motion Publisher and Subscriber
class Direction():
    def __init__(self):
        print("Hi")
                
class DirectionPublisher(Node):
    def __init__(self):
        
        super().__init__('Direction_Publisher')
        self.cp = Direction()
        self.Direction_Publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.driving = Twist()
        #self.driving.override_system = True
       
    def timer_callback(self):
        [case, CI2] = CameraModel()
        print(case)
        CI=int(CI2)
        print(type(CI))
        count=860
        current_time = self.get_clock().now()
        print(case)
        if CI>90:
            if case=="Mario":
                [linx, liny, linz, angx, angy, angz] = GoRight()
                print("Going Left!")
            elif case=="Mug":
                [linx, liny, linz, angx, angy, angz] = GoLeft()
                print("Going Left!")
            elif case=="Rubix Cube":
                [linx, liny, linz, angx, angy, angz] = GoLeft()
                print("Going Left!")
            elif case=="Tractor":
                [linx, liny, linz, angx, angy, angz] = GoLeft()
                print("Going Left!")
            elif case=="Kiwi":
                [linx, liny, linz, angx, angy, angz] = GoRight()
                print("Going Right!")
            elif case=="Bear":
                [linx, liny, linz, angx, angy, angz] = GoLeft()
                print("Going Left!")
            elif case=="Elephant":
                [linx, liny, linz, angx, angy, angz] = Stop()
                print("Stopping!")
                exit()
        else:
            [linx, liny, linz, angx, angy, angz] = GoStraight()
            print("Going Straight!")
        
        if linx!=0.1:
            for num in range(count):
                self.driving.linear.x = float(linx)
                self.driving.linear.y = float(liny)
                self.driving.linear.z = float(linz)
                self.driving.angular.x = float(angx)
                self.driving.angular.y = float(angy)
                self.driving.angular.z = float(angz)
                self.Direction_Publisher.publish(self.driving) 
                print("turning")
        else:
            self.driving.linear.x = float(linx)
            self.driving.linear.y = float(liny)
            self.driving.linear.z = float(linz)
            self.driving.angular.x = float(angx)
            self.driving.angular.y = float(angy)
            self.driving.angular.z = float(angz)
            self.Direction_Publisher.publish(self.driving) 

       
    def reset(self):
        #self.driving.override_system = False
        self.Direction_Publisher.publish(self.driving)

def main(args=None):
    while True:
        '''
        The rclpy library is initialized.
        '''
        rclpy.init(args=args)
       
        '''
        The node is created and can be used in other parts of the script.
        '''
        sound_publisher = SoundPublisher()
        direction_publisher = DirectionPublisher()

        '''
        The node is "spun" so the callbacks can be called.
        '''
        print('Callbacks are called')
        
        try:
            rclpy.spin_once(sound_publisher)
            rclpy.spin_once(direction_publisher)
        except KeyboardInterrupt:
            print('\nCaught Keyboard Interrupt')
            break
        finally:
            print("Done")  # Destroy the node explicitly
            sound_publisher.destroy_node()
            direction_publisher.destroy_node()
            print('shutting down')
            rclpy.shutdown()
            

if __name__ == '__main__':
    main()
