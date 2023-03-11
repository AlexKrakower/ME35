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


def KeyRead():
    URL = 'https://api.airtable.com/v0/appKuG93cT3Q3DqhQ/Commands?api_key=key2aZIIQWBIGlcn2'
    r = requests.get(url = URL, params = {})
    data = r.json()
    linx=data['records'][0]['fields']['Linear x']
    liny=data['records'][0]['fields']['Linear y']
    linz=data['records'][0]['fields']['Linear z']
    angx=data['records'][0]['fields']['Angular x']
    angy=data['records'][0]['fields']['Angular y']
    angz=data['records'][0]['fields']['Angular z']
    cnt = data['records'][0]['fields']['Cnt']
    
    return linx, liny, linz, angx, angy, angz, cnt
    
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
        
        current_time = self.get_clock().now()
       
        
        x = input('Ready? (y/n)')
        if x == 'y':
        
            [linx, liny, linz, angx, angy, angz, cnt] = KeyRead()
            print(cnt)
            
            for i in range(int(cnt)):
                self.driving.linear.x = float(eval(linx))
                self.driving.linear.y = float(eval(liny))
                self.driving.linear.z = float(eval(linz))
                self.driving.angular.x = float(eval(angx))
                self.driving.angular.y = float(eval(angy))
                self.driving.angular.z = float(eval(angz))
                
        else:
            x = input('Ready? (y/n)')
     
        # self.driving.header.stamp = current_time.to_msg()
        self.Direction_Publisher.publish(self.driving)
       
    def reset(self):
        #self.driving.override_system = False
        self.Direction_Publisher.publish(self.driving)


class DirectionSubscriber(Node):
    def __init__(self):
        super().__init__('Direction_subscriber')
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.listener_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)) #NEED TO DEFINE WHAT DRIVING DATA IS
    def listener_callback(self, msg:Twist):
        self.printIR(msg)
    def printIR(self, msg):
        for reading in msg.readings:
            val = reading.value
            print("Twist:" + str(val))

def main(args=None):
    '''
    The rclpy library is initialized.
    '''
    rclpy.init(args=args)
   
    '''
    The node is created and can be used in other parts of the script.
    '''
    direction_publisher = DirectionPublisher()
    direction_subscriber = DirectionSubscriber()

    '''
    The node is "spun" so the callbacks can be called.
    '''
    print('Callbacks are called')
    try:

        rclpy.spin(direction_publisher)
        rclpy.spin(direction_subscriber)
  
    except KeyboardInterrupt:
        print('\nCaught Keyboard Interrupt')
    finally:
        print("Done")  # Destroy the node explicitly
        direction_publisher.reset()
        # direction_subscriber.reset()
        direction_publisher.destroy_node()
        direction_subscriber.destroy_node()
        print('shutting down')
        rclpy.shutdown()

if __name__ == '__main__':
    main()
