#!/usr/bin/env python
import RPi.GPIO as GPIO
from gpiozero import Servo 
from time import sleep
import rospy
from std_msgs.msg import Float32

servo = Servo(12)
def servo_callback(data):
    angle =  0
    if data.data ==  0:
        angle = -0.5
        print("here")
    else:
        angle = 0.5
        print(data)

    
    print("message recived")
    servo.value = angle



def servo_node_listener():
    rospy.init_node("servo_node", anonymous=True)
    rospy.Subscriber('servo_state', Float32, servo_callback)
    rospy.spin()

if __name__ == '__main__':
    servo_node_listener()
    print(servo.value)
    
    