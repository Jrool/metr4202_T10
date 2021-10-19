#!/usr/bin/env python
import RPi.GPIO as GPIO
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Servo 
from time import sleep
import rospy
from std_msgs.msg import Float32
import pigpio
servo = 12
pwm = pigpio.pi()
pwm.set_mode(servo, pigpio.OUTPUT)
pwm.set_PWM_frequency(servo, 50)

def servo_callback(data):
    angle =  0
    #closed = 0
    if data.data ==  0:
        angle = 1520
    else:
        angle = 2000

    
    print("message recived")
    pwm.set_servo_pulsewidth(servo, angle)



def servo_node_listener():
    rospy.init_node("servo_node", anonymous=True)
    rospy.Subscriber('servo_state', Float32, servo_callback)
    rospy.spin()

if __name__ == '__main__':
    servo_node_listener()
    
    
