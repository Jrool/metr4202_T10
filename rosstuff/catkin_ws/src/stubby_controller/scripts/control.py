#! /usr/bin/env python3.8

from numpy.lib.function_base import angle
import rospy
import time
import getch as inp
import numpy as np 
from std_msgs.msg import Float64

def reset_angles():
    ang1 = rospy.Publisher("stubby_complete/yaw_position_controller/command",Float64, queue_size = 1)
    ang2 = rospy.Publisher("stubby_complete/Rev1_position_controller/command",Float64, queue_size = 1)
    ang3 = rospy.Publisher("stubby_complete/Rev2_position_controller/command",Float64, queue_size = 1)
    ang1.publish(0)
    ang2.publish(0)
    ang3.publish(0)

def test_angles():
    ang1 = rospy.Publisher("stubby_complete/yaw_position_controller/command",Float64, queue_size = 1)
    ang2 = rospy.Publisher("stubby_complete/Rev1_position_controller/command",Float64, queue_size = 1)
    ang3 = rospy.Publisher("stubby_complete/Rev2_position_controller/command",Float64, queue_size = 1)
    
    rate = rospy.Rate(10)
    angle1 = 0
    angle2 = 0
    angle3 = 0
    increment = 0.1
    ang1.publish(angle1)
    ang2.publish(angle2)
    ang3.publish(angle3)
    while not rospy.is_shutdown():
        print("Please enter a control character:")
        print("a d: Revolute 1\nw s: Revolute 2\ni j: Revolute 3\n")
        
        command =  inp.getch()
        if command not in ['w','s','a','d', 'i','j']:
            print("You entered an invalid command")
            continue
        elif command == "w":
            angle2+=increment
        elif command == "s":
            angle2-=increment
        elif command == "a":
            angle1+=increment
        elif command == "d":
            angle1-=increment
        elif command == "j":
            angle3-=increment
        elif command == "i":
            angle3+=increment
        print("You entered " + str(command))
        ang1.publish(angle1)
        ang2.publish(angle2)
        ang3.publish(angle3)
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("angle_stuff", anonymous=True)
    print("This Controller is intended to work with the most recent version of the Stubby robot:")
    print("Currently  this is stubby_v2")
    try:
        reset_angles()
        test_angles()
    except rospy.ROSInterruptException:
        pass
