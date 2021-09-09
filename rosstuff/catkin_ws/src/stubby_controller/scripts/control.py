#! /usr/bin/env python3.8

import rospy
import time
import numpy as np 
from std_msgs.msg import Float64

def reset_angles():
    ang1 = rospy.Publisher("stubby/Rev1_position_controller/command",Float64, queue_size = 1)
    ang2 = rospy.Publisher("stubby/Rev2_position_controller/command",Float64, queue_size = 1)
    ang3 = rospy.Publisher("stubby/Rev3_position_controller/command",Float64, queue_size = 1)
    ang1.publish(0)
    ang2.publish(0)
    ang3.publish(0)

def test_angles():
    ang1 = rospy.Publisher("stubby/Rev1_position_controller/command",Float64, queue_size = 1)
    ang2 = rospy.Publisher("stubby/Rev2_position_controller/command",Float64, queue_size = 1)
    ang3 = rospy.Publisher("stubby/Rev3_position_controller/command",Float64, queue_size = 1)
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        
        current_time = rospy.get_time()
        angle = current_time
        ang1.publish(angle)
        if angle<np.pi/4:
            ang2.publish(angle)
        if angle == 360:
            reset_angles()
            break

if __name__ == "__main__":
    rospy.init_node("angle_stuff", anonymous=True)
    try:
        reset_angles()
        test_angles()
    except rospy.ROSInterruptException:
        pass
