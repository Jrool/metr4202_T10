#! /usr/bin/env python3.8
import rospy 
import numpy as np
import getch as inp
import modern_robotics as mr
from sensor_msgs.msg import JointState
from fiducial_msgs.msg import FiducialTransformArray
from fiducial_msgs.msg import FiducialTransform
from std_msgs.msg import Header
import helper_functions as hf

import tf_conversions
import tf2_ros as tf2

def get_aruco_tf(listener, tfBuffer):
    source = "Component2_1"
    #aruco_pos = 
    target = "fixed_id5"
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform(source, target, rospy.Time())
            print(trans)
        except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException) as e:
            print("NotFound")
            print(e)
            rate.sleep()
            continue
        print(trans)
        rate.sleep()
if __name__ == "__main__":
    try:
        rospy.init_node("tf_adders", anonymous = True)
        tfBuffer = tf2.Buffer()
        listener = tf2.TransformListener(tfBuffer)
        
        get_aruco_tf(listener, tfBuffer)   
    except rospy.ROSInterruptException:
        pass