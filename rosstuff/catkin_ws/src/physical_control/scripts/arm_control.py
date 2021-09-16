#! /usr/bin/env python3.8

import rospy 
import numpy as numpy
import getch as inp
import modern_robotics as mr
from sensor_msgs.msg import JointState
def control_angles():
    print("begin")
    servoPub = rospy.Publisher("/desired_joint_states",JointState,queue_size = 10)
    rate = rospy.Rate(10)
    joints = JointState()
    joints.name = ["yaw","joint_1","joint_2","joint_3"]
    yaw = 0
    joint1 = 0
    joint2 = 0
    joint3 = 0
    increment = 0.1
    joints.position = [yaw,joint1,joint2, joint3]
    while not rospy.is_shutdown():
        print("Please enter a control character:")
        print("A D: Yaw\nW S: Revolute 1\nJ K: Revolute 2\nH L:Grip Angle")
        command = inp.getch()
        if command not in ["w" ,"s", "a", "d", "j", "k", "h","l"]:
            print("You entered an invalid command")
            continue
        elif command == "w":
            joint1 -= increment
        elif command == "s": 
            joint1 += increment
        elif command == "a":
            yaw += increment
        elif command == "d":
            yaw -= increment
        elif command == "j":
            joint2 -= increment
        elif command == "k":
            joint2 +=increment
        print("\nYou entered" + str(command))
        joints.position = [yaw,joint1,joint2,joint3]
        servoPub.publish(joints)
        rate.sleep()
if __name__ == "__main__":
    rospy.init_node("dynamixal_control", anonymous=True)
    try:
        control_angles()
    except rospy.ROSInterruptException:
        pass
