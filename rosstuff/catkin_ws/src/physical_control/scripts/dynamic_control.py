#! /usr/bin/env python3.8

import rospy 
import numpy as np
import getch as inp
import modern_robotics as mr
from sensor_msgs.msg import JointState
from control_msgs.msg import JointControllerState

class Listener():
    def __init__(self, yaw,angle1,angle2,angle3):
        self.yaw = yaw
        self.angle1 = angle1
        self.angle2 = angle2
        self.angle3 = angle3
    def yaw_callback(self, data):
        self.yaw = data.process_value
        return
    def angle1_callback(self,data):
        self.angle1 = data.process_value
    def angle2_callback(self,data):
        self.angle2 = data.process_value
    def angle3_callback(self,data):
        self.angle3 = data.process_value

    def get_yaw(self):
        return self.yaw
    def get_angle1(self):
        return self.angle1
    def get_angle2(self):
        return self.angle2
    def get_angle3(self):
        return self.angle3



class robotConfig():
    def __init__(self, slist, M,topicNames,msgs):
        self.slist = slist
        self.M = M
        self.tNames = topicNames
        self.msgs = msgs
        self.check_setup()
        self.yaw = 0
        self.theta1 = 0
        self.theta2 = 0
        self.theta3 = 0
        self.arrange_thetas()

    def check_setup(self):
        length = len(self.slist)
        if(len(self.tNames) != length):
            print("WARNING:\nLengths do not match slist")
    def get_slist(self):
        return self.slist
    def get_M(self):
        return self.M
    def get_tNames(self):
        return self.tNames
    def get_msgs(self):
        return self.msgs
    def arrange_thetas(self):
        """Sets up the theta list of the robot"""
        self.thetalist = np.array([
            self.yaw,
            self.theta1,
            self.theta2,
            self.theta3])

def get_angles(robot,listen):
    topics = robot.get_tNames()
    msgs = robot.get_msgs()
    ang0 = topics[0]
    msg0 = msgs[0]
    ang1 = topics[1]
    msg1 = msgs[1]
    ang2 = topics[2]
    msg2 = msgs[2]
    ang3 = topics[3]
    msg3 = msgs[3]

    rospy.Subscriber(ang0,msg0, listen.yaw_callback)
    rospy.Subscriber(ang1,msg1, listen.angle1_callback)
    rospy.Subscriber(ang2,msg2, listen.angle2_callback)
    rospy.Subscriber(ang3,msg3, listen.angle3_callback)
    robot.yaw = listen.get_yaw()
    robot.theta1 = listen.get_angle1()
    robot.theta2 = listen.get_angle2()
    robot.theta3 = listen.get_angle3()

    #Update the thetalist of the robot
    robot.arrange_thetas()
def setup():
    S0 = [0,0,1,0,0,0]
    S1 = [1,0,0,0,19,-14.5]
    S2 = [1,0,0,0,19,-134.5]
    S3 = [1,0,0,0,19,-254.5]

    slist = np.array([
        S0,
        S1,
        S2,
        S3
        ]).T
    M = np.array([
        [1,0,0,0],
        [0,1,0,350.97],
        [0,0,1,16.5],
        [0,0,0,1]
        ])
    topicNames = [
            "stubby_complete/yaw_position_controller/state",
            "stubby_complete/Rev1_position_controller/state",
            "stubby_complete/Rev2_position_controller/state",
            "stubby_complete/gripAngle_position_controller/state"
            ]
    msgs = [
            JointControllerState,
            JointControllerState,
            JointControllerState,
            JointControllerState
            ]
    stumpy = robotConfig(slist,M,topicNames,msgs)
    return stumpy
if __name__ == "__main__":
    np.set_printoptions(precision = True,suppress = True)
    try:
        rospy.init_node("dynamic_control", anonymous=True)
        
        rospy.Rate(10)
        listener = Listener(0,0,0,0)
        print("Active: Robot Angles:")
        stumpy = setup()
        while not rospy.is_shutdown():
            
            print("{} {} {} {}".format(
                stumpy.yaw, 
                stumpy.theta1,
                stumpy.theta2,
                stumpy.theta3
                ))
            
            get_angles(stumpy,listener)
            T = mr.FKinSpace(stumpy.M,stumpy.slist,stumpy.thetalist)
            print("Current Position:\n {}".format(T))
    except rospy.ROSInterruptException:
        pass
