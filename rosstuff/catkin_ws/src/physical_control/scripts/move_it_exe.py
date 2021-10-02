#! /usr/bin/env python3.8

import rospy 
import numpy as np
import getch as inp
import modern_robotics as mr
from sensor_msgs.msg import JointState

class Listener():
    def __init__(self, yaw,angle1,angle2,angle3):
        self.yaw = yaw
        self.angle1 = angle1
        self.angle2 = angle2
        self.angle3 = angle3
    def angles_callback(self,data):
        position = data.position
        self.yaw = position [3]
        self.angle1 = position[2]
        self.angle2 = position[1]
        self.angle3 = position[0]
    def yaw_callback(self, data):
        self.yaw = data.process_value
        return
    def angle1_callback(self,data):
        self.angle1 = data.process_value
    def angle2_callback(self,data):
        self.angle2 = data.process_value
    def angle3_callback(self,data):
        self.angle3 = data.process_valuea
    def rviz_callback(self, data):
        self.data = data
    def get_data(self):
        return self.data

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

    def check_setup(self):
        length = len(self.slist)
    def get_slist(self):
        return self.slist
    def get_M(self):
        return self.M
    def get_tNames(self):
        return self.tNames
    def get_msgs(self):
        return self.msgs
    def get_angles(self,listener):
        self.yaw = listener.get_yaw()
        self.theta1 = listener.get_angle1()
        self.theta2 = listener.get_angle2()
        self.theta3 = listener.get_angle3()
        """Sets up the theta list of the robot"""
        self.thetalist = np.array([
            self.yaw,
            self.theta1,
            self.theta2,
            self.theta3])
        return self.thetalist
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
        [0,1,0,335.8],
        [0,0,1,19],
        [0,0,0,1]
        ])
    topicNames = [
            "/joint_states"
            ]
    msgs = [
            JointState
            ]
    stumpy = robotConfig(slist,M,topicNames,msgs)
    return stumpy

class RobotControl():
    def __init__(self, pub):
        self.pub = pub

    def reset_configuration(self):
        """Resets the robot condition to a defined zero condition"""
        print("reset configuration")
        joints  = JointState()
        joints.name = ["yaw", "Rev1", "Rev2", "gripAngle"]
        joints.position = [0, 1.57, -1.57, -1.57]
        joints.velocity = [1,1,1,1]
        self.pub.publish(joints)
    def set_angles(self, thetalist):
        
        joints  = JointState()
        joints.name = ["yaw", "Rev1", "Rev2", "gripAngle"]
        joints.position = thetalist.tolist()
        print(joints.position)
        joints.velocity = [1,1,1,1]
        self.pub.publish(joints)
    def set_position(self, recievedData):
        """Sets the position based on the data"""
        self.pub.publish(recievedData)
    def set_configuration(self, robot,desiredConfig):
        """Sets the configuration to the desiredConfig with expectation
        to Reach there in 1 second"""
        pass
#########################MAIN############################

if __name__ == "__main__":
    np.set_printoptions(precision = True,suppress = True)
    SETUP_FLAG = 0
    try:
        rospy.init_node("dynamic_control", anonymous=True)
        rate = rospy.Rate(10)
        
        armPublisher = rospy.Publisher(
                "/desired_joint_states",
                JointState,
                queue_size = 10)
        RC = RobotControl(armPublisher)
        listener = Listener(0,0,0,0)
        topic = "/move_group/fake_controller_joint_states"
        rospy.Subscriber(topic, JointState,
                listener.rviz_callback)
        print("Initial Setup delay")
        rospy.sleep(4)
        stumpy = setup()
        rospy.sleep(2)
        startTime = rospy.get_rostime().secs
        while not rospy.is_shutdown():
            joints = listener.get_data()
            joints.velocity = [0.75,0.75,0.75,0.75]
            print(joints)
            RC.set_position(joints)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
