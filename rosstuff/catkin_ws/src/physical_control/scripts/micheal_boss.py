#! /usr/bin/env python3.8

import rospy 
import numpy as np
import getch as inp
import modern_robotics as mr
from sensor_msgs.msg import JointState
from fiducial_msgs.msg import FiducialTransformArray
from fiducial_msgs.msg import FiducialTransform
#SETUP
class Listener():
    def __init__(self, yaw,angle1,angle2,angle3):
        self.yaw = yaw
        self.angle1 = angle1
        self.angle2 = angle2
        self.angle3 = angle3
        self.translationParameters = [0,0,0]
        self.rotationParameters = [0,0,0,0]
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
        self.angle3 = data.process_value
    def micheal_callback(self, data):
        if(len(data.transforms) == 0):
            return
        #self.translationParameters = [data.transforms[0].transform.translation.x, data.transforms[0].transform.translation.y, data.transforms[0].transform.translation.z]
        #self.rotationParameters = [data.transforms[0].transform.rotation.x,data.transforms[0].transform.rotation.y,data.transforms[0].transform.rotation.y,data.transforms[0].transform.rotation.z]

    def get_translation(self):
        return self.translationParameters
    def get_rotation(self):
        return self.rotationParameters
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
        [0,0,1,321.3],
        [0,-1,0,19],
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
        joints.position = [0 + 6.15* np.pi/180, 1.57, -1.57, -1.57]
        joints.velocity = [1,1,1,1]
        self.pub.publish(joints)
    def set_angles(self, thetalist):
        """Sets the joints identified by the numpy array thetalist"""
        joints  = JointState()
        joints.name = ["yaw", "Rev1", "Rev2", "gripAngle"]
        joints.position = thetalist
        joints.position[0] = thetalist[0] + 6.15 *np.pi/180
        joints.velocity = [1,1,1,1]
        self.pub.publish(joints)

    def set_coordinates(self, targetCoords, robot):
        #TODO:
        #   -Fix the orientation of the gripper, currently, the y axis in direc
        #   tion z should be
        #   
        #   -Figure out why negative coordinates do not work with the analytical

        """ Using the analytical inverse kinematics, the robot is positioned 
        to the desired coordinates such that the arm is constrained within the
        range determined:
        A bad solution is checked
        Inputs:
            targetCoords: list of target coordinates in the form [x,y,z]  """
        x = targetCoords[0]
        y = targetCoords[1] 
        z = targetCoords[2]

        yaw = np.arctan2(-x,y)
        y_offset = 14.5 * np.cos(yaw)
        x_offset = 14.5 * np.sin(yaw)
        z_offset = 19
        gripRange = np.linspace(-2.1,2)
        revRange = np.linspace(-2.5, 2)
        
        L1 = 120
        L2 = 120
        L3 = 81.3
        #Define constraints on the arm itself, the first guess will be
        #in the 
        upper = 0 *np.pi/180
        lower = -135 * np.pi/180

        phi_3 = -1.57
        #First check, if no solution possible, then adjust these until there is
        #a solution => Check the triangle inequality first
        solutionFlag = 0
        increment = 0.5 * np.pi/180
        print("x: {} y: {} z: {}".format(x, y, z))
        while (phi_3 < upper):
            x2 = x - x_offset
            z2 = z - L3 * np.sin(phi_3) - z_offset
            y2 = y - L3 * np.cos(phi_3) - y_offset
            print("phi_3: {}\nZ2: {} Y2: {}".format(phi_3,z2, y2))
            if (np.sqrt(z2**2 + y2**2) > (z2 + y2)):
                phi_3+= increment
                print("Outside Workspace grip = {}".format(phi_3))
                solutionFlag = 0
                continue

            c_theta2 = (z2**2 + y2**2 + x2**2 -L1**2 - L2**2)/(2* L1 *L2)#TODO: Verify
            if(c_theta2 > 1):
                phi_3+= increment
                solutionFlag = 0
                continue
            theta_2 = np.arctan2(-np.sqrt(1 - c_theta2**2), c_theta2) #TODO: pm
            theta_1 = np.arctan2(z2,y2) - \
                np.arctan2( L2 * np.sin(theta_2), L1 + L2 * np.cos(theta_2))
            theta_3 = phi_3 - theta_2 - theta_1
            solutionFlag = 1
            if ((theta_3 < -2.1) or (theta_3 > 2)\
                    or (theta_2 < -2.5) or (theta_2 > 2)):
                phi_3 += increment
                solutionFlag = 0
                continue
            break

        if (solutionFlag == 0):
            print("Unable to find solution\n")
            return False
        thetalist = [yaw, theta_1, theta_2, theta_3]
        self.set_angles(thetalist)
        #Use this as an initial guess for newton rapssona
        """
        eomg  = 0.5
        ev = 10
        phi_z = np.arctan2(-x,y)
        print(phi_z)
        phi_x = -np.pi #Initial guess for the phi_x, will adjust if cant find sol
        while ( phi_x < -np.pi/2):
            z_rot = mr.VecTose3(np.array([0,0,1,0,0,0]))
            x_rot = mr.VecTose3(np.array([1,0,0,0,0,0]))
            R = mr.MatrixExp6(z_rot * phi_z) * mr.MatrixExp6(x_rot * phi_x)
            T= np.array([
                [R[0][0], R[0][1], R[0][2], x],
                [R[1][0], R[1][1], R[1][2], y],
                [R[2][0], R[2][1], R[2][2], z],
                [0,0,0,1]
                ])
            result = mr.IKinSpace(robot.slist, robot.M, T, 
                    np.array(thetalist), eomg, ev)
            print(result)
            if(result[1] == False):
                print(T)
                self.set_angles(result[0])
                break
                phi_x += increment * 5
                continue
            else:
                self.set_angles(result[0])
                break
        if result[1] == False:
            print("Error: Newton Raphson could not find a solution\n")
        print(result[0])
        """
        return True

    def set_configuration(self, robot,desiredConfig):
        """Sets the configuration to the desiredConfig with expectation
        to Reach there in 1 second, uses Newon Rhapson"""
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
        topic = "/joint_states"
        rospy.Subscriber(topic, JointState,
                listener.angles_callback)
        rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, listener.micheal_callback)
        print("Initial Setup delay")
        rospy.sleep(4)
        stumpy = setup()
        RC.reset_configuration()
        rospy.sleep(2)
        startTime = rospy.get_rostime().secs
        while not rospy.is_shutdown():
            stumpy.get_angles(listener)
            T = mr.FKinSpace(stumpy.M,stumpy.slist,stumpy.thetalist)
            print("Current Position:\n {}".format(T))
            #Demo Desired positiona
            currentTime = rospy.get_rostime().secs - startTime
            print(currentTime)
            """ NEWTON RAPSON
            Td = np.array([
                [0.8,0.1,-0.6,0],
                [0.6,0.1,0.8,100],
                [0, -1, 0.1, 0],
                [0,0,0,1]])
            eomg = 0.01
            ev = 0.01
            thetalist0 = stumpy.get_angles(listener)
            reqTheta = mr.IKinSpace(stumpy.slist, stumpy.M, Td, thetalist0,eomg,ev)
            print(reqTheta)
            RC.set_angles(reqTheta[0])
            """
            print(listener.get_translation, listener.get_rotation)
            x = float(input("Enter x:\n"))
            y = float(input("Enter y:\n"))
            z = float(input("Enter z:\n"))
            target = [x,y,z]
            RC.set_coordinates(target, stumpy)
            rospy.sleep(5)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
