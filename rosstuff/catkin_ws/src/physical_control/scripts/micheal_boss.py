#! /usr/bin/env python3.8

from numpy.core.fromnumeric import put
import rospy 
import numpy as np
import getch as inp
import modern_robotics as mr
from rospy.topics import Publisher
from sensor_msgs.msg import JointState
from fiducial_msgs.msg import FiducialTransformArray
from fiducial_msgs.msg import FiducialTransform
from std_msgs.msg import Header, Float32, Int16MultiArray
from geometry_msgs.msg import TransformStamped
import helper_functions as hf
import tf_broadcast as broadcast
import tf_conversions
import tf2_ros as tf2
import tf.transformations 

#SETUP

class Listener():
    def __init__(self, yaw,angle1,angle2,angle3):
        self.yaw = yaw
        self.angle1 = angle1
        self.angle2 = angle2
        self.angle3 = angle3
        self.transforms = None
        self.colour_array = None
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
    
    def fiducial_callback(self, data):
        if(len(data.transforms) == 0):
            return
        self.transforms = data.transforms
    def get_transforms(self):
        return self.transforms
    
    def get_yaw(self):
        return self.yaw
    
    def get_angle1(self):
        return self.angle1
    
    def get_angle2(self):
        return self.angle2
    
    def get_angle3(self):
        return self.angle3
    def colour_callback(self,data):
        print(data)
        self.colour_array = data
        
    
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
        self.fiducial_blacklist = [42]

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

    def reset_configuration(self, servo):
        """Resets the robot condition to a defined zero condition"""
        print("reset configuration")
        
        joints  = JointState()
        joints.name = ["yaw", "Rev1", "Rev2", "gripAngle"]
        joints.position = [0 + 6.15* np.pi/180, 1.57, -2.037, -2.1]
        joints.velocity = [1,1,1,1]
        self.pub.publish(joints)
        rospy.sleep(1)
        servo.publish(3)
        joints.position = [0 + 6.15* np.pi/180, 2.1138, -2.037, -2.1]
        self.pub.publish(joints)
    def set_angles(self, thetalist):
        """Sets the joints identified by the numpy array thetalist"""
        joints  = JointState()
        joints.name = ["yaw", "Rev1", "Rev2", "gripAngle"]
        joints.position = thetalist
        joints.position[0] = thetalist[0] #+ 6.15 *np.pi/180
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
            #print("phi_3: {}\nZ2: {} Y2: {}".format(phi_3,z2, y2))
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
        
        return True

    def set_configuration(self, robot,desiredConfig):
        """Sets the configuration to the desiredConfig with expectation
        to Reach there in 1 second, uses Newon Rhapson"""
        pass

    def set_red_zone(self, servo: rospy.Publisher):
        """Places the robot in the yellow zone for effective placement", drops the angle
        afterwards to place the block down"""
        thetalist = [1.96,1.213,-1.453,-1.0595]

        self.set_angles(thetalist)
        rospy.sleep(4)
        servo.publish(1)
        rospy.sleep(1)
    def set_yellow_zone(self, servo : rospy.Publisher):
        #thetalist = [-0.1945, 2.2725, 0.96223, 1.556]
        thetalist = [-0.21, 1.57, 1.16223, 1.556]
        self.set_angles(thetalist)
        rospy.sleep(2)
        thetalist = [-0.21, 2, 1.16223, 1.556]
        self.set_angles(thetalist)
        rospy.sleep(2)
        servo.publish(1)
        thetalist = [-0.21, 1.57, 0.96223, 1.556]
        self.set_angles(thetalist)
        rospy.sleep(2)

    def set_green_zone(self, servo: rospy.Publisher):
        thetalist = [0.31733, 1.57, 1.16223, 1.556]
        self.set_angles(thetalist)
        rospy.sleep(2)
        thetalist = [0.31733, 2.2, 1.16223, 1.556]
        self.set_angles(thetalist)
        rospy.sleep(2)
        
        thetalist = [0.38387, 2.1957, 1.3307, 1.136]
        self.set_angles(thetalist)
        servo.publish(1)
        rospy.sleep(1)
        thetalist = [0.31733, 1.57, 1.16223, 1.556]
        self.set_angles(thetalist)
        rospy.sleep(1)
    
    def set_blue_zone(self, servo:rospy.Publisher):
        thetalist = [1.1464919, 1.57, 1.16223, 1.556]
        self.set_angles(thetalist)
        rospy.sleep(2)
        thetalist = [1.1464919, 2.1725, 1.16223, 1.556]
        self.set_angles(thetalist)
        rospy.sleep(2)
        servo.publish(1)
        rospy.sleep(1)
    def make_veritcal(self, servo: rospy.Publisher):
        thetalist = [0.10734, 1.57, 0, 0.7370305345665205]
        self.set_angles(thetalist)
        rospy.sleep(5)

def update_transform(newTransform : FiducialTransform, parentFrame, childname = None):
    """Updates the frame transform with respect to the parent frame
    establishes it as the name given in the fiducial id
    INPUTS: 
        -newTransform: <FiducialTransform>: The transform to be published
        -parentFrame: <str> The frame to define it in terms of (Typically camera)
    """
    br = tf2.TransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "camera"
    if(childname == None):
        t.child_frame_id = str(newTransform.fiducial_id)
    else:
        t.child_frame_id = str(childname)
    t.transform.translation.x = newTransform.transform.translation.x
    t.transform.translation.y = newTransform.transform.translation.y
    t.transform.translation.z = newTransform.transform.translation.z

    t.transform.rotation.x = newTransform.transform.rotation.x
    t.transform.rotation.y = newTransform.transform.rotation.y
    t.transform.rotation.z = newTransform.transform.rotation.z
    t.transform.rotation.w = newTransform.transform.rotation.w
    br.sendTransform(t)

def camera_frame_setup(listener,tfBuffer):
    """From the camera frame, the frame from space to camera can be
    found
    Inputs:
        -listener <Listener>: Class that is used in broadcasting"""
    while not rospy.is_shutdown():

        if(listener.get_transforms() is not None):
            transforms = listener.get_transforms()
            for item in transforms:
                print("here")
                if (item.fiducial_id == 42):
                    
                    print("ID5 found")
                    x = input("Continue (C) or retry(R)?\n")
                    
                    if(x == "R"):
                        continue
                    elif(x == "C"):
                        pass
                    else: 
                        print("Invalid Option")
                        continue

                    #for i in range(0,5):
                    #update_transform(item,"fixed_id5", childname="camera")
                    #broadcast.send_transform(item ,"fixed_id5","camera_1", True)
                    update_transform(item,"fixed_id5", childname="camera")              
                    while not rospy.is_shutdown():
                        try:
                            #Tsc = tfBuffer.lookup_transform("camera_1", "space_frame", rospy.Time(),timeout = rospy.Duration(4.0) )
                            #broadcast.send_transform(Tsc, "space_frame", "camera",True)
                            Tsc = tfBuffer.lookup_transform("camera", "space_frame", rospy.Time(),timeout = rospy.Duration(4.0) )
                            return
                        except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException) as e:
                            update_transform(item,"fixed_id5", childname="camera")
                    
                            print(e)
                            continue
                    return
            print("iterating")


def check_box_orientation(x, y, currentAng):
    #set approapriate angle ranges
    flag = False
    if currentAng<0:
        currentAng  = currentAng+2*np.pi
    if x<0:
        desiredAng = (np.pi/2 - np.abs(np.arctan(y/x)))
    else:
        desiredAng = -1*(np.pi/2 - np.abs(np.arctan(y/x)))
    print(x,y)
    #print("Desired Angle", desiredAng)
    fR = 2*np.pi
    possibleAng = [(desiredAng+0)%fR, (desiredAng+np.pi/2)%fR, (desiredAng+np.pi)%fR, (desiredAng+(3*np.pi/2))%fR]
    #print("possible Angles:", possibleAng)
    for i in possibleAng:
        print(currentAng)
        if ((i+0.3) > currentAng and currentAng > (i-0.3)):
            flag = True
            break
        else:
            print("Orientation Invalid")
            flag = False 
    return flag


def handle_boxes(listener : Listener,robot : robotConfig, RC: RobotControl, tfBuffer: tf2.Buffer, colourPub : Publisher):
    """In the event that there are more than one boxes detected in the 
    workspace, the robot will find the x, y,z and compute the angles to
    pick it up
    INPUTS:
        -listener: The listener class
        -robot: robot class
        -robotControl: robot  controller class"""
    servo = rospy.Publisher("/servo_state", Float32, queue_size=1)
    
    validTransforms = []
    moving = 0
    checked = 0
    print("Detected {} Markers".format(len(listener.transforms)))
    for marker in listener.get_transforms():
        
        if marker.fiducial_id in robot.fiducial_blacklist:
            continue
        # tempx = marker.transforms.transform.rotation.x
        # tempy = box.transforms.transform.rotation.y
        # tempz = box.transforms.transform.rotation.z
        # tempw = box.transforms.transform.rotation.w
        lastOmega = None
        angle = 10 * np.pi/180
        for i in range(0,4):
            if moving:
                return
            elif checked:
                break
            T = tfBuffer.lookup_transform("space_frame","fiducial_" + str(marker.fiducial_id), rospy.Time())
            #(trans, rot) = tfBuffer.lookup_transform("space_frame","fiducial_" + str(marker.fiducial_id), rospy.Time())
            thetas = hf.euler_from_quaternion(T.transform.rotation.x, T.transform.rotation.y, T.transform.rotation.z, T.transform.rotation.w)
            print("Thetaz: {}".format(thetas[2]))
            thetaZ = thetas[2]
            if (lastOmega is not None):
                if(thetaZ> lastOmega + angle) or (thetaZ < lastOmega - angle):
                    #It is moving
                    moving = 1
           
            lastOmega = thetas[2]
            rospy.sleep(0.5)
        print("Moving: {}".format(moving))
        checked = 1
        if (T.transform.translation.y < 0):
            print("y negative ({})".format(T.transform.translation.y))
            robot.fiducial_blacklist.append(marker.fiducial_id)
            continue
        print("Appending {}".format(marker.fiducial_id))
        validTransforms.append(marker.fiducial_id)
        break
        #Speed Stuff here check the speeds mabye go after only one box at a time
    for box in validTransforms:
        tries = 0
        #let [0,0,0] be invalid transforms
        #while (tries < 5) and (not rospy.is_shutdown()):
        try:
            
            x_found = 0
            y_found = 0
            z_found =0
            for i in range(0,5):
                Tsb = tfBuffer.lookup_transform("space_frame","fiducial_" + str(box), rospy.Time())
                x = (Tsb.transform.translation.x ) * pow(10,3)
                y = (Tsb.transform.translation.y) * pow(10,3)
                z = Tsb.transform.translation.z * pow(10,3)
                
            
            
            theta = np.arctan(x/y)
            
            parent = "space_frane"

            child =  "fiducial_" + str(box)
            
            # br = tf2.TransformBroadcaster()
            # trans = TransformStamped()
            # trans.header.frame_id = parent
            # trans.child_frame_id = child
            #get the quaternion for the detected aruco marker
            tempx = Tsb.transform.rotation.x
            tempy = Tsb.transform.rotation.y
            tempz = Tsb.transform.rotation.z
            tempw = Tsb.transform.rotation.w
            nu = hf.euler_from_quaternion(tempx,tempy,tempz,tempw)[2]
            

           
            x_off = 7.5#mm
            y_off = 2.5
            x += x_off
            
            y += y_off
            coords = [x,y,80]
            print(coords)

            
        
            print(check_box_orientation(x+30,y,nu))
            if check_box_orientation(x+30,y,nu) == 0:
                continue
            
            if (RC.set_coordinates(coords,robot) == False):
                continue
            rospy.sleep(2.5)
            servo.publish(1)
            coords = [x,y,0]
            RC.set_coordinates(coords,robot)
            
            rospy.sleep(2.5)
            servo.publish(0)
            rospy.sleep(2.5)
            coords = [x,y,60]
            RC.set_coordinates(coords,robot)
            rospy.sleep(1)
            RC.make_veritcal(servo)
            rospy.sleep(4)
            colourSend = Int16MultiArray()
            colourSend.data = [1,0]
            colourPub.publish(colourSend)
            rospy.sleep(3)
            #while ((listener.colour_array is None) or (listener.colour_array.data[0] == 1) and not rospy.is_shutdown()):
                #rospy.sleep(1)
            print("COLOUR: {}".format(listener.colour_array.data[1]))
            colour = listener.colour_array.data[1]
            if (colour == 0):
                RC.set_red_zone(servo)
            elif (colour == 1):
                RC.set_green_zone(servo)
            elif (colour == 2):
                RC.set_blue_zone(servo)
            elif (colour == 3):
                RC.set_yellow_zone(servo)
            RC.reset_configuration(servo)
            rospy.sleep(3)
            return
        except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException):
            print("Exception")
            


#def colour_dominant_colour():
               


#########################MAIN############################
if __name__ == "__main__":
    np.set_printoptions(precision = True,suppress = True)
    SETUP_FLAG = 0
    
    try:
        rospy.init_node("dynamic_control", anonymous=True)
        rate = rospy.Rate(50)

        tfBuffer = tf2.Buffer()
        tfListener = tf2.TransformListener(tfBuffer)
        armPublisher = rospy.Publisher(
                "/desired_joint_states",
                JointState,
                queue_size = 10)
        RC = RobotControl(armPublisher)
        listener = Listener(0,0,0,0)
        topic = "/joint_states"
        rospy.Subscriber(topic, JointState,
                listener.angles_callback)
        rospy.Subscriber("/fiducial_transforms", FiducialTransformArray,
                listener.fiducial_callback)
        servo = rospy.Publisher("/servo_state", Float32, queue_size=1)
        camera_colour_pos = rospy.Publisher("/colour_data", Int16MultiArray, queue_size=1)
        rospy.Subscriber("/colour_data", Int16MultiArray, listener.colour_callback)



        #################BEGIN PROGRAM####################
        print("Initial Setup delay")
        rospy.sleep(4)
        stumpy = setup()
        RC.reset_configuration(servo)
        rospy.sleep(2)
        startTime = rospy.get_rostime().secs
        x  = input("Press any key to continue")
        while not rospy.is_shutdown():
            stumpy.get_angles(listener)
            
            T = mr.FKinSpace(stumpy.M,stumpy.slist,stumpy.thetalist)
            #print("Current Position:\n {}".format(T))
            #In the event that the arm detects more than one target, it needs to handle
            # all of these markers
            
            if (len(listener.get_transforms()) >= 1):
                #Is moving??
                #Then do the handle boxes
                handle_boxes(listener,stumpy, RC, tfBuffer, camera_colour_pos) 
            
            
            #RC.set_coordinates(target, stumpy)
            print("detected {} markers".format(len(listener.get_transforms())))
            print("blacklist {}".format(stumpy.fiducial_blacklist))
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
