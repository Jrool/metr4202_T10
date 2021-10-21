#! /usr/bin/env python3.8

import rospy

import tf
import tf2_ros as tf2
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from fiducial_msgs.msg import FiducialTransformArray
from fiducial_msgs.msg import FiducialTransform
from std_msgs.msg import Header
from tf2_ros import transform_broadcaster
import tf2_ros
#from geometry_msgs.msg import float64

class Listener():
    def __init__(self):
        self.transforms =  None 
        self.set_static = 0

    def camera_callback(self, data : FiducialTransform):
        """OBtains the camera frame based on the initial Tsc"""
        if (len(data.transforms) == 0):
            return
        self.transforms = data.transforms
        tfPub = rospy.Publisher("/tf_static", TFMessage, queue_size=1)
        for item in data.transforms:
            if ((item.fiducial_id == 42) and (self.set_static == 0)):
                #print(item) ROS
                bcaster = tf2.TransformBroadcaster()
                staticBcaster = tf2.StaticTransformBroadcaster()
                t = TransformStamped()
                tfBuffer = tf2.Buffer()
                listener = tf2.TransformListener(tfBuffer)
                parent = "fixed_id5"
                child = "camera_temp"
                t.child_frame_id = child
                t.header.frame_id = parent
                t.transform.translation.x = item.transform.translation.x
                t.transform.translation.y = item.transform.translation.y
                t.transform.translation.z = item.transform.translation.z

                t.transform.rotation.x = item.transform.rotation.x
                t.transform.rotation.y = item.transform.rotation.y
                t.transform.rotation.z = item.transform.rotation.z
                t.transform.rotation.w = -item.transform.rotation.w
            
                #tfm = TFMessage([t])
                #tfPub.publish(tfm)
                rate = rospy.Rate(50)
                bcaster.sendTransform(t)            
                #EVERYTHING BELOW UNSURE
            
                while not rospy.is_shutdown():
                    try:
                        trans = tfBuffer.lookup_transform(child ,"space_frame", rospy.Time())
                    except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException):
                        rate.sleep()
                        continue
                    break
                child = "0"
                parent = "space_frame"
                t = TransformStamped()
                print(trans)
                t.child_frame_id = child
                t.header.frame_id = parent
                t.transform.translation.x = trans.transform.translation.x
                t.transform.translation.y = trans.transform.translation.y
                t.transform.translation.z = trans.transform.translation.z

                t.transform.rotation.x = trans.transform.rotation.x
                t.transform.rotation.y = trans.transform.rotation.y
                t.transform.rotation.z = trans.transform.rotation.z
                t.transform.rotation.w = trans.transform.rotation.w
                x = input("Publish y/N?\n")
                if(x != "y"):
                    return
                staticBcaster.sendTransform(t)
                self.set_static = 1
                exit(0)

    def get_transforms(self):
        """returns the transform"""
        return self.transforms

def tf_callback(data):
    br = tf.TransformBroadcaster()
   
    if(len(data.transforms) == 0):
        return
    else:
        for item in data.transforms:
            child = item.child_frame_id
            parent = item.header.frame_id
            
            trans = item.transform
            #print(trans)
            br.sendTransform( (trans.translation.x, trans.translation.y, trans.translation.z),
                (trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w),
                rospy.Time.now(),
                 child, parent)
            


def aurco_frame(x,y,z,rx,ry,rz):
    """Develops an aruco tag transformation frame with respect to the joint
    compinent9_1
    ARGUMENTS:
        -x: the x location
        -y: the y location
        -z: the z location
        -r*: the rotations about axis * in Euler angles"""

    br = tf2.StaticTransformBroadcaster()
    trans = TransformStamped()
    parent = "Component9_1"
    child = "fixed_id5"

    trans.header.stamp = rospy.Time.now()
    trans.header.frame_id = parent
    trans.child_frame_id = child
    trans.transform.translation.x = x
    trans.transform.translation.y = y
    trans.transform.translation.z = z
    rotations = tf.transformations.quaternion_from_euler(rx,ry,rz)    
    trans.transform.rotation.x = rotations[0]
    trans.transform.rotation.y = rotations[1]
    trans.transform.rotation.z = rotations[2]
    trans.transform.rotation.w = rotations[3]
    br.sendTransform(trans)

def base_frame(x,y,z,rx,ry,rz):
    """Develops an base frame transformation frame with respect to the joint
    compinent2_1
    ARGUMENTS:
        -x: the x location
        -y: the y location
        -z: the z location
        -r*: the rotations about axis * in Euler angles"""

    br = tf2.StaticTransformBroadcaster()
    trans = TransformStamped()
    parent = "Component2_1"
    child = "space_frame"

    trans.header.stamp = rospy.Time.now()
    trans.header.frame_id = parent
    trans.child_frame_id = child
    trans.transform.translation.x = x
    trans.transform.translation.y = y
    trans.transform.translation.z = z
    rotations = tf.transformations.quaternion_from_euler(rx,ry,rz)    
    trans.transform.rotation.x = rotations[0]
    trans.transform.rotation.y = rotations[1]
    trans.transform.rotation.z = rotations[2]
    trans.transform.rotation.w = rotations[3]
    br.sendTransform(trans)

def send_transform(item, parent, child, static=False):
    """Sends the transform off to the broadcaster 
    INPUTS:
        -item:<FiducialTransform> type recieved from camera"""
    if static:
        br = tf2.StaticTransformBroadcaster()
    else:
        br = tf2.TransformBroadcaster()
    trans = TransformStamped()
    print(item)
    trans.header.frame_id = parent
    trans.child_frame_id = child
    trans.transform.translation.x = item.transform.translation.x 
    trans.transform.translation.y = item.transform.translation.y
    trans.transform.translation.z = item.transform.translation.z
    trans.transform.rotation.x =  item.transform.rotation.x
    trans.transform.rotation.y = item.transform.rotation.y
    trans.transform.rotation.z = item.transform.rotation.z
    trans.transform.rotation.w = item.transform.rotation.z
    br.sendTransform(trans)

# def base_frame_setup(listener):
#     """From the camera frame, the frame from space to camera can be
#     found"""
#     parent = "Component2_1"
#     child =  ""
#     tfBuffer = tf2.Buffer()
#     tfListener = tf2.TransformListener(tfBuffer)
#     while rospy.is_shutdown():
#         rospy.spin()

if __name__ == "__main__":
    rospy.init_node('tf_broadcaster')
    listener = Listener()
    rospy.Subscriber("/fiducial_transforms", FiducialTransformArray,
            listener.camera_callback)
    #base_frame_setup(listener)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        try:
            # aurco_frame(-1.5 * pow(10,-2), 0, 0, 0, 0, 0)
            #base_frame(0, 0, 3.4 * pow(10,2), 0, 0, 0)
            rospy.Subscriber("/tf", TFMessage, tf_callback)
            rospy.spin()
            
        except rospy.ROSInterruptException:
            pass
