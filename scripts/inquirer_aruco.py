#!/usr/bin/env python3

from matplotlib import markers
from soupsieve import SelectorSyntaxError
import rospy
import numpy as np
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Pose
# from tf.transformations import quaternion_from_euler
from arm_vision.msg import ArucoPoses
import tf2_ros
import tf2_msgs
from geometry_msgs.msg import TransformStamped

found_arucos=[]
reference_frame=''

markers_dict={'button 1':           1,
              'button 2':           2,
              'button 3':           3,
              'button 4':           4,
              'button 5':           5,
              'button 6':           6,
              'button 7':           7,
              'button 8':           8,
              'button 9':           9,
              'imu':                10,
              'imu storage':        11,
              'inspection panel':   12,
              'lid':                13,
              'lid storage':        14
              }
def nameOfReference(id):
    global markers_dict
    for name, age in markers_dict.iteritems():
        if age == id:
            return name

def foundMarkersCalback(aruco_msg):
    global found_arucos
    for current_id,current_pose in zip(aruco_msg.ids,aruco_msg.poses):
        found_arucos.append((current_id,current_pose))
    # found_arucos[ii][1].position.x eg. is accessible

def getReferencePosition():
    global reference_frame

def tfBroadcastCallback(_):
    global found_arucos

def arucoInquirer():
    global CAMERA_MATR
    CAMERA_MATR=np.ndarray
    global CAMERA_DIST_COEFF
    CAMERA_DIST_COEFF=np.ndarray
    global CAMERA_FOCAL_LEN
    CAMERA_FOCAL_LEN=np.ndarray
    TIMER_DURATION=rospy.Duration(nsecs=2E7)
    global aruco_topic
    aruco_topic='/aruco_poses'
    global aruco_sub
    aruco_sub=rospy.Subscriber(aruco_topic,ArucoPoses,foundMarkersCalback)
    tf_timer=rospy.Timer(TIMER_DURATION,tfBroadcastCallback)

if __name__=='__main__':
    print("inquirer")
    node_name="inquirer"    
    rospy.init_node(node_name,anonymous=False)
    
    arucoInquirer()

    try:
        rospy.spin()

    except KeyboardInterrupt:
        # rospy.signal_shutdown('Esc key pressed; Closing node: {}'.format(node_name))
        print('bye')