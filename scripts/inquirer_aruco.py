#!/usr/bin/env python3

from unicodedata import name
from matplotlib import markers
from soupsieve import SelectorSyntaxError
import rospy
import numpy as np
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Pose
# from tf.transformations import quaternion_from_euler
from arm_vision.msg import ArucoPoses
import tf2_ros
from tf2_msgs.msg import TFMessage
from tf import transformations
from geometry_msgs.msg import TransformStamped

CAMERA_MATR=np.ndarray
CAMERA_DIST_COEFF=np.ndarray
CAMERA_FOCAL_LEN=np.ndarray
found_arucos=[]
base_frame='base_link'
camera_frame='camera_link'
tf_buffer=None
tf_listener=None
tf_broadcaster=None
tf_publisher=None

#COULD ADD WAIT_FOR_SERVICE/_FOR_SERVER TO PREVENT ERRORS


markers_dict={'button_1':           1,
              'button_2':           2,
              'button_3':           3,
              'button_4':           4,
              'button_5':           5,
              'button_6':           6,
              'button_7':           7,
              'button_8':           8,
              'button_9':           9,
              'imu_':                10,
              'imu_panel':        11,
              'inspection_panel':   12,
              'lid_':                13,
              'lid_storage':        14
              }

def nameOfMarkerReference(id):
    global markers_dict
    for marker_name, marker_id in markers_dict.items():
        if marker_id == id:
            return marker_name

def getFoundMarkers(aruco_msg):
    global found_arucos
    for current_id,current_pose in zip(aruco_msg.ids,aruco_msg.poses):
        found_arucos.append((current_id,current_pose))
    # found_arucos[ii][1].position.x eg. is accessible

def transformToReference(targer_frame):
    # better to obtain aruco_tf first, then transform it into base to use it
    global base_frame
    # global camera_frame
    global tf_buffer
    global tf_listener
    # has .transform.{translation.{x,y,z},rotation.{x,y,z,w}
    tf_tf=tf_buffer.lookup_transform(base_frame,targer_frame,rospy.Time())
    tf_tf.header.stamp=rospy.Time.now()
    tf_tf.header.frame_id=base_frame


def broadcastTFMarkers(_):
    # global found_arucos
    # global camera_frame
    # global tf_broadcaster
    for current_id,current_pose in found_arucos:
        tf_stamped=TransformStamped()
        current_frame=nameOfMarkerReference(current_id)
        tf_stamped.header.stamp=rospy.Time.now()
        tf_stamped.header.frame_id=camera_frame
        tf_stamped.child_frame_id=current_frame
        tf_stamped.transform.translation.x=current_pose.position.x
        tf_stamped.transform.translation.y=current_pose.position.y
        tf_stamped.transform.translation.z=current_pose.position.z
        tf_stamped.transform.rotation.x=current_pose.orientation.x
        tf_stamped.transform.rotation.y=current_pose.orientation.y
        tf_stamped.transform.rotation.z=current_pose.orientation.z
        tf_stamped.transform.rotation.w=current_pose.orientation.w
        tf_broadcaster.sendTransform(tf_stamped)
        # try:
        #     tf_stamped=transformToReference(current_frame)
        #     tf_broadcaster.sendTransform(tf_stamped)
        # except (tf2_ros.LookupException, \
        #     tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     pass

def arucoInquirer():
    global CAMERA_MATR
    global CAMERA_DIST_COEFF
    global CAMERA_FOCAL_LEN
    global aruco_topic
    global aruco_sub
    global tf_buffer
    global tf_listener
    global tf_broadcaster
    global tf_publisher
    TIMER_DURATION=rospy.Duration(nsecs=2E7)
    aruco_topic='/aruco_poses'
    
    tf_buffer=tf2_ros.Buffer(cache_time=None)
    tf_listener=tf2_ros.TransformListener(tf_buffer,queue_size=None)
    tf_broadcaster=tf2_ros.StaticTransformBroadcaster()
    aruco_sub=rospy.Subscriber(aruco_topic,ArucoPoses,getFoundMarkers)
    tf_timer=rospy.Timer(TIMER_DURATION,broadcastTFMarkers)

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