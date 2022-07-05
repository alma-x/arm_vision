#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState,CameraInfo
from geometry_msgs.msg import Pose
# from tf.transformations import quaternion_from_euler
from arm_vision.msg import ArucoPoses
import tf2_ros
from tf2_msgs.msg import TFMessage
from tf import transformations
from geometry_msgs.msg import TransformStamped

# CAMERA_MATR=np.ndarray
# CAMERA_DIST_COEFF=np.ndarray
# CAMERA_FOCAL_LEN=np.ndarray

aruco_topic='/aruco_poses'
base_frame='base_link'
camera_frame='camera_link'
joint_states_topic='/joint_states'

tf_buffer=None
tf_listener=None
tf_broadcaster=None
joint_velocities=[]



markers_dict={'button_1':           1,
              'button_2':           2,
              'button_3':           3,
              'button_4':           4,
              'button_5':           5,
              'button_6':           6,
              'button_7':           7,
              'button_8':           8,
              'button_9':           9,
              'imu_':               10,
              'imu_wall':           11,
              'inspection_panel':   12,
              'lid_':               13,
              'lid_storage':        14
              }

arucos_in_sight=[]
found_arucos=[[xx,False] for xx in range(1,15)]


def nameOfMarkerReference(id):
    global markers_dict
    for marker_name, marker_id in markers_dict.items():
        if marker_id == id:
            return marker_name


def getMarkersInSight(aruco_msg):
    global arucos_in_sight
    for current_id,current_pose in zip(aruco_msg.ids,aruco_msg.poses):
        arucos_in_sight.append((current_id,current_pose))
    # arucos_in_sight[ii][1].position.x eg. is accessible


def transformCameraToReference(target_frame):
    global tf_buffer
    # has .transform.{translation.{x,y,z},rotation.{x,y,z,w}
    while True:
        try:
            return tf_buffer.lookup_transform(base_frame,target_frame,rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, \
            tf2_ros.ExtrapolationException): continue


# def poseErrorTooBig(current_pose,broadcasted_pose):
#TODO: threshold is too high
#     TRANSLATION_THRESHOLD=1
#     error_translation=np.sqrt(np.square(current_pose.position.x-broadcasted_pose.translation.x)+\
#                 np.square(current_pose.position.y-broadcasted_pose.translation.y)+\
#                     np.square(current_pose.position.z-broadcasted_pose.translation.z))

#     # error_rotation=np.sqrt(np.square(current_pose.position.x-broadcasted_pose.rotation.x)+\
#     #             np.square(current_pose.position.y-broadcasted_pose.rotation.y)+\
#     #                 np.square(current_pose.position.z-broadcasted_pose.rotation.z))
#     if error_translation>TRANSLATION_THRESHOLD: return True
#     return False


def getVelocities(js_msg):
    global joint_velocities
    current_velocity=js_msg.velocity
    joint_velocities=np.roll(joint_velocities,1)
    joint_velocities[0]=current_velocity
    


def velocityTooBig():
    for current_component in range(len(joint_velocities.T)):
        current_speed=joint_velocities.T[current_component]
        current_max=np.max(current_speed)
        current_mean=np.mean(current_speed[np.nonzero(current_speed)])
        if current_max>1.2*current_mean: return True
    return False


def broadcastTFMarkers(_):
    global tf_broadcaster
    global found_arucos
    tfs_to_broadcast=[]

    if not velocityTooBig():
        for current_id,current_pose in arucos_in_sight:
            if not found_arucos[current_id-1][1]:
                tf_stamped=TransformStamped()
                tf_stamped.header.stamp=rospy.Time.now()
                current_frame=nameOfMarkerReference(current_id)
                tf_stamped.child_frame_id=current_frame

                tf_stamped.header.frame_id=camera_frame
                tf_stamped.transform.translation.x=current_pose.position.x
                tf_stamped.transform.translation.y=current_pose.position.y
                tf_stamped.transform.translation.z=current_pose.position.z
                tf_stamped.transform.rotation.x=current_pose.orientation.x
                tf_stamped.transform.rotation.y=current_pose.orientation.y
                tf_stamped.transform.rotation.z=current_pose.orientation.z
                tf_stamped.transform.rotation.w=current_pose.orientation.w
                tfs_to_broadcast.append(tf_stamped)

        if tfs_to_broadcast:
            tf_broadcaster.sendTransform(tfs_to_broadcast)
            try:
                for current_tf in tfs_to_broadcast:
                    current_frame=current_tf.child_frame_id
                    new_tf=transformCameraToReference(current_frame)
                    tf_broadcaster.sendTransform(new_tf)
                    current_id=markers_dict.get(current_frame)
                    if not found_arucos[current_id-1][1]:
                        found_arucos[current_id-1][1]=True
                        print('broadcasting tf for: {} (markerd id: {})'.
                            format(current_frame,current_id))
            except (tf2_ros.LookupException, \
                tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass

#TODO still does not work well, as for now i will simply broadcast tf when the robot is moving slow
# def refineTFMarkers(_):
#     for current_id,current_pose in arucos_in_sight:
#         current_frame=nameOfMarkerReference(current_id)
#         try:
#             broadcasted_pose=tf_buffer.lookup_transform(base_frame,current_frame,rospy.Time()).transform
#             if poseErrorTooBig(current_pose,broadcasted_pose):
#                 found_arucos[current_id-1][1]=False
#                 print('transform for {} (id: {}) scheduled for refinement'.
#                     format(current_frame,current_id))

#         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, \
#             tf2_ros.ExtrapolationException): pass
#----------------------------------------------------------

def arucoInquirer():
    global joint_velocities
    joint_velocities=np.zeros(\
        (50,len(rospy.wait_for_message(joint_states_topic,JointState).velocity)))

    global aruco_sub
    aruco_sub=rospy.Subscriber(aruco_topic,ArucoPoses,getMarkersInSight)

    global tf_buffer
    global tf_listener
    global tf_broadcaster
    tf_buffer=tf2_ros.Buffer(cache_time=rospy.Duration(1))
    tf_listener=tf2_ros.TransformListener(tf_buffer,queue_size=None)
    #COULD ADD WAIT_FOR_SERVICE/_FOR_SERVER TO PREVENT ERRORS
    tf_broadcaster=tf2_ros.StaticTransformBroadcaster()

    TIMER_DURATION=rospy.Duration(nsecs=2E6)
    tf_timer=rospy.Timer(TIMER_DURATION,broadcastTFMarkers)
    # refinement_timer=rospy.Timer(5*TIMER_DURATION,refineTFMarkers)

    velocity_sub=rospy.Subscriber(joint_states_topic,JointState,getVelocities,queue_size=1)

#############################################################

if __name__=='__main__':

    node_name="referencer"
    print('node name: {}\n publishes coordinates transforms for found markers'.
        format(node_name))

    rospy.init_node(node_name,anonymous=False)
    arucoInquirer()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.signal_shutdown('Esc key pressed; Closing node: {}'.format(node_name))