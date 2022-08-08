#!/usr/bin/env python3

import rospy
import numpy as np
from math import pi as PI
# from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Quaternion
# from tf.transformations import quaternion_from_euler
from arm_vision.msg import ArucoPoses,FoundArucos
from arm_vision.srv import ReferenceAcquisition,ReferenceAcquisitionResponse
import tf2_ros
# from tf2_msgs.msg import TFMessage
from tf.transformations import quaternion_about_axis
from geometry_msgs.msg import TransformStamped

aruco_topic='/camera_poses'

base_frame='base_link'
camera_frame='camera_link'
# joint_states_topic='/joint_states'
findings_topic='/found_arucos'

REFERENCES_SERVICE='references'

tf_buffer=None
tf_listener=None
tf_broadcaster=None
findings_pub=None
# joint_velocities=[]

# markers_dict={'button_1':           1,
#               'button_2':           2,
#               'button_3':           3,
#               'button_4':           4,
#               'button_5':           5,
#               'button_6':           6,
#               'button_7':           7,
#               'button_8':           8,
#               'button_9':           9,
#               'imu_':               10,
#               'left_panel':         11,
#               'inspection_panel':   12,
#               'lid_':               13,
#               'lid_storage':        14
#               }

markers_dict={'id_1':           1,
              'id_2':           2,
              'id_3':           3,
              'id_4':           4,
              'id_5':           5,
              'id_6':           6,
              'id_7':           7,
              'id_8':           8,
              'id_9':           9,
              'id_10':          10,
              'id_11':          11,
              'id_12':          12,
              'id_13':          13,
              'id_14':          14
              }

MAX_MID_PANEL_ARUCO_ID=4
# MAX_MID_PANEL_ARUCO_ID=9
objects_markers={'table_':            [10,14],
                #  'robot_frame':[id for id in range(1,MAX_MID_PANEL_ARUCO_ID+1)],
                 'mid_panel':[marker for marker in range (1,MAX_MID_PANEL_ARUCO_ID+1)],
                 'button_1': [marker for marker in range (1,MAX_MID_PANEL_ARUCO_ID+1)],
                'button_2':  [marker for marker in range (1,MAX_MID_PANEL_ARUCO_ID+1)],
                'button_3':  [marker for marker in range (1,MAX_MID_PANEL_ARUCO_ID+1)],
                'button_4': [marker for marker in range (1,MAX_MID_PANEL_ARUCO_ID+1)],
                'button_5': [marker for marker in range (1,MAX_MID_PANEL_ARUCO_ID+1)],
                'button_6':  [marker for marker in range (1,MAX_MID_PANEL_ARUCO_ID+1)],
                'button_7':  [marker for marker in range (1,MAX_MID_PANEL_ARUCO_ID+1)],
                'button_8':  [marker for marker in range (1,MAX_MID_PANEL_ARUCO_ID+1)],
                'button_9':  [marker for marker in range (1,MAX_MID_PANEL_ARUCO_ID+1)],
                'imu_':               [10],
                'left_panel':          [11],
                'right_panel':        [12,13],
                'inspection_panel':   [12,13],
                'lid_':               [13]}

MAX_ARUCO_ID=14
arucos_in_sight=[]
DEFAULT_IN_SIGHT=False
found_arucos=[[aruco_id,DEFAULT_IN_SIGHT,Pose()] for aruco_id in range(1,MAX_ARUCO_ID+1)]

TABLE_BROADCASTED=False
ROBOT_FRAME_BROADCASTED=False
MID_PANEL_BROADCASTED=False
BUTTONS_BROADCASTED=[False]*MAX_MID_PANEL_ARUCO_ID
LEFT_PANEL_BROADCASTED=False
IMU_BROADCASTED=False
RIGHT_PANEL_BROADCASTED=False
LID_BROADCASTED=False
INSPECTION_PANEL_BROADCASTED=False
objects_correctly_broadcasted=True
objects_to_broadcast=[]

can_reference=False
override_reference_timeout=False
REFERENCE_TIMEOUT=5


def nameOfMarkerReference(id):
    #TODO: why not working??: return "id_"+str(id)
    for marker_name, marker_id in markers_dict.items():
        if marker_id == id:
            return marker_name


def getMarkersInSight(aruco_msg):
    global arucos_in_sight
    arucos_in_sight=[]
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


# def getVelocities(js_msg):
#     global joint_velocities
#     current_velocity=js_msg.velocity
#     joint_velocities=np.roll(joint_velocities,1)
#     joint_velocities[0]=current_velocity


#TODO
def velocityTooBig():
    # for current_component in range(len(joint_velocities.T)):
    #     considered_speeds=abs(joint_velocities.T[current_component])
    #     current_speed=np.round(considered_speeds[0],5)
    #     previous_speed=np.round(considered_speeds[10],5)
    #     if abs(current_speed)>20*abs(previous_speed) and previous_speed!=0:
    #     # current_max=np.max(considered_speeds)
    #     # current_mean=\
    #     #     np.mean(considered_speeds[np.nonzero(considered_speeds)]) if \
    #     #     not considered_speeds[np.nonzero(considered_speeds)].size!=0 else 0
    #     # if current_max>5*current_mean or np.isnan(current_mean):
    #         return True
    return False


def distanceTooBig():
    return False


def assembleFindingsMessage():
    msg=FoundArucos()
    for _,current_finding,_ in found_arucos:
        msg.findings.append(current_finding)
    return msg


def referencesServer(permission_signal):
    global can_reference
    if override_reference_timeout:
        permisison_timeout=REFERENCE_TIMEOUT
    else:permisison_timeout=permission_signal.timeout
    #TODO: timed-out refereing deactivated since it starts
    #       with the robot still moveing
    # can_reference=True
    # print("permitting referencing for {} seconds".format(permisison_timeout))
    # start_time=rospy.get_time()
    # while rospy.get_time()-start_time<permisison_timeout:continue
    print("waiting {} seconds before referencing".format(permisison_timeout))
    start_time=rospy.get_time()
    while rospy.get_time()-start_time<permisison_timeout:continue
    can_reference=True
    rospy.sleep(rospy.Duration(nsecs=1E7))
    can_reference=False
    # return ReferenceAcquisitionResponse(done=True)
    return True


def broadcastTFMarkers(_):
    global tf_broadcaster
    global found_arucos
    tfs_to_broadcast=[]

    if not (velocityTooBig() and distanceTooBig()) and can_reference:
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
                        print('broadcasting tf for: {} (markerd id: {})'.
                            format(current_frame,current_id))
                        found_arucos[current_id-1][2].orientation=new_tf.transform.rotation
                        found_arucos[current_id-1][2].position=new_tf.transform.translation
                        found_arucos[current_id-1][1]=True
            except (tf2_ros.LookupException, \
                tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass
    findings_msg=assembleFindingsMessage()
    findings_pub.publish(findings_msg)
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


def updateObjectPoses(_):
    # print(found_arucos)
    global objects_to_broadcast
    global objects_correctly_broadcasted
    if objects_correctly_broadcasted:
        objects_to_broadcast=[]

    if not MID_PANEL_BROADCASTED and \
    all([found_arucos[required-1][1] for required in objects_markers['mid_panel']]):
        print('referencing '+'mid_panel')
        computeMidPanelPose()

    if not TABLE_BROADCASTED and \
        all([found_arucos[required-1][1] for required in objects_markers['table_']]):
        print('referencing '+'table_')
        computeTablePose()

    if not ROBOT_FRAME_BROADCASTED:
        print('referencing '+'robot_frame')
        computeRobotFramePose()

    if not LEFT_PANEL_BROADCASTED and \
    all([found_arucos[required-1][1] for required in objects_markers['left_panel']]):
        print('referencing '+'left_panel')
        computeLeftPanelPose()

    if not RIGHT_PANEL_BROADCASTED and \
    all([found_arucos[required-1][1] for required in objects_markers['right_panel']]):
        print('referencing '+'right_panel')
        computeRightPanelPose()

    [[print('referencing '+'button_'+str(id)), computeButtonPose(id)]\
        for id in range(1,MAX_MID_PANEL_ARUCO_ID+1) \
        if not BUTTONS_BROADCASTED[id-1] and found_arucos[id-1][1]]
    
    if not IMU_BROADCASTED and \
    all([found_arucos[required-1][1] for required in objects_markers['imu_']]):
        print('referencing '+'imu_')
        computeImuPose()
    
    if not INSPECTION_PANEL_BROADCASTED and \
    all([found_arucos[required-1][1] for required in objects_markers['inspection_panel']]):
        print('referencing '+'inspection_panel')
        computeInspectionPanelPose()
    
    if not LID_BROADCASTED and \
    all([found_arucos[required-1][1] for required in objects_markers['lid_']]):
        print('referencing '+'lid_')
        computeLidPose()

    if objects_to_broadcast:
        try:
            tf_broadcaster.sendTransform(objects_to_broadcast)
            objects_correctly_broadcasted=True
        except (tf2_ros.LookupException, \
                    tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            objects_correctly_broadcasted=False
            #TODO: add check if object already in the list
            #   currently leave to tf the job


def computeTablePose():
    global objects_to_broadcast
    global TABLE_BROADCASTED
    # object_pose.orientation=[found_arucos[0][2].orientation for required in objects_markers['mid_panel']][0]

    tf_stamped=TransformStamped()
    tf_stamped.header.stamp=rospy.Time.now()
    current_frame="table_"
    tf_stamped.child_frame_id=current_frame
    tf_stamped.header.frame_id=base_frame
    tf_stamped.transform.translation.x=.7/4
    tf_stamped.transform.translation.y=0
    tf_stamped.transform.translation.z=\
     np.average([found_arucos[required-1][2].position.z for required in objects_markers['table_']])-.025
    tf_stamped.transform.rotation.w=1
    objects_to_broadcast.append(tf_stamped)
    TABLE_BROADCASTED=True


def computeMidPanelPose():
    global objects_to_broadcast
    global MID_PANEL_BROADCASTED
    object_pose=found_arucos[0][2]

    tf_stamped=TransformStamped()
    tf_stamped.header.stamp=rospy.Time.now()
    current_frame="mid_panel"
    tf_stamped.child_frame_id=current_frame
    tf_stamped.header.frame_id=base_frame
    tf_stamped.transform.translation.x=\
        np.average([found_arucos[required-1][2].position.x for required in objects_markers['mid_panel']])
    tf_stamped.transform.translation.y=\
        np.average([found_arucos[required-1][2].position.y for required in objects_markers['mid_panel']])
    tf_stamped.transform.translation.z=\
        np.average([found_arucos[required-1][2].position.z for required in objects_markers['mid_panel']])
    #TODO:quaternions average
    tf_stamped.transform.rotation.x=object_pose.orientation.x
    tf_stamped.transform.rotation.y=object_pose.orientation.y
    tf_stamped.transform.rotation.z=object_pose.orientation.z
    tf_stamped.transform.rotation.w=object_pose.orientation.w
    objects_to_broadcast.append(tf_stamped)
    MID_PANEL_BROADCASTED=True


def computeRobotFramePose():
    """
      z=base_frame-.1
      y=base_frame
      x=average(mid_panel,base_frame)
      orientation=base_frame
      size= x:.7
            y,z=.1
    """
    global objects_to_broadcast
    global ROBOT_FRAME_BROADCASTED

    tf_stamped=TransformStamped()
    tf_stamped.header.stamp=rospy.Time.now()
    current_frame="robot_frame"
    tf_stamped.child_frame_id=current_frame
    tf_stamped.header.frame_id=base_frame
    tf_stamped.transform.translation.x=.7/2.5
    tf_stamped.transform.translation.y=0
    tf_stamped.transform.translation.z=-.08
    tf_stamped.transform.rotation.w=1
    objects_to_broadcast.append(tf_stamped)
    ROBOT_FRAME_BROADCASTED=True


def computeButtonPose(id):
    global objects_to_broadcast
    global BUTTONS_BROADCASTED

    # object_pose.position.x=found_arucos[id-1][2].position.x
    # object_pose.position.y=found_arucos[id-1][2].position.y-.055
    # object_pose.position.z=found_arucos[id-1][2].position.z+.01
    tf_stamped=TransformStamped()
    tf_stamped.header.stamp=rospy.Time.now()
    current_frame="button_"+str(id)
    tf_stamped.child_frame_id=current_frame
    tf_stamped.header.frame_id="id_"+str(id)
    tf_stamped.transform.translation.x=0
    tf_stamped.transform.translation.y=-.055
    tf_stamped.transform.translation.z=.01
    tf_stamped.transform.rotation.w=1
    objects_to_broadcast.append(tf_stamped)
    BUTTONS_BROADCASTED[id-1]=True


def computeLeftPanelPose():
    global objects_to_broadcast
    global LEFT_PANEL_BROADCASTED

    tf_stamped=TransformStamped()
    tf_stamped.header.stamp=rospy.Time.now()
    reference_id=11
    current_frame='left_panel'
    tf_stamped.child_frame_id=current_frame
    tf_stamped.header.frame_id="id_"+str(reference_id)
    tf_stamped.transform.translation.x=0.0625
    tf_stamped.transform.translation.y=-.15
    tf_stamped.transform.translation.z=0
    tf_stamped.transform.rotation.w=1
    objects_to_broadcast.append(tf_stamped)
    LEFT_PANEL_BROADCASTED=True


def computeImuPose():
    global objects_to_broadcast
    global IMU_BROADCASTED
    
    tf_stamped=TransformStamped()
    tf_stamped.header.stamp=rospy.Time.now()
    reference_id=10
    current_frame="imu_"
    tf_stamped.child_frame_id=current_frame
    tf_stamped.header.frame_id="id_"+str(reference_id)
    tf_stamped.transform.translation.x=0
    tf_stamped.transform.translation.y=0
    tf_stamped.transform.translation.z=-.025
    #starting from (0,0,0,1)
    new_orientation=quaternion_about_axis(PI/2,(1,0,0))
    tf_stamped.transform.rotation.x=new_orientation[0]
    tf_stamped.transform.rotation.y=new_orientation[1]
    tf_stamped.transform.rotation.z=new_orientation[2]
    tf_stamped.transform.rotation.w=new_orientation[3]
    objects_to_broadcast.append(tf_stamped)
    IMU_BROADCASTED=True


def computeRightPanelPose():
    global objects_to_broadcast
    global RIGHT_PANEL_BROADCASTED
    
    tf_stamped=TransformStamped()
    tf_stamped.header.stamp=rospy.Time.now()
    reference_id=12
    current_frame="right_panel"
    tf_stamped.child_frame_id=current_frame
    tf_stamped.header.frame_id="id_"+str(reference_id)
    tf_stamped.transform.translation.x=0
    tf_stamped.transform.translation.y=-(.04+.02)
    tf_stamped.transform.translation.z=-.1
    tf_stamped.transform.rotation.w=1
    objects_to_broadcast.append(tf_stamped)
    computeLidHandlePose()
    RIGHT_PANEL_BROADCASTED=True


def computeLidPose():
    global objects_to_broadcast
    global LID_BROADCASTED
    
    tf_stamped=TransformStamped()
    tf_stamped.header.stamp=rospy.Time.now()
    reference_id=13
    current_frame="lid_"
    tf_stamped.child_frame_id=current_frame
    tf_stamped.header.frame_id="id_"+str(reference_id)
    tf_stamped.transform.translation.x=.05
    tf_stamped.transform.translation.y=.025
    tf_stamped.transform.translation.z=-.002
    tf_stamped.transform.rotation.w=1
    objects_to_broadcast.append(tf_stamped)
    computeLidHandlePose()
    LID_BROADCASTED=True


def computeLidHandlePose():
    global objects_to_broadcast
    
    tf_stamped=TransformStamped()
    tf_stamped.header.stamp=rospy.Time.now()
    reference_id=13
    current_frame="lid_handle"
    tf_stamped.child_frame_id=current_frame
    tf_stamped.header.frame_id="id_"+str(reference_id)
    tf_stamped.transform.translation.x=.05
    tf_stamped.transform.translation.y=.025
    tf_stamped.transform.translation.z=.0175
    tf_stamped.transform.rotation.w=1
    objects_to_broadcast.append(tf_stamped)


def computeInspectionPanelPose():
    global objects_to_broadcast
    global INSPECTION_PANEL_BROADCASTED
    reference_id=12
    tf_stamped=TransformStamped()
    tf_stamped.header.stamp=rospy.Time.now()
    current_frame="inspection_panel"
    tf_stamped.child_frame_id=current_frame
    tf_stamped.header.frame_id="id_"+str(reference_id)
    tf_stamped.transform.translation.x=0
    tf_stamped.transform.translation.y=0
    tf_stamped.transform.translation.z=-.05
    tf_stamped.transform.rotation.w=1
    objects_to_broadcast.append(tf_stamped)
    INSPECTION_PANEL_BROADCASTED=True


# def cleanObjectPoses(_):
#TODO: removing tf_static not possible: can reparent it to "" otherwise
#   could change it into dynamic and wait until expired ("too old tf")
#     request_cleaning_operation=input("enter {y,Y,yes,YES} to remove all objects' references: ")
#     if request_cleaning_operation in ['y','yes','Y','YES']:
#         if input("hit enter to confirm")=="":
            
    
        
#----------------------------------------------------------
def arucoReferencer():
    # global joint_velocities
    # joint_velocities=np.zeros(\
    #     (50,len(rospy.wait_for_message(joint_states_topic,JointState).velocity)))

    global aruco_sub
    aruco_sub=rospy.Subscriber(aruco_topic,ArucoPoses,getMarkersInSight)

    global tf_buffer
    global tf_listener
    global tf_broadcaster
    tf_buffer=tf2_ros.Buffer(cache_time=rospy.Duration(1))
    tf_listener=tf2_ros.TransformListener(tf_buffer,queue_size=None)
    #COULD ADD WAIT_FOR_SERVICE/_FOR_SERVER TO PREVENT ERRORS
    tf_broadcaster=tf2_ros.StaticTransformBroadcaster()

    global findings_pub
    findings_pub=rospy.Publisher(findings_topic,FoundArucos,queue_size=1)

    rospy.Service(REFERENCES_SERVICE, ReferenceAcquisition, referencesServer)
    
    ARUCO_TIMER_DURATION=rospy.Duration(nsecs=2E6)
    #TODO: instead of a timer, should use a service
    #   requested when the robot is not moving, by controller
    tf_timer=rospy.Timer(ARUCO_TIMER_DURATION,broadcastTFMarkers)

    # refinement_timer=rospy.Timer(5*TIMER_DURATION,refineTFMarkers)

    # velocity_sub=rospy.Subscriber(joint_states_topic,JointState,getVelocities,queue_size=1)

    OBJECTS_TIMER_DURATION=rospy.Duration(secs=2)
    #TODO: could change into broadcasting only objects and only using pose
    #   for scoring the aruco findings
    update_timer=rospy.Timer(OBJECTS_TIMER_DURATION,updateObjectPoses)

    # CLEANING_TIMER_DURATION=rospy.Duration(nsecs=1E7)
    # cleaning_timer=rospy.Timer(CLEANING_TIMER_DURATION,cleanObjectPoses,oneshot=True)

#############################################################

if __name__=='__main__':

    node_name="referencer"
    print('node name: {}\n publishes coordinates transforms for found markers'.
        format(node_name))

    rospy.init_node(node_name,anonymous=False)
    arucoReferencer()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.signal_shutdown('Esc key pressed; Closing node: {}'.format(node_name))