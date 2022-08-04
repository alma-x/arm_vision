#!/usr/bin/env python3

from traceback import print_tb
import rospy
import numpy as np
import os
from time import sleep
from geometry_msgs.msg import Pose
import tf2_ros
from geometry_msgs.msg import TransformStamped
from arm_vision.msg import FoundArucos
from arm_control.srv import aruco_service,aruco_serviceResponse
from arm_control.srv import cv_server,cv_serverResponse, cv_serverRequest
from arm_control.msg import cv_to_bridge as bridge_msg
from arm_control.srv import DummyMarker,DummyMarkerResponse,DummyMarkerRequest

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

findings_topic="/found_arucos"
base_frame='base_link'
inquiries_service='aruco_inquiries'
found_markers=[]
available_ids=[]
available_corners=[]

tf_buffer=None
tf_listener=None

def nameOfMarkerReference(id):
    global markers_dict
    for marker_name, marker_id in markers_dict.items():
        if marker_id == id:
            return marker_name


def getArucoPose(marker_id):
    marker_frame=nameOfMarkerReference(marker_id)
    global tf_buffer
    while True:
        try:
            return tf_buffer.lookup_transform(base_frame,marker_frame,rospy.Time()).transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, \
            tf2_ros.ExtrapolationException): continue


def getFoundMarkers(findings_msg):
    global found_markers
    found_markers=findings_msg.findings


def arucoInquiriesServer(aruco_req):
    print('checking if id {} has been found'.format(aruco_req.id))
    if (aruco_req.id-1) in [*np.where(found_markers)][0]:
        aruco_response=DummyMarkerResponse()
        aruco_response.found=True
        tf_=getArucoPose(aruco_req.id)
        # aruco_response.pose.position=[tf_.translation.x,tf_.translation.y,tf_.translation.z]
        aruco_response.pose.position=tf_.translation
        aruco_response.pose.orientation=tf_.rotation
    else:
        aruco_response=DummyMarkerResponse()
        aruco_response.found=False
        aruco_response.pose=Pose()
    print(aruco_response)
    return aruco_response
       

def arucoInquirer():
    findings_sub=rospy.Subscriber(findings_topic,FoundArucos,getFoundMarkers,queue_size = 1)
    
    global  tf_buffer,\
            tf_listener
    tf_buffer=tf2_ros.Buffer(cache_time=rospy.Duration(1))
    tf_listener=tf2_ros.TransformListener(tf_buffer,queue_size=None)

    # global bridge_pub
    # bridge_pub = rospy.Publisher('aruco_bridge_opencv', bridge_msg, queue_size=1)
    
    # INQUIRE_DURATION=rospy.Duration(nsecs=1E6)
    # inquire_timer=rospy.Timer(INQUIRE_DURATION,inquireFoundMarkers)

    # rospy.Service('cv_server', cv_server, callback_service)

    rospy.Service(inquiries_service, DummyMarker, arucoInquiriesServer)
    
###########################################################

if __name__ == '__main__':
    
    node_name="inquirer"
    print('node name: {}\n checks if certain aruco has been found'.
        format(node_name))

    rospy.init_node(node_name,anonymous=False)
    arucoInquirer()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.signal_shutdown('Esc key pressed; Closing node: {}'.format(node_name))


# bool_exit=False

# targetList=[[1,50],
#             [2,50],
#             [3,50],
#             [4,50],
#             [5,50],
#             [6,50],
#             [7,50],
#             [8,50],
#             [9,50],
#             [10,40],
#             [11,50],
#             [12,50],
#             [13,40],
#             [14,50]]
# aruco_success=False

# targetCounter=0
# targetListLen=len(targetList)
# remaining_targets=targetListLen

# global findNewTarget
# findNewTarget=1

# global targetMarkId,targetMarkSize

# msgVector=[0,0,0]#np.zeros([1,3])
# msgRotMatrix=[[0,0,0,],[0,0,0],[0,0,0]]#np.zeros([3,3])

# tglWristLengthRecovery=1
# recovered percentage
# recovLenRatio=1


# def rotationMatrixFromQuaternion(Q):
#     #https://www.meccanismocomplesso.org/en/hamiltons-quaternions-and-3d-rotation-with-python/
#     # R00=1-2*Q[1]*Q[1]-2*Q[2]*Q[2]
#     # R01=2*(Q[0]*Q[1]-Q[3]*Q[2])
#     # R02=2*(Q[0]*Q[2]+Q[3]*Q[1])
#     # R10=2*(Q[0]*Q[1]+Q[3]*Q[2])
#     # R11=1-2*Q[0]*Q[0]-2*Q[2]*Q[2]
#     # R12=2*(Q[1]*Q[2]-Q[3]*Q[0])
#     # R20=2*(Q[0]*Q[2]-Q[3]*Q[1])
#     # R21=2*(Q[1]*Q[2]+Q[3]*Q[0])
#     # R22=1-2*Q[0]*Q[0]-2*Q[1]*Q[1]
#     #https://automaticaddison.com/how-to-convert-a-quaternion-to-a-rotation-matrix/
#     R00=2*(Q.x*Q.x+Q.y*Q.y)-1
#     R01=2*(Q.y*Q.z-Q.x*Q.w)
#     R02=2*(Q.y*Q.w+Q.x*Q.z)
#     R10=2*(Q.y*Q.z+Q.x*Q.w)
#     R11=2*(Q.x*Q.x+Q.z*Q.z)-1
#     R12=2*(Q.z*Q.w-Q.x*Q.y)
#     R20=2*(Q.y*Q.w-Q.x*Q.z)
#     R21=2*(Q.z*Q.w+Q.x*Q.y)
#     R22=2*(Q.x*Q.x-2*Q.w*Q.w)-1
#     R=[[R00,R01,R02],[R10,R11,R12],[R20,R21,R22]]
#     return np.array(R)


# def inquireFoundMarkers(_):
#     global aruco_success
#     global msgVector
#     global msgRotMatrix
#     global targetCounter
#     global findNewTarget
#     global remaining_targets
#     global bool_exit

#     if bool_exit: os._exit(os.EX_OK)

#     msg=bridge_msg()
#     msg.aruco_found=14*[False]

#     if np.any(found_markers):
#         aruco_success=False 
#         for found_id in [*np.where(found_markers)][0]:
#             msg.aruco_found[int(found_id)]=True
#             if found_id+1==targetList[targetCounter][0]:
#                 tf_=getArucoPose(found_id+1)      
#                 msgVector=[tf_.translation.x,tf_.translation.y,tf_.translation.z]  
#                 msgRotMatrix=rotationMatrixFromQuaternion(tf_.rotation)
                
#                 aruco_success=True       
#     else:
#         aruco_success=False

#     msg.success=aruco_success
#     msg.id_aruco=targetCounter+1

#     if msg.success:
#         #msg.x=0.001*msgVector[2] +(recovLenRatio*0.08 if tglWristLengthRecovery else 0)
#         msg.x=0.001*msgVector[0]
#         msg.y=0.001*msgVector[1]
#         msg.z=0.001*msgVector[2]
#         msg.vector=msgRotMatrix.flatten()
    
#     bridge_pub.publish(msg)


# def callback_service(req):
#     global aruco_success,\
#             msgVector,\
#             msgRotMatrix,\
#             targetCounter,\
#             findNewTarget,\
#             remaining_targets,\
#             bool_exit
    
#     print('Service received')

#     if req.message=="exit":
#         bool_exit=True
#     if req.message=="select_next_aruco":
#         targetCounter=int(req.second_information)-1

#     return cv_serverResponse(
#         success=aruco_success,
#         moreTargets=remaining_targets,
#         x=0.001*msgVector[2] +(recovLenRatio*0.08 if tglWristLengthRecovery else 0),#[m]
#         y=0.001*msgVector[0],   
#         z=0.001*msgVector[1],
#         vector=np.ravel(msgRotMatrix)#flattened array
#         )