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
from arm_control.srv import aruco_service,aruco_serviceResponse,cv_server,cv_serverResponse, cv_serverRequest
from arm_vision.srv import FoundMarker,FoundMarkerResponse,FoundMarkerRequest,ErcAruco,ErcArucoRequest,ErcArucoResponse

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
              'left_panel':         11,
              'inspection_panel':   12,
              'lid_':               13,
              'lid_storage':        14
              }

findings_topic="/found_arucos"
base_frame='base_link'
INQUIRIES_SERVICE='aruco_inquiries'
found_markers=[]
available_ids=[]
available_corners=[]
SCORE_SERVICE='/erc_aruco_score'
score_service_called=False
aruco_score=0

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
    if not score_service_called and all(found_markers):
        scoreServiceClient()



def arucoInquiriesServer(aruco_req):
    # # print('checking if id {} has been found'.format(aruco_req.id))
    # if (aruco_req.id-1) in [*np.where(found_markers)][0]:
    #     aruco_response=FoundMarkerResponse(
    #                         found=True,
    #     )
    #     # aruco_response.found=True
    #     tf_=getArucoPose(aruco_req.id)
    #     # aruco_response.pose.position=[tf_.translation.x,tf_.translation.y,tf_.translation.z]
    #     aruco_response.pose.position=tf_.translation
    #     aruco_response.pose.orientation=tf_.rotation
    # else:
    #     aruco_response=FoundMarkerResponse(
    #                         found=False,
    #                         pose=Pose())
    #     # aruco_response.found=False
    #     # aruco_response.pose=Pose()
    # # print(aruco_response)
    aruco_response=arucoInquiriesRoutine(aruco_req.id)
    return aruco_response
    

def arucoInquiriesRoutine(id):
    print('trying inquiry {}...'.format(id),end='')
    if (id-1) in [*np.where(found_markers)][0]:
        aruco_response=FoundMarkerResponse(
                            found=True,
        )
        tf_=getArucoPose(id)
        aruco_response.pose.position=tf_.translation
        aruco_response.pose.orientation=tf_.rotation
        print('found')
    else:
        aruco_response=FoundMarkerResponse(
                            found=False,
                            pose=Pose())
        print('NOT FOUND!')
    return aruco_response


def scoreServiceClient():
    global score_service_called
    global aruco_score
    try:
        score_proxy = rospy.ServiceProxy(SCORE_SERVICE,ErcAruco)
        score_request=ErcArucoRequest()
        score_request.tag1=positionToArray(arucoInquiriesRoutine(1).pose.position)
        score_request.tag2=positionToArray(arucoInquiriesRoutine(2).pose.position)
        score_request.tag3=positionToArray(arucoInquiriesRoutine(3).pose.position)
        score_request.tag4=positionToArray(arucoInquiriesRoutine(4).pose.position)
        # score_request.tag5=positionToArray(arucoInquiriesRoutine(5).pose.position)
        # score_request.tag6=positionToArray(arucoInquiriesRoutine(6).pose.position)
        # score_request.tag7=positionToArray(arucoInquiriesRoutine(7).pose.position)
        # score_request.tag8=positionToArray(arucoInquiriesRoutine(8).pose.position)
        # score_request.tag9=positionToArray(arucoInquiriesRoutine(9).pose.position)
        score_request.tag5=[.0,.0,.0]
        score_request.tag6=[.0,.0,.0]
        score_request.tag7=[.0,.0,.0]
        score_request.tag8=[.0,.0,.0]
        score_request.tag9=[.0,.0,.0]
        score_request.tag10=positionToArray(arucoInquiriesRoutine(10).pose.position)
        score_request.tag11=positionToArray(arucoInquiriesRoutine(11).pose.position)
        score_request.tag12=positionToArray(arucoInquiriesRoutine(12).pose.position)
        score_request.tag13=positionToArray(arucoInquiriesRoutine(13).pose.position)
        score_request.tag14=positionToArray(arucoInquiriesRoutine(14).pose.position)
        # print(score_request)
        aruco_score=score_proxy(score_request)
        print('RECEIVED SCORE FOR ENVIRONMENT EXPLORATION: {}'.format(aruco_score))
        score_service_called=True
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")


def positionToArray(pose_position):
    return [round(pose_position.x,4),
            round(pose_position.y,4),
            round(pose_position.z,4)]


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

    #aruco localization score service
    rospy.wait_for_service(SCORE_SERVICE)

    rospy.Service(INQUIRIES_SERVICE, FoundMarker, arucoInquiriesServer)

###########################################################

if __name__ == '__main__':

    node_name="inquirer"
    print('node name: {}\n checks if certain aruco has been found'.
        format(node_name))

    rospy.init_node(node_name,anonymous=False)

    arucoInquirer()

    try:
        rospy.spin()
    except KeyboardInterrupt or rospy.ROSInterruptException:
        rospy.signal_shutdown('Esc key pressed; Closing node: {}'.format(node_name))


"""
def rotationMatrixFromQuaternion(Q):
    #https://www.meccanismocomplesso.org/en/hamiltons-quaternions-and-3d-rotation-with-python/
    # R00=1-2*Q[1]*Q[1]-2*Q[2]*Q[2]
    # R01=2*(Q[0]*Q[1]-Q[3]*Q[2])
    # R02=2*(Q[0]*Q[2]+Q[3]*Q[1])
    # R10=2*(Q[0]*Q[1]+Q[3]*Q[2])
    # R11=1-2*Q[0]*Q[0]-2*Q[2]*Q[2]
    # R12=2*(Q[1]*Q[2]-Q[3]*Q[0])
    # R20=2*(Q[0]*Q[2]-Q[3]*Q[1])
    # R21=2*(Q[1]*Q[2]+Q[3]*Q[0])
    # R22=1-2*Q[0]*Q[0]-2*Q[1]*Q[1]
    #https://automaticaddison.com/how-to-convert-a-quaternion-to-a-rotation-matrix/
    R00=2*(Q.x*Q.x+Q.y*Q.y)-1
    R01=2*(Q.y*Q.z-Q.x*Q.w)
    R02=2*(Q.y*Q.w+Q.x*Q.z)
    R10=2*(Q.y*Q.z+Q.x*Q.w)
    R11=2*(Q.x*Q.x+Q.z*Q.z)-1
    R12=2*(Q.z*Q.w-Q.x*Q.y)
    R20=2*(Q.y*Q.w-Q.x*Q.z)
    R21=2*(Q.z*Q.w+Q.x*Q.y)
    R22=2*(Q.x*Q.x-2*Q.w*Q.w)-1
    R=[[R00,R01,R02],[R10,R11,R12],[R20,R21,R22]]
    return np.array(R)
    
"""
