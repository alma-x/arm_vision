#!/usr/bin/env python3

import rospy
import numpy as np
import sys
import os
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
#from sensor_msgs.msg import PointCloud2 as sensPCld

from arm_control.srv import aruco_service,aruco_serviceResponse
from arm_control.srv import cv_server,cv_serverResponse, cv_serverRequest
from arm_control.msg import cv_to_bridge as bridge_msg

import cv2 
from cv2 import aruco as aruco
from cv_bridge import CvBridge, CvBridgeError
from roscamLibrary3 import nsingleAruRelPos as singleAruRelPos

#PUBLISHER DI ARRAY:
#aruco_position_pub = rospy.Publisher('/almax/aruco_target',Float64MultiArray,queue_size=20)
#array = [69.1,0,1,33,1,1,1,0]
#robaccia = Float64MultiArray(data=array)
#aruco_position_pub.publish(robaccia)
#------------------------------------------------

#TODO: MAKE EVERYTHING LESS CHAOTIC, MAYBE IN A CLASS

#TODO: LESS GENERIC PUB NAME
pub = rospy.Publisher('aruco_bridge_opencv', bridge_msg, queue_size=1)
bool_exit=False

ARUCO_PARAMETERS = aruco.DetectorParameters_create()

#TODO? MOVE DICT INSIDE RELATIVE FUNCTION?
aruLibrary={'original':aruco.DICT_ARUCO_ORIGINAL
            ,'51000':aruco.DICT_5X5_1000
            ,'61000':aruco.DICT_6X6_1000
            ,'71000':aruco.DICT_7X7_1000
            }
#ARUCO_DICT = aruco.Dictionary_get(aruLibrary['original'])

def loadArucoDict(requestedDict):#TODO
    global ARUCO_DICT
    ARUCO_DICT = aruco.Dictionary_get(aruLibrary[requestedDict])

selectedDictionary='original'
loadArucoDict(selectedDictionary)   
 
#----------------------------------------------
    
def loadCameraParam(myCam):
    # TODO: "/color" MUST BE PART OF ARG
    global cameraMatr
    global cameraDistCoefs
    global cameraFocLen
    
    print('loading camera parameters...')
    # cameraInfoMsg=rospy.wait_for_message(myCam+'/color/camera_info',CameraInfo)
    cameraInfoMsg=rospy.wait_for_message(myCam+'/camera_info',CameraInfo)
    cameraMatr=np.reshape(cameraInfoMsg.K,[3,3])
    cameraDistCoefs=cameraInfoMsg.D
    cameraFocLen=np.mean([np.ravel(cameraMatr[0])[0],np.ravel(cameraMatr[1])[1]])
    
#------------------------------------------------------------
def getMarkerSizeFromId(target_list,target_id):
    '''
    returns the marker size passing target marker id and list of all markers
    '''
    for current_id,current_size in target_list:
        if current_id==target_id: return current_size
    return 0

#aruTargetDict={'panelSwitch1':{'id':1,'size':50}
#               'panelSwitch2':{'id':2,'size':50}}

#TODO: MAKE THIS INTO A DICTIONARY AS ABOVE
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
targetList=[[1,50],
            [2,50],
            [3,50],
            [4,50],
            [5,50],
            [6,50],
            [7,50],
            [8,50],
            [9,50],
            [10,40],
            [11,50],
            [12,50],
            [13,40],
            [14,50],
            [102,40],
            [104,40],
            [106,40],
            [108,40]]
aruco_success=False

#TODO: FOLLOWING TARGET STRING DOES NOT WORK
#targetList=['panelSwitch8','panelSwitch7','panelSwitch6','panelSwitch5','panelSwitch4'
#            ,'panelSwitch3','panelSwitch2','panelSwitch1']

#TODO? WHY IS A TARGET STRING NEEDED?
targetListLen=len(targetList)

#global findNewTarget
#findNewTarget=1

#def receiveTargetRequest():
#    selectedTarget=read_somewhere(targetRequestTopic)
#    (targetMarkId,targetMarkSize)=loadTargetData(selectedTarget)

#def loadTargetData(requestString):
#    return aruTargetDict[requestString]

#TODO? IS global NEEDED HERE?
global targetMarkId,targetMarkSize

#----------------------------------------
bridge=CvBridge()
 
def callbackRaw(raw_img):
    global aruco_success
    global msgVector
    global msgRotMatrix
    global targetCounter
    global findNewTarget
    global remaining_targets
    global bool_exit

    cv_image=bridge.imgmsg_to_cv2(raw_img, desired_encoding='passthrough')
    cv_gray=cv2.cvtColor(cv_image,cv2.COLOR_RGB2GRAY)

    detCorners, detIds, _ = aruco.detectMarkers(cv_gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
        
    if detIds is not None or len(detIds) >= 1:
            detAruImg = aruco.drawDetectedMarkers(cv_image.copy(), detCorners, borderColor=(0, 255, 0))
            

            for mId, aruPoints in zip(detIds, detCorners):
                targetMarkSize=getMarkerSizeFromId(targetList,mId)
                detAruImg,aruDistnc,Pmatr=singleAruRelPos(detAruImg,aruPoints,mId,targetMarkSize,
                                            cameraMatr,cameraDistCoefs,tglDrawMark=1)
                
                rotMatr,tVect=Pmatr[0:3,0:3],Pmatr[0:3,3]
                msgRotMatrix=rotMatr
                msgVector=tVect
            aruco_img_size=detAruImg.shape[:2]
            detAruImg=cv2.resize(detAruImg,(aruco_img_size[1]//2,aruco_img_size[0]//2))
            display_image=detAruImg
    else:
        cv_img_size=cv_image.shape[:2]
        cv_image=cv2.resize(cv_image,(cv_img_size[1]//2,cv_img_size[0]//2))
        display_image=cv_image

    cv2.imshow('detected markers',display_image)
    kk = cv2.waitKey(12) & 0xFF
    if kk == 27:
      rospy.on_shutdown()
        
    
#-----------------------------------------------------------------

def listener(myCam,myTop,myType,myCallk):
    node_name='camera_listener'
    rospy.init_node(node_name, anonymous=False)
    loadCameraParam(myCam)
    print('node: {}'.format(node_name))
    print('camera: {}'.format(myCam))
    print('input topic: {}'.format(myTop))

    rospy.Subscriber(myCam+myTop,myType,myCallk,queue_size = 1)

    try:
        rospy.spin()
    except KeyboardInterrupt:#
        print('Closing node: {}'.format(node_name))
    cv2.destroyAllWindows()
    
#---------------------------------------------------------------
    
#TODO? DOES SPLITTING CAMERA AND "TOPIC" NAME HAVE ANY, NON COSMETHIC, MEANING?    
camDict={'moving':"/camera_image",
            'fixed':"/camera_image_fix"}

topicDict={'raw':("/color/image_raw",
                   Image,
                    callbackRaw)    
            }   

################===========================#####################
if __name__ == '__main__':
    #TODO: FIX CAMERA SELECTION
    # myCamera=camDict['moving']
    # myTopicFull=topicDict['raw']
    myCamera="/camera_image"
    myTopicFull=("/image_raw",Image,callbackRaw)
    
    #TODO: MOVE WAITING INSIDE THE CLASS
    print('Detector Node for Panal Aruco Visual Markers')
    print('connecting to:'+myCamera+myTopicFull[0]+'...')
    listener(myCamera,myTopicFull[0],myTopicFull[1],myTopicFull[2])