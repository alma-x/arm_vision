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
    print(type(cameraMatr))
    cameraDistCoefs=cameraInfoMsg.D
    cameraFocLen=np.mean([np.ravel(cameraMatr[0])[0],np.ravel(cameraMatr[1])[1]])
    
#------------------------------------------------------------
    
#aruTargetDict={'panelSwitch1':{'id':1,'size':50}
#               'panelSwitch2':{'id':2,'size':50}}

#TODO: MAKE THIS INTO A DICTIONARY AS ABOVE
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
            [14,50]]
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
#             [14,50],
#             [102,40],
#             [104,40],
#             [106,40],
#             [108,40]]
aruco_success=False

#TODO: FOLLOWING TARGET STRING DOES NOT WORK
#targetList=['panelSwitch8','panelSwitch7','panelSwitch6','panelSwitch5','panelSwitch4'
#            ,'panelSwitch3','panelSwitch2','panelSwitch1']

#TODO? WHY IS A TARGET STRING NEEDED?
# targetList=[[102,40],[104,40],[106,40],[108,40]]
targetCounter=0
targetListLen=len(targetList)
remaining_targets=targetListLen

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

    if bool_exit:
        cv2.destroyAllWindows()
        os._exit(os.EX_OK)

    # TODO: ADD try:
    cv_image=bridge.imgmsg_to_cv2(raw_img, desired_encoding='passthrough')
    cv_gray=cv2.cvtColor(cv_image,cv2.COLOR_RGB2GRAY)
    # except CvBridgeError: ...

    msg=bridge_msg()
    msg.aruco_found=[False,False,False,False,False,False,False,False,False,False,False,False,False,False]
    # msg.aruco_found=[False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False]
    
    #TODO: WHY targetCounter goes above len(targetList) even if forced otherwise?
    (targetMarkId,targetMarkSize)=tuple(targetList[targetCounter])
    detCorners, detIds, _ = aruco.detectMarkers(cv_gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
        
    if detIds is not None and len(detIds) >= 1: # Check if at least one marker has been found
        
        detAruImg = aruco.drawDetectedMarkers(cv_image.copy(), detCorners, borderColor=(0, 255, 0))
        
        aruco_success=False 

        for mId, aruPoints in zip(detIds, detCorners):
            if len(msg.aruco_found)>int(mId):
                msg.aruco_found[int(mId)]=True
            if mId==targetList[targetCounter][0]:
                # targetMarkSize=targetList[mId-1][1]
                detAruImg,aruDistnc,Pmatr=singleAruRelPos(detAruImg,aruPoints,mId,targetMarkSize,
                                            cameraMatr,cameraDistCoefs,tglDrawMark=1)
                
                rotMatr,tVect=Pmatr[0:3,0:3],Pmatr[0:3,3]
                msgRotMatrix=rotMatr
                msgVector=tVect
                
                aruco_success=True
                # remaining_targets=targetListLen-targetCounter-1
                #if targetCounter<targetListLen-1:
                #    targetCounter+=1
                
    else:
        aruco_success=False
        detAruImg=cv_image.copy()

    #    newSize,_=int(np.shape(detAruImg))
    #    detAruImg=cv2.resize(detAruImg,newSize)
    cv2.imshow('detected markers',detAruImg)

    msg.success=aruco_success

    msg.id_aruco=targetCounter+1

    if msg.success:
        #msg.x=0.001*msgVector[2] +(recovLenRatio*0.08 if tglWristLengthRecovery else 0)
        msg.x=0.001*msgVector[0]
        msg.y=0.001*msgVector[1]
        msg.z=0.001*msgVector[2]
        msg.vector=msgRotMatrix.flatten()
        #print(msg.vector)
    
    pub.publish(msg)
    key = cv2.waitKey(12) & 0xFF
    if key == ord('q'):
      rospy.on_shutdown()
        
    
#-----------------------------------------------------------------

msgVector=[0,0,0]#np.zeros([1,3])
msgRotMatrix=[[0,0,0,],[0,0,0],[0,0,0]]#np.zeros([3,3])

        
tglWristLengthRecovery=1
# recovered percentage
recovLenRatio=1

def callback_service(req):
    global aruco_success,msgVector,msgRotMatrix,targetCounter,findNewTarget,remaining_targets,bool_exit
    
    print('Service received')
    if req.message=="exit":
        bool_exit=True
    #if req.next_aruco:
    if req.message=="select_next_aruco":
        #if targetCounter<targetListLen-1:
        #    targetCounter=targetCounter+1
        #remaining_targets=targetListLen-targetCounter-1
        targetCounter=int(req.second_information)-1

    #print('Arucopy:\nService received')
    #print('Service received')
    #print('Target number:'+str(targetCounter))
    #print('Remaining_targets:'+str(remaining_targets))
    #print('TargetListLen:'+str(targetListLen))
    return cv_serverResponse(
        success=aruco_success,
        moreTargets=remaining_targets,
        x=0.001*msgVector[2] +(recovLenRatio*0.08 if tglWristLengthRecovery else 0),#[m]
        y=0.001*msgVector[0],   
        z=0.001*msgVector[1],
        vector=np.ravel(msgRotMatrix)#flattened array
        )


#NOTE:
#   tVect       tool0/maniulator e.e reference frame
#    X              Z
#    Y              x
#    Z              Y
#rotation matrix: tVect=R*tool0_vects
#
#        	R= 0 0 1
#           1 0 0
#           0 1 0
#------------------------------------------------------

#TODO: TO MAKE THIS LISTENER LESS GENERIC (:ONLY Image OBJ TO WATCH...)
#      PUT THIS INTO A CLASS
#       ALSO, GENERIC PARAM PASSING COULD BE AVOIDED
#TODO: REMOVE GENERIC CALLBACK PASSING

def listener(myCam,myTop,myType,myCallk):
    node_name='camera_listener'
    rospy.init_node(node_name, anonymous=False)
    loadCameraParam(myCam)
    print('node: {}'.format(node_name))
    print('camera: {}'.format(myCam))
    print('input topic: {}'.format(myTop))

    rospy.Subscriber(myCam+myTop,myType,myCallk,queue_size = 1)
    rospy.Publisher('aruco_bridge_opencv', bridge_msg, queue_size=10)
    rospy.Service('cv_server', cv_server, callback_service)
    
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

"""
bibliography
    (1)
compressed images
cast in np array and cv2.imdecode
http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber
   (2)
https://answers.ros.org/question/249775/display-compresseddepth-image-python-cv2/
  (3)
https://strawlab.github.io/python-pcl/
"""