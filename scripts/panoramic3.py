#!/usr/bin/env python3

import rospy
import numpy as np

from sensor_msgs.msg import Image asImage
from sensor_msgs.msg import CameraInfo
#frofm sensor_msgs.msg import PointCloud2 as sensPCld

from arm_control.srv import aruco_service,aruco_serviceResponse
from arm_control.srv import cv_server,cv_serverResponse, cv_serverRequest
from arm_control.msg import cv_to_bridge as bridge_msg

import cv2 as cv
from cv2 import aruco as aruco
#from cv2 import aruco as aruco
from cv_bridge import CvBridge
from roscamLibrary3 import nsingleAruRelPos as singleAruRelPos

#PUBLISHER DI ARRAY:
#aruco_position_pub = rospy.Publisher('/almax/aruco_target',Float64MultiArray,queue_size=20)
#array = [69.1,0,1,33,1,1,1,0]
#robaccia = Float64MultiArray(data=array)
#aruco_position_pub.publish(robaccia)
#------------------------------------------------

pub = rospy.Publisher('aruco_bridge_opencv', bridge_msg, queue_size=1)

ARUCO_PARAMETERS = aruco.DetectorParameters_create()

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
    global cameraMatr
    global cameraDistCoefs
    global cameraFocLen
    
    print('loading camera parameters...')
    cameraInfoMsg=rospy.wait_for_message(myCam+'/color/camera_info',CameraInfo)
    cameraMatr=np.reshape(cameraInfoMsg.K,[3,3])
    cameraDistCoefs=cameraInfoMsg.D
    cameraFocLen=np.mean([np.ravel(cameraMatr[0])[0],np.ravel(cameraMatr[1])[1]])
    
#------------------------------------------------------------
    
#aruTargetDict={'panelSwitch1':(101,
#                        40)
#                ,'panelSwitch2':(102,
#                        40)
#                ,'panelSwitch3':(103,
#                        40)
#                ,'panelSwitch4':(104,
#                        40)
#                ,'panelSwitch5':(105,
#                        40)
#                ,'panelSwitch6':(106,
#                        40)
#                ,'panelSwitch7':(107,
#                        40)
#                ,'panelSwitch8':(108,
#                        40)
#                }


aruco_success=False
#targetList=['panelSwitch8','panelSwitch7','panelSwitch6','panelSwitch5','panelSwitch4'
#            ,'panelSwitch3','panelSwitch2','panelSwitch1']
#targetList=[[101,40],[102,40],[103,40],[104,40],
#            [105,40],[106,40],[107,40],[108,40]]
targetList=[[102,40],[104,40],[106,40],[108,40]]
#global targetCounter
targetCounter=0
remaining_targets=0
targetListLen=len(targetList)

#global findNewTarget
#findNewTarget=1

#def receiveTargetRequest():
#    selectedTarget=read_somewhere(targetRequestTopic)
#    (targetMarkId,targetMarkSize)=loadTargetData(selectedTarget)

#def loadTargetData(requestString):
#    return aruTargetDict[requestString]

global targetMarkId,targetMarkSize
#(targetMarkId,targetMarkSize)=targetMarker=aruTargetDict['panelSwitch3']
#global selectionString
#selectionString=targetList[4]
#(targetMarkId,targetMarkSize)=aruTargetDict[selectionString]
# apparently aruTargetDict[targetList[4]] it's different, even if prints ==


#----------------------------------------
bridge=CvBridge()
 
def callbackRaw(raw_img):
    global aruco_success
    global msgVector
    global msgRotMatrix
    global targetCounter
    global findNewTarget
    global remaining_targets
    
    
    cv_image=bridge.imgmsg_to_cv2(raw_img, desired_encoding='passthrough')
    cv_gray=cv2.cvtColor(cv_image,cv2.COLOR_RGB2GRAY)
    
    
    (targetMarkId,targetMarkSize)=tuple(targetList[targetCounter])
    detCorners, detIds, _ = aruco.detectMarkers(cv_gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
        
    if detIds is not None and len(detIds) >= 1: # Check if at least one marker has been found
        
        detAruImg = aruco.drawDetectedMarkers(cv_image.copy(), detCorners, borderColor=(0, 255, 0))
           
        aruco_success=False 
        for mId, aruPoints in zip(detIds, detCorners):
            if mId==targetList[targetCounter][0]:    
                detAruImg,aruDistnc,Pmatr=singleAruRelPos(detAruImg,aruPoints,mId,targetMarkSize,
                                              cameraMatr,cameraDistCoefs,tglDrawMark=1)
                
                rotMatr,tVect=Pmatr[0:3,0:3],Pmatr[0:3,3]
                msgRotMatrix=rotMatr
                msgVector=tVect
                
                aruco_success=True
                remaining_targets=targetListLen-targetCounter-1
                
    else:
        aruco_success=False
        detAruImg=cv_image.copy()#
    #    newSize,_=int(np.shape(detAruImg))
    #    detAruImg=cv2.resize(detAruImg,newSize)
    cv2.imshow('detected markers',detAruImg)

    msg=bridge_msg()
    msg.success=aruco_success
    if msg.success:
        msg.x=0.001*msgVector[2] +(recovLenRatio*0.08 if tglWristLengthRecovery else 0)
        msg.y=0.001*msgVector[0]
        msg.z=0.001*msgVector[1]
    pub.publish(msg)
    key = cv2.waitKey(12) & 0xFF# key still unused
#    if key == 27:# 27:esc, ord('q'):q
#       exit_somehow()
        
    
#-----------------------------------------------------------------

msgVector=[0,0,0]#np.zeros([1,3])
msgRotMatrix=[[0,0,0,],[0,0,0],[0,0,0]]#np.zeros([3,3])

        
tglWristLengthRecovery=1
# recovered percentage
recovLenRatio=1


    
def listener(myCam,myTop,myType,myCallk):    
    rospy.init_node('camera_listener', anonymous=True)
    loadCameraParam(myCam)
    print('ready')
    rospy.Subscriber(myCam+myTop,myType,myCallk,queue_size = 1)
    try:
        rospy.spin()
    except KeyboardInterrupt:#
        print('Closing')
    cv2.destroyAllWindows()
    
#---------------------------------------------------------------
    
    
camDict={'moving':"/camera_image",
            'fixed':"/camera_image_fix"}

topicDict={'raw':("/color/image_raw",
                   Image,
                    callbackRaw)    
            }   

if __name__ == '__main__':
    myCamera=camDict['fixed']
    myTopicFull=topicDict['raw']
    
    print('connecting to:'+myCamera+myTopicFull[0]+'...')
    listener(myCamera,myTopicFull[0],myTopicFull[1],myTopicFull[2])



#bibliography
#     (1)
#compressed images
#cast in np array and cv2.imdecode
#http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber
#    (2)
#https://answers.ros.org/question/249775/display-compresseddepth-image-python-cv2/
#   (3)
#https://strawlab.github.io/python-pcl/
