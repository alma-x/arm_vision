#!/usr/bin/env python3
"""
author:     FRANCESCO CERRI
            francesco.cerri2@studio.unibo.it
            github.com/gnoccoalpesto

for:        ALMA-X ROVER TEAM
            github.com/alma-x

            EUROPEAN ROVER CHALLENGE

BIG THANKS TO FEDERICO ZECCHI WHO HELPED JUMPSTART THIS CODE'S FIRST VERSION


# WHAT THIS CODE DOES ########################
- ACQUIRES IMAGE FROM A CAMERA (IN THIS CASE, GRIPPER'S CAMERA)
- SELECTED A DICTIONARY, DETECTS ALL ARUCO MARKERS IN THE IMAGE
- FOR EACH MARKER, PUBLISHES POSITION AND ORIENTATION WRT CAMERA


# CODE'S STRUCTURE

class
    init
    subscribers
    detector function
    geometric function
    cosmethic function
"""
import rospy
import numpy as np
import sys
import os
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
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

class MarkerDetector:
    def __init__(self) -> None:

        self.MAX_VISIBLE_MARKERS=14
        self.detected_markers_count=0        
        self.bridge_topic='aruco_bridge_opencv'
        self.markers_topic='?'
        self.input_topic=""
        self.bridge_pub = rospy.Publisher(self.bridge_topic, bridge_msg, queue_size=1)
        self.markers_pub=rospy.Publisher(self.markers_topic,Float32MultiArray,queue_size=self.MAX_VISIBLE_MARKERS)
        self.input_sub=rospy.Subscriber(self.input_topic,Image,queue_size=1)
        self.do_exit=False

        self.ARUCO_PARAMS = aruco.DetectorParameters_create()


        self.supported_aruco_dicts={ 'original':aruco.DICT_ARUCO_ORIGINAL
                                    ,'51000':aruco.DICT_5X5_1000
                                    ,'61000':aruco.DICT_6X6_1000
                                    ,'71000':aruco.DICT_7X7_1000
                                    }

        self.DEFAULT_SELECTED_DICT='original'
        self.loadArucoDict(self.DEFAULT_SELECTED_DICT)

        self.CAMERA_MATR=np.ndarray
        self.CAMERA_DISTORSION_COEFS=np.ndarray
        self.CAMERA_FOCAL_LEN=np.ndarray
        self.target_id=int
        self.target_size=float
        self.cvbridge=CvBridge()


    def loadArucoDict(self,dict_name):
        """
        load data relatively to selected dictionary
        :param dict_name: string identifying name of dictionary;
                            as in self.supported_aruco_dicts
        """
        requested_dict=self.supported_aruco_dicts[dict_name]
        self.ARUCO_DICT=aruco.Dictionary_get(requested_dict)
    

    def loadCameraParams(self,camera_name):
        """
        load intrinsecal parameters of the camera
        params will be retrieved from camera_name/camera_info topic
        NOTE: no callback since it is supposed that params won't change in 

        :param camera_name: string identifying camera name;
                            must appear in the published ones
        """
        print('loading intrinsecal params from camera: {}'.format(camera_name))
        info_topic=camera_name+"/camera_info"
        info_msg=rospy.wait_for_message(info_topic,CameraInfo)

        self.CAMERA_MATR=np.reshape(info_msg.K,[3,3])
        self.CAMERA_DISTORSION_COEFS=info_msg.D
        self.CAMERA_FOCAL_LEN=np.mean([np.ravel(self.CAMERA_MATR[0])[0],np.ravel(self.CAMERA_MATR[1])[1]])
    

 
    def cameraCallback(img_msg):
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
        # msg.aruco_found=[False,False,False,False,False,False,False,False,False,False,False,False,False,False]
        msg.aruco_found=[False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False]
        
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


TODO: what about subs aruco.DICT... name with my convention 'original'
self.used_panel_arucos_dict={'button 1':         {'id':1,'size':50,'aruco_dict':aruco.DICT_ARUCO_ORIGINAL},
                             'button 2':         {'id':2,'size':50,'aruco_dict':aruco.DICT_ARUCO_ORIGINAL},
                             'button 3':         {'id':3,'size':50,'aruco_dict':aruco.DICT_ARUCO_ORIGINAL},
                             'button 4':         {'id':4,'size':50,'aruco_dict':aruco.DICT_ARUCO_ORIGINAL},
                             'button 5':         {'id':5,'size':50,'aruco_dict':aruco.DICT_ARUCO_ORIGINAL},
                             'button 6':         {'id':6,'size':50,'aruco_dict':aruco.DICT_ARUCO_ORIGINAL},
                             'button 7':         {'id':7,'size':50,'aruco_dict':aruco.DICT_ARUCO_ORIGINAL},
                             'button 8':         {'id':8,'size':50,'aruco_dict':aruco.DICT_ARUCO_ORIGINAL},
                             'button 9':         {'id':9,'size':50,'aruco_dict':aruco.DICT_ARUCO_ORIGINAL},
                             'imu':              {'id':10,'size':40,'aruco_dict':aruco.DICT_ARUCO_ORIGINAL},
                             'imu storage':      {'id':11,'size':50,'aruco_dict':aruco.DICT_ARUCO_ORIGINAL},
                             'inspection panel': {'id':9,'size':50,'aruco_dict':aruco.DICT_ARUCO_ORIGINAL},
                             'lid':              {'id':9,'size':40,'aruco_dict':aruco.DICT_ARUCO_ORIGINAL},
                             'lid storage':      {'id':9,'size':50,'aruco_dict':aruco.DICT_ARUCO_ORIGINAL}
                            }

def receiveTargetRequest():
   selected_target=readSomewhere(target_request_topic)
   (target_id,target_size)=loadTargetData(selected_target)

def loadTargetData(selected_target):
   return used_panel_arucos_dict[requestString]


"""