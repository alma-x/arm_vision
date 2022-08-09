#!/usr/bin/env python3

from ctypes.wintypes import SMALL_RECT
import rospy
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
from arm_vision.msg import ArucoPoses
import cv2 
from cv2 import aruco as aruco
from cv_bridge import CvBridge, CvBridgeError
# from roscamLibrary3 import nsingleAruRelPos as singleAruRelPos

#------------------------------------------------
camera_poses_topic='/camera_poses'

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
    
    print('loading camera parameters... ',end="")
    # cameraInfoMsg=rospy.wait_for_message(myCam+'/color/camera_info',CameraInfo)
    cameraInfoMsg=rospy.wait_for_message(myCam+'/camera_info',CameraInfo)
    print('ok')
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
            # {'button 1':         {'id':1,'size':50,'aruco_dict':aruco.DICT_ARUCO_ORIGINAL},
            #   'button 2':         {'id':2,'size':50,'aruco_dict':aruco.DICT_ARUCO_ORIGINAL},
            #   'button 3':         {'id':3,'size':50,'aruco_dict':aruco.DICT_ARUCO_ORIGINAL},
            #   'button 4':         {'id':4,'size':50,'aruco_dict':aruco.DICT_ARUCO_ORIGINAL},
            #   'button 5':         {'id':5,'size':50,'aruco_dict':aruco.DICT_ARUCO_ORIGINAL},
            #   'button 6':         {'id':6,'size':50,'aruco_dict':aruco.DICT_ARUCO_ORIGINAL},
            #   'button 7':         {'id':7,'size':50,'aruco_dict':aruco.DICT_ARUCO_ORIGINAL},
            #   'button 8':         {'id':8,'size':50,'aruco_dict':aruco.DICT_ARUCO_ORIGINAL},
            #   'button 9':         {'id':9,'size':50,'aruco_dict':aruco.DICT_ARUCO_ORIGINAL},
            #   'imu':              {'id':10,'size':40,'aruco_dict':aruco.DICT_ARUCO_ORIGINAL},
            #   'imu storage':      {'id':11,'size':50,'aruco_dict':aruco.DICT_ARUCO_ORIGINAL},
            #   'inspection panel': {'id':9,'size':50,'aruco_dict':aruco.DICT_ARUCO_ORIGINAL},
            #   'lid':              {'id':9,'size':40,'aruco_dict':aruco.DICT_ARUCO_ORIGINAL},
            #   'lid storage':      {'id':9,'size':50,'aruco_dict':aruco.DICT_ARUCO_ORIGINAL}
            #   }

#TODO: rework this
#originally: nsingleAruRelPos
def singleAruRelPos(queryImg,corners,Id,markerSize_mm,camera_matrix,camera_dist_coefs, 
                     superimpAru='none',tglDrawMark=0,tglDrawCenter=0):
#    positiion estimation
    rvecs,tvecs,object_points= aruco.estimatePoseSingleMarkers(corners,markerSize_mm,camera_matrix,camera_dist_coefs)
    (rvecs - tvecs).any()  # get rid of that nasty numpy value array error
    
#    distance [mm]
    distnc_mm=np.sqrt((tvecs**2).sum())
#    rotation and projection matrix
    rotation_matrix = cv2.Rodrigues(rvecs)[0]
    P = np.hstack((rotation_matrix, np.reshape(tvecs,[3,1])))
#    euler_angles_degrees = - cv2.decomposeProjectionMatrix(P)[6]
#    euler_angles_radians = euler_angles_degrees * np.pi / 180
    
#    substitute marker with distance of Id
    # if superimpAru=='distance': queryImg=distOverAruco(round(distnc_mm, 1),corners,queryImg)
    # elif superimpAru=='marker': queryImg=IdOverAruco(Id,corners,queryImg)
#    draws axis half of the size of the marker
    if tglDrawMark:
        markerDim_px = np.sqrt((corners[0][0][0] - corners[0][3][0])**2 + (corners[0][0][1] - corners[0][3][1])**2)    
        aruco.drawAxis(queryImg, camera_matrix, camera_dist_coefs, rvecs, tvecs, int(markerDim_px//4))

    if tglDrawCenter:
        centerx,centery=np.abs(corners[0][0] + corners[0][2])/2
        markerDim_px = np.sqrt((corners[0][0][0] - corners[0][3][0])**2 + (corners[0][0][1] - corners[0][3][1])**2)
        queryImg=cv2.circle(queryImg, (int(centerx),int(centery)),int(markerDim_px/16),(255,255,0),-1)
        
    return queryImg,distnc_mm,P

#----------------------------------------

bridge=CvBridge()
DOWNSIZE_COEFF=2
if DOWNSIZE_COEFF<1:DOWNSIZE_COEFF=1

def eulerFromRotatioMatrix(R):
    singularity = np.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    # singularity = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = singularity < 1e-6
    if  not singular :
        np.arctan2
        teta_x = np.arctan2(R[2,1] , R[2,2])
        teta_y = np.arctan2(-R[2,0], singularity)
        teta_z = np.arctan2(R[1,0], R[0,0])
        # teta_x = math.atan2(R[2,1] , R[2,2])
        # teta_y = math.atan2(-R[2,0], sy)
        # teta_z = math.atan2(R[1,0], R[0,0])
        return np.array([teta_x,teta_y,teta_z])
    teta_x = np.arctan2(-R[1,2], R[1,1])
    teta_y = np.arctan2(-R[2,0], singularity)
    teta_z = 0
    return np.array([teta_x,teta_y,teta_z])   

def cameraCallback(raw_img):
    global aruco_pub
    aruco_msg=ArucoPoses()

    try:
        cv_image=bridge.imgmsg_to_cv2(raw_img, desired_encoding='passthrough')
        cv_gray=cv2.cvtColor(cv_image,cv2.COLOR_RGB2GRAY)

        detCorners, detIds, _ = aruco.detectMarkers(cv_gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
            
        if detIds is not None and len(detIds) >= 1:
                aruco_msg=ArucoPoses()
                detAruImg = aruco.drawDetectedMarkers(cv_image.copy(), detCorners, borderColor=(0, 255, 0))
                
                for mId, aruPoints in zip(detIds, detCorners):
                    aruco_pose=Pose()
                    targetMarkSize=getMarkerSizeFromId(targetList,mId)
                    detAruImg,aruDistnc,Pmatr=singleAruRelPos(detAruImg,aruPoints,mId,targetMarkSize,
                                                cameraMatr,cameraDistCoefs,tglDrawMark=1)
                    
                    rotMatr,tVect=Pmatr[0:3,0:3],Pmatr[0:3,3]
                    tVect=np.round(1E-3*tVect,5)
                    distance_aruco=np.round(aruDistnc*1E-3,4)
                    #TODO: RECOVER DISTANCE CAMERA-GRIPPER
                    # ALSO REDEFINE DIMENSIONS IN APPROPRIATE REFERENCE FRAME'S AXIS
                    
                    euler=eulerFromRotatioMatrix(rotMatr)
                    quaternion=quaternion_from_euler(euler[0],euler[1],euler[2])
                    quaternion=np.round(quaternion,5)
                    
                    aruco_pose.position.x=tVect[0]
                    aruco_pose.position.y=tVect[1]
                    aruco_pose.position.z=tVect[2]
                    aruco_pose.orientation.x=quaternion[0]
                    aruco_pose.orientation.y=quaternion[1]
                    aruco_pose.orientation.z=quaternion[2]
                    aruco_pose.orientation.w=quaternion[3]

                    aruco_msg.ids.append(int(mId))
                    aruco_msg.poses.append(aruco_pose)
                    aruco_msg.distances.append(distance_aruco)
                
                if DOWNSIZE_COEFF>1:
                    aruco_img_size=detAruImg.shape[:2]
                    detAruImg=cv2.resize(detAruImg,(aruco_img_size[1]//DOWNSIZE_COEFF,aruco_img_size[0]//DOWNSIZE_COEFF))
                display_image=detAruImg
        else:
            if  DOWNSIZE_COEFF>1:
                cv_img_size=cv_image.shape[:2]
                # aruco_img_size=detAruImg.shape[:2]
                cv_image=cv2.resize(cv_image,(cv_img_size[1]//DOWNSIZE_COEFF,cv_img_size[0]//DOWNSIZE_COEFF))
            display_image=cv_image

        cv2.imshow('detected markers',display_image)

    except CvBridgeError: pass

    aruco_pub.publish(aruco_msg)

    kk = cv2.waitKey(12) & 0xFF
    if kk == 27:
        cv2.destroyAllWindows()
        rospy.signal_shutdown('Esc key pressed')
        
    
#-----------------------------------------------------------------

def detector(myCam,myTop,myType,myCallk):
    node_name='camera_detector'
    rospy.init_node(node_name, anonymous=False)
    print('node: {}'.format(node_name))
    print('camera: {}'.format(myCam))
    print('input topic: {}\n'.format(myTop))

    loadCameraParam(myCam)

    print('connecting to:'+myCamera+myTopicFull[0]+'... ',end="")
    rospy.wait_for_message(myCam+myTop,myType)
    print("ok")
    
    rospy.Subscriber(myCam+myTop,myType,myCallk,queue_size = 1)
    global aruco_pub
    aruco_pub=rospy.Publisher(camera_poses_topic,ArucoPoses,queue_size=1)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        rospy.signal_shutdown('Esc key pressed; Closing node: {}'.format(node_name))
    print('bye')
#---------------------------------------------------------------
    
#TODO? DOES SPLITTING CAMERA AND "TOPIC" NAME HAVE ANY, NON COSMETHIC, MEANING?    
camDict={'moving':"/camera_image",
            'fixed':"/camera_image_fix"}

topicDict={'raw':("/color/image_raw",
                   Image,
                    cameraCallback)    
            }   

################===========================#####################
if __name__ == '__main__':
    print('Detector Node for Panal Aruco Visual Markers\n')

    # myCamera=camDict['moving']
    # myTopicFull=topicDict['raw']
    myCamera="/camera_image"
    myTopicFull=("/image_raw",Image,cameraCallback)
    
    detector(myCamera,myTopicFull[0],myTopicFull[1],myTopicFull[2])
