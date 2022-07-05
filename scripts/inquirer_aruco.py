#!/usr/bin/env python3

import rospy
import numpy as np
import os
from geometry_msgs.msg import Pose
import tf2_ros
from geometry_msgs.msg import TransformStamped

from arm_control.srv import aruco_service,aruco_serviceResponse
from arm_control.srv import cv_server,cv_serverResponse, cv_serverRequest
from arm_control.msg import cv_to_bridge as bridge_msg

pub = rospy.Publisher('aruco_bridge_opencv', bridge_msg, queue_size=1)
bool_exit=False

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

aruco_success=False

targetCounter=0
targetListLen=len(targetList)
remaining_targets=targetListLen

# COMMENTATO
global findNewTarget
findNewTarget=1


#def receiveTargetRequest():
#    selectedTarget=read_somewhere(targetRequestTopic)
#    (targetMarkId,targetMarkSize)=loadTargetData(selectedTarget)


#def loadTargetData(requestString):
#    return aruTargetDict[requestString]


# global targetMarkId,targetMarkSize

available_ids=[]
available_corners=[]

def at_least_one_marker_found():return True

def getArucoPose():
    pose=Pose()
    return pose

def callbackRaw(raw_img):
    global aruco_success
    global msgVector
    global msgRotMatrix
    global targetCounter
    global findNewTarget
    global remaining_targets
    global bool_exit

    if bool_exit: os._exit(os.EX_OK)

    msg=bridge_msg()
    msg.aruco_found=14*[False]

    # (targetMarkId,targetMarkSize)=tuple(targetList[targetCounter])
        
    if at_least_one_marker_found():
        
        aruco_success=False 

        for mId in available_ids:

            if len(msg.aruco_found)>int(mId):
                msg.aruco_found[int(mId)]=True

            if mId==targetList[targetCounter][0]:    
                Pmatr=getArucoPose(mId)
                
                rotMatr,tVect=Pmatr[0:3,0:3],Pmatr[0:3,3]
                msgRotMatrix=rotMatr
                msgVector=tVect
                
                aruco_success=True

                # remaining_targets=targetListLen-targetCounter-1
                #if targetCounter<targetListLen-1:
                #    targetCounter+=1
                
    else:
        aruco_success=False

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



#------------------------------------------------------

def arucoInquirer():

    rospy.Subscriber(,queue_size = 1)
    rospy.Publisher('aruco_bridge_opencv', bridge_msg, queue_size=10)
    rospy.Service('cv_server', cv_server, callback_service)
    
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

