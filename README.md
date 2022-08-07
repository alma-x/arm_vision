# arm_vision
visual recognition of arm's work environment


TODO--------------------------------

REFERENCER
- change reference timer into a request server, requested when robot 
    not moving

- computeXPose() should call a function, passing the correct arguments,
    when possible, hence to avoid repetition of same code too many times

- rotate correctly reference frames:
  IMU: y UP, x TOWARD LEFT

- change to correct MAX_MID_PANEL_ARUCO_ID to 9


ERRORS------------------------

DETECTOR
- sometimes (during exploration)

[ERROR] [1659625745.921060, 4721.782000]: bad callback: <function cameraCallback at 0x7f041559a9d0>
Traceback (most recent call last):
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/topics.py", line 750, in _invoke_callback
    cb(msg)

  File "/catkin_ws/src/almax/arm_vision/scripts/detector_aruco.py", line 136, in cameraCallback
    detAruImg,aruDistnc,Pmatr=singleAruRelPos(detAruImg,aruPoints,mId,targetMarkSize,

  File "/catkin_ws/src/almax/arm_vision/scripts/roscamLibrary3.py", line 259, in nsingleAruRelPos
    rvecs,tvecs,object_points= aruco.estimatePoseSingleMarkers(corners,markerSize_mm,camera_matrix,camera_dist_coefs)
    
cv2.error: OpenCV(4.2.0) ../contrib/modules/aruco/src/aruco.cpp:1064: error: (-215:Assertion failed) markerLength > 0 in function 'estimatePoseSingleMarkers'

INQUIRER+REFERENCER:

when restarting nodes, while objects references already published
Error:   TF_DENORMALIZED_QUATERNION: Ignoring transform for child_frame_id "..." from authority "/referencer" because of an invalid quaternion in the transform (0.000000 0.000000 0.000000 0.000000)
         at line 255 in /tmp/binarydeb/ros-noetic-tf2-0.7.5/src/buffer_core.cpp

...={robot_frame, button_3, button_4}

-> should only specify a quaternion?
