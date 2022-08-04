# arm_vision
visual recognition of arm's work environment


TODO

ERRORS:
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
