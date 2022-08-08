cmake_minimum_required(VERSION 3.0.2)
project(arm_vision)

find_package(catkin REQUIRED COMPONENTS
  #cv2
  #matplotlib
  #numpy
  #roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  geometry_msgs
  #time
)

add_library(time STATIC IMPORTED)

# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  ArucoPose.msg
  ArucoPoses.msg
  FoundArucos.msg
)

add_service_files(
  FILES
  FoundMarker.srv
  ReferenceAcquisition.srv
)

#msg and srv
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## in contrast to setup.py
catkin_install_python(
  PROGRAMS
  scripts/detector_aruco.py
  scripts/inquirer_aruco.py
  scripts/referencer.py
  # scripts/roscamLibrary3.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
