cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_auto_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
  geometry_msgs
  visualization_msgs
  sensor_msgs
  tf_conversions
  tf
)

add_message_files(
  FILES
  CarControl.msg
  CoordinationSignal.msg
  DuckieSensor.msg
  LanePose.msg
#  LEDControl.msg
  LEDDetection.msg
  LEDDetectionArray.msg
  LEDDetectionDebugInfo.msg
  WheelsCmd.msg
  WheelsCmdStamped.msg
  Pose2DStamped.msg
  SignalsDetection.msg
  Twist2DStamped.msg
  AprilTagDetection.msg
  AprilTagDetectionArray.msg
  AprilTagsWithInfos.msg
  TagInfo.msg
  Pixel.msg
  Vector2D.msg
  Segment.msg
  SegmentList.msg
  Rect.msg
  Rects.msg
  SceneSegments.msg
  SourceTargetNodes.msg
  CoordinationClearance.msg
#  IntersectionDetection.msg
#  CoordinationSignal.msg
#  TrafficLightDetection.msg
  VehicleCorners.msg
  VehiclePose.msg
#  VehicleDetection.msg
  FSMState.msg
  BoolStamped.msg
  StopLineReading.msg
  LEDInterpreter.msg
  KinematicsWeights.msg
  KinematicsParameters.msg
  ThetaDotSample.msg
  Vsample.msg
  Trajectory.msg
  ObstacleType.msg
  ObstacleImageDetection.msg
  ObstacleImageDetectionList.msg
  ObstacleProjectedDetection.msg
  ObstacleProjectedDetectionList.msg
#RBIZ
  ImgSignals.msg
)

add_service_files(
  FILES
  SetFSMState.srv
  SetValue.srv
  # Service2.srv
)

# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES turtlebot3_auto_msgs
   CATKIN_DEPENDS
   geometry_msgs
   roscpp rospy
   sensor_msgs
   std_msgs
   message_runtime
   visualization_msgs
#  DEPENDS system_lib
)

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# add_library(turtlebot3_auto_msgs
#   src/${PROJECT_NAME}/turtlebot3_auto_msgs.cpp
# )

# add_dependencies(turtlebot3_auto_msgs ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(turtlebot3_auto_msgs_node src/turtlebot3_auto_msgs_node.cpp)

# add_dependencies(turtlebot3_auto_msgs_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# target_link_libraries(turtlebot3_auto_msgs_node
#   ${catkin_LIBRARIES}
# )

# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(TARGETS turtlebot3_auto_msgs turtlebot3_auto_msgs_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# catkin_add_gtest(${PROJECT_NAME}-test test/test_turtlebot3_auto_msgs.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

# catkin_add_nosetests(test)
