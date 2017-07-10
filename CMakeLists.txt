cmake_minimum_required(VERSION 2.8.3)
project(raw_video_stream)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  image_transport
  camera_info_manager
  sensor_msgs
)


find_package(OpenCV)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(raw_video_stream src/raw_video_stream.cpp)
target_link_libraries(raw_video_stream ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS raw_video_stream
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS
  scripts/test_raw_video_resource.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
