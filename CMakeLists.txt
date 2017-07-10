cmake_minimum_required(VERSION 2.8.3)
project(raw_video_stream)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  image_transport
  camera_info_manager
  sensor_msgs
  dynamic_reconfigure
  nodelet
)


find_package(OpenCV REQUIRED)
if (OpenCV_VERSION VERSION_EQUAL "3")
  add_definitions("-DOPENCV3=1")
endif()

find_package(Boost REQUIRED COMPONENTS thread)

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(cfg/ProcessFile.cfg)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs
  DEPENDS OpenCV
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
include_directories(include)

add_library(${PROJECT_NAME}
  src/nodelets/process_file.cpp
  src/libraw_video_stream/advertisement_checker.cpp
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})

# Standalone node
add_executable(raw_video_stream_exe src/nodes/raw_video_stream.cpp)
target_link_libraries(raw_video_stream_exe ${PROJECT_NAME}  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})
SET_TARGET_PROPERTIES(raw_video_stream_exe PROPERTIES OUTPUT_NAME raw_video_stream)
install(TARGETS raw_video_stream_exe
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/test_raw_video_resource.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
