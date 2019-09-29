cmake_minimum_required(VERSION 2.8.3)
project(xm_vision)

add_definitions(-std=c++11)
ADD_DEFINITIONS(-DOPENCV)

find_package(
catkin REQUIRED COMPONENTS 
  roscpp
  std_msgs
  xm_msgs
)
find_package( OpenCV REQUIRED )
find_library(darknet libdarknet.so /home/domistic/darknet)
catkin_package(
)
include_directories(
   include
    ${catkin_INCLUDE_DIRS}
)
include_directories (
    /home/domistic/catkin_ws/src/xm_vision/src/inc
    /home/domistic/freenect2/include
    /home/domistic/Vision/Camera/lib
    /home/domistic/darknet/include
    /usr/include
    /usr/local/include 
    /usr/local/cuda-8.0/include
  )
aux_source_directory(./src/ DIR_SRC)

add_executable(people_tracking     src/people_tracking.cpp)

 target_link_libraries( people_tracking
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS} 
    ${darknet}
    /home/domistic/freenect2/lib/libfreenect2.so
    /home/domistic/Vision/Camera/lib/lib/libcamera.so
    /home/domistic/darknet/libdarknet.so
 )

add_dependencies(people_tracking ${PROJECT_NAME}_cpp)
