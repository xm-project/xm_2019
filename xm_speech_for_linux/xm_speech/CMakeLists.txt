# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(xm_speech)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   xm_speech_isr.srv
   xm_speech_iat.srv
   xm_Speech_tts.srv
   xm_Speech_meaning.srv
   xm_Speech.srv
 )
## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES xm_speech
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib
)

# include_directories(include)
include_directories(
  include     ##放置讯飞头文件
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  lib         ##放置讯飞链接库
)

## Declare a C++ executable
add_executable(isr_sample src/isr_sample.cpp)
add_executable(tts_sample src/tts_sample.cpp)
add_executable(isr_sample_shopping src/isr_sample_shopping.cpp)
add_executable(isr_sample_who src/isr_sample_who.cpp)
add_executable(isr_sample_question src/isr_sample_question.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(isr_sample xm_speech_gencpp)
add_dependencies(tts_sample xm_speech_gencpp)
add_dependencies(isr_sample_shopping xm_speech_gencpp)
add_dependencies(isr_sample_who xm_speech_gencpp)
add_dependencies(isr_sample_question xm_speech_gencpp)

## Specify libraries to link a library or executable target against
target_link_libraries(isr_sample
   ${catkin_LIBRARIES}
   libmsc.so   ##
 )

target_link_libraries(tts_sample
   ${catkin_LIBRARIES}
   libmsctts.so   ##
 )

target_link_libraries(isr_sample_shopping
   ${catkin_LIBRARIES}
   libmsc.so   ##
 )

target_link_libraries(isr_sample_who
   ${catkin_LIBRARIES}
   libmsc.so   ##
 )

target_link_libraries(isr_sample_question
   ${catkin_LIBRARIES}
   libmsc.so   ##
 )

#set (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS}  -g ")
#set (CMAKE_VERBOSE_MAKEFILE ON)


