cmake_minimum_required(VERSION 2.8)		#指定cmake版本 
project(AFCFaceDemo_Linux)				#指定项目的名称，一般和项目的文件夹名称对应
find_package( OpenCV REQUIRED )
set(CMAKE_CXX_STANDARD 11)				#设置c++标准

#指定头文件目录
include_directories(/home/domistic/catkin_ws/src/xm_vision/src/)	
include_directories(/home/domistic/catkin_ws/src/xm_vision/src/inc)
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories (
	/home/domistic/Vision/Camera/lib
	/home/domistic/freenect2/include
	/usr/local/include
  )

#指定静态和动态文件目录
link_directories(/home/domistic/catkin_ws/src/xm_vision/src/linux_so
 /home/domistic/Vision/Camera/lib
)

#该指令的主要作用就是将指定的源文件生成链接文件，然后添加到工程中去。
#arcsoft_face生成的库文件名		[STATIC | SHARED | MODULE] 库类型
add_executable(Gender gender.cpp
        /home/domistic/catkin_ws/src/xm_vision/src/inc/amcomdef.h
        /home/domistic/catkin_ws/src/xm_vision/src/inc/arcsoft_face_sdk.h
		/home/domistic/catkin_ws/src/xm_vision/src/inc/asvloffscreen.h
		/home/domistic/catkin_ws/src/xm_vision/src/inc/merror.h
		/home/domistic/catkin_ws/src/xm_vision/src/inc/samplecode.h)

#-fPIC	在给定的作用域内设置一个命名的属性
set_property(TARGET Gender PROPERTY POSITION_INDEPENDENT_CODE ON)


#链接库文件
target_link_libraries(Gender	
		arcsoft_face
		arcsoft_face_engine
		${OpenCV_LIBS}
	/home/domistic/freenect2/lib/libfreenect2.so
	/home/domistic/Vision/Camera/lib/lib/libcamera.so
        )
