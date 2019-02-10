cmake_minimum_required(VERSION 3.6)
project(3DHeadTracker)

set(CMAKE_CXX_STANDARD 11)

find_package( OpenCV REQUIRED )
find_package( Threads REQUIRED )
find_package( Boost REQUIRED COMPONENTS)
find_package( PCL 1.8 REQUIRED )
include_directories(${PCL_INCLUDE_DIRS})

set(SOURCE_FILES main.cpp Camera.hpp Camera.cpp StereoCamera.cpp StereoCamera.hpp LockQueue.cpp LockQueue.hpp)
set(CAMERA_SOURCE_FILES CameraTest.cpp Camera.hpp Camera.cpp LockQueue.cpp LockQueue.hpp)
set(OPENCV_SOURCE_FILES opencv_test.cpp)

add_executable(3DHeadTracker ${SOURCE_FILES})
add_executable(CameraTest ${CAMERA_SOURCE_FILES})
add_executable(opencv_test ${OPENCV_SOURCE_FILES})

target_link_libraries( 3DHeadTracker ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT} ${Boost_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries( CameraTest ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT} ${Boost_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries( opencv_test ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT} ${Boost_LIBRARIES} ${PCL_LIBRARIES})