project(gps)
#set complier
set(CMAKE_CXX_COMPILER "x86_64-poky-linux-g++")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m64 -march=core2 -mtune=core2 -msse3 -mfpmath=sse --sysroot=/opt/poky/1.8.1/sysroots/core2-64-poky-linux")

cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST}) 

set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS -pthread LINK_FLAGS -pthread)
#find_package(OpenCV REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
