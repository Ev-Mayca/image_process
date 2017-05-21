cmake_minimum_required(VERSION 2.8)
project(stiching)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

link_directories(${PROJECT_SOURCE_DIR}/lib)

find_package(OpenCV 2.4.9 REQUIRED)
find_package(Pangolin REQUIRED)

#include_directories(
#    ${Pangolin_INCLUDE_DIRS}
#)

add_executable(stitching merge.cpp)

target_link_libraries(stitching 
	${OpenCV_LIBS}
    #lcm
    #pthread
    #oabase
    #${Pangolin_LIBRARIES}
    #glog
)
