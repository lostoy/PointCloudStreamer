cmake_minimum_required(VERSION 2.8)
project(PCL_Streamer)



find_package(Boost REQUIRED)
find_package(PCL 1.2 REQUIRED)
find_package( OpenCV REQUIRED )

add_subdirectory(gpu_utility)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_dependencies(gpu_utility gpu_utility)

add_executable(PointCloudStreamer PointCloudStreamer.cpp SocketServer.cpp SocketServer.hpp )


target_link_libraries( PointCloudStreamer ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} gpu_utility)


