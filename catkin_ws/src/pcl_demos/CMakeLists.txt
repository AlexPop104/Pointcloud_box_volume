cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcl_tutorials)

find_package(PCL 1.2 REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable (openni_grabber openni_grabber/openni_grabber.cpp)
target_link_libraries (openni_grabber ${PCL_LIBRARIES})

add_executable (create_planes Final/create_planes.cpp)
target_link_libraries (create_planes ${PCL_LIBRARIES})

add_executable (distante_final Final/distante_final.cpp)
target_link_libraries (distante_final ${PCL_LIBRARIES})

add_executable (linii_final Final/linii_final.cpp)
target_link_libraries (linii_final ${PCL_LIBRARIES})

add_executable (proiectii_final Final/proiectii_final.cpp)
target_link_libraries (proiectii_final ${PCL_LIBRARIES})
