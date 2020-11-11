cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcl_tutorials)

find_package(PCL 1.2 REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable (openni_grabber openni_grabber/openni_grabber.cpp)
target_link_libraries (openni_grabber ${PCL_LIBRARIES})

add_executable (compute_volume Final/compute_volume.cpp)
target_link_libraries (compute_volume ${PCL_LIBRARIES})

add_executable (compute_volume_2 Final/compute_volume_2.cpp)
target_link_libraries (compute_volume_2 ${PCL_LIBRARIES})

add_executable (compute_volume_3 Final/compute_volume_3.cpp)
target_link_libraries (compute_volume_3 ${PCL_LIBRARIES})


add_executable (intersectii_find_line Intersectii/intersectii_find_line.cpp)
target_link_libraries (intersectii_find_line ${PCL_LIBRARIES})

add_executable (intersectii_find_line_all Intersectii/intersectii_find_line_all.cpp)
target_link_libraries (intersectii_find_line_all ${PCL_LIBRARIES})

add_executable (intersectii_find_point Intersectii/intersectii_find_point.cpp)
target_link_libraries (intersectii_find_point ${PCL_LIBRARIES})
