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

add_executable (compute_volume Final/compute_volume.cpp)
target_link_libraries (compute_volume ${PCL_LIBRARIES})

add_executable (compute_volume_2 Final/compute_volume_2.cpp)
target_link_libraries (compute_volume_2 ${PCL_LIBRARIES})


add_executable (distante_final Final/distante_final.cpp)
target_link_libraries (distante_final ${PCL_LIBRARIES})

add_executable (linii_final Final/linii_final.cpp)
target_link_libraries (linii_final ${PCL_LIBRARIES})

add_executable (proiectii_final Final/proiectii_final.cpp)
target_link_libraries (proiectii_final ${PCL_LIBRARIES})

add_executable (intersectii_find_line Intersectii/intersectii_find_line.cpp)
target_link_libraries (intersectii_find_line ${PCL_LIBRARIES})

add_executable (intersectii_find_line_all Intersectii/intersectii_find_line_all.cpp)
target_link_libraries (intersectii_find_line_all ${PCL_LIBRARIES})

add_executable (intersectii_find_point Intersectii/intersectii_find_point.cpp)
target_link_libraries (intersectii_find_point ${PCL_LIBRARIES})
