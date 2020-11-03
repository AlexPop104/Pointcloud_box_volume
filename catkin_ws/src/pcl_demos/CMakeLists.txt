cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcl_tutorials)

find_package(PCL 1.2 REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (write_pcd io/write_pcd.cpp)
target_link_libraries (write_pcd ${PCL_LIBRARIES})

add_executable (read_pcd io/read_pcd.cpp)
target_link_libraries (read_pcd ${PCL_LIBRARIES})

add_executable (openni_grabber openni_grabber/openni_grabber.cpp)
target_link_libraries (openni_grabber ${PCL_LIBRARIES})

add_executable (compute_normals 3D_features/compute_normals.cpp)
target_link_libraries (compute_normals ${PCL_LIBRARIES})

add_executable (filtering filtering/filtering.cpp)
target_link_libraries (filtering ${PCL_LIBRARIES})

add_executable (keypoints keypoints/keypoints.cpp)
target_link_libraries (keypoints ${PCL_LIBRARIES})

add_executable (sample_consensus sample_consensus/sample_consensus.cpp)
target_link_libraries (sample_consensus ${PCL_LIBRARIES})

add_executable (plane_segmentation segmentation/plane_segmentation.cpp)
target_link_libraries (plane_segmentation ${PCL_LIBRARIES})

add_executable (euclidean_cluster_extraction segmentation/euclidean_cluster_extraction.cpp)
target_link_libraries (euclidean_cluster_extraction ${PCL_LIBRARIES})

add_executable (kdtree trees/kdtree.cpp)
target_link_libraries (kdtree ${PCL_LIBRARIES})

add_executable (octree trees/octree.cpp)
target_link_libraries (octree ${PCL_LIBRARIES})

add_executable (icp registration/icp.cpp)
target_link_libraries (icp ${PCL_LIBRARIES})

add_executable (template_matching registration/template_matching.cpp)
target_link_libraries (template_matching ${PCL_LIBRARIES})

add_executable (project_lines project_inliers/project_lines.cpp)
target_link_libraries (project_lines ${PCL_LIBRARIES})

add_executable (project_inliers project_inliers/project_inliers.cpp)
target_link_libraries (project_inliers ${PCL_LIBRARIES})

add_executable (project_inliers_concat project_inliers/project_inliers_concat.cpp)
target_link_libraries (project_inliers_concat ${PCL_LIBRARIES})

add_executable (project_inliers_file project_inliers/project_inliers_file.cpp)
target_link_libraries (project_inliers_file ${PCL_LIBRARIES})

add_executable (get_min_max project_inliers/get_min_max.cpp)
target_link_libraries (get_min_max ${PCL_LIBRARIES})

add_executable (create_planes Final/create_planes.cpp)
target_link_libraries (create_planes ${PCL_LIBRARIES})

add_executable (distante_final Final/distante_final.cpp)
target_link_libraries (distante_final ${PCL_LIBRARIES})

add_executable (linii_final Final/linii_final.cpp)
target_link_libraries (linii_final ${PCL_LIBRARIES})

add_executable (proiectii_final Final/proiectii_final.cpp)
target_link_libraries (proiectii_final ${PCL_LIBRARIES})
