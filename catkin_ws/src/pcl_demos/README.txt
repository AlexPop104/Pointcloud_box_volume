PCL Tutorials 
=============

This is a collection of sample codes illustrating some of the basic functionality of PCL.  All 
of these programs were adapted from tutorials at http://pointclouds.org/documentation/tutorials/
or from the code samples provided by several PCL workshops at academic conferences.  My 
contribution has been to collect these examples in one place and streamline them so that they 
form a comprehensive primer for using PCL.

PCL Installation Instructions: 
------------------------------ 

1. Before you begin, you should have a system that is compatible with PCL. PCL installs most 
easily on Ubuntu, but prebuilt binaries are available for MS Windows and Mac OSX, but require you 
to install dependencies separately. 

2. If you already have ROS installed on your system, you most likely already have a version of PCL
installed.  However, you may experience conflicts in running some of the examples with the ROS
version of PCL (notably the openni_grabber tutorial), and installing a standalone version as well
may not resolve these issues.

3. Regardless of your OS, you can also choose to use an Ubuntu virtual machine and install PCL
within that. Unfortunately, VirtualBox does not support OpenNI RGBD cameras at this time (other VM
managers may), so the openni_grabber tutorial also will not work if you go this route.

4. Once you've arranged your system, go to: http://pointclouds.org/downloads/ and follow the
corresponding directions. Installing prebuilt binaries is recommended, but you can also choose to
compile from source if you like.

Tutorials Installation Instructions: 
------------------------------------ 
1. If you're reading this README, you've probably already cloned this repository, but if not, 
navigate to your desired install directory and run: 
    git clone https://jdelmerico@bitbucket.org/jdelmerico/pcl_tutorials.git

2. Navigate into the pcl_tutorials directory, then execute the following commands to build the
examples: 
    mkdir build 
    cd build 
    cmake ..  
    make

3. Assuming the examples all built correctly, you should have several executables in your build
directory.  These examples can be executed as follows:

I/O:
    - write a pcd file of randomly generated points and visualize it with pcd_viewer 
        ./write_pcd
        pcd_viewer test_pcd.pcd
    - read in that same pcd file 
        ./read_pcd
    - grab point clouds from an RGBD camera (be sure one is connected) 
        ./openni_grabber

3D Features:
    - compute point normals on a point cloud and use built-in visualizer 
        ./compute_normals

Filtering:
    - run one of 3 different filters on a point cloud 
        ./filtering 0  (pass through filter)
        ./filtering 1  (downsample to a voxel grid) 
        ./filtering 2  (perform statistical outlier removal)
    - visualize the output side-by-side with the original 
        pcd_viewer -multiview 1 ../data/table_scene_lms400.pcd table_scene_lms400_filtered.pcd 
      press 'r' to zero the viewpoint, and 'l' to list the color handlers

Keypoints:
    - Find SIFT keypoints in a point cloud and visualize 
        ./keypoints ../data/robot1.pcd keypoints
    - Compute PFH features on SIFT keypoints for two point clouds and then compute their
      correspondences 
        ./keypoints ../data/robot correspondences

Trees:
    - Put some random points into a KdTree; do NN and radius search near a random point in space
        ./kdtree
    - Put some random points into an Octree; do voxel, NN, and radius search near a random point in
      space 
        ./octree

Sample Consensus:
    - Generate some points that fit a planar model as well as a bunch of outliers 
        ./sample_consensus
    - Generate points as before, but use sample consensus to find inliers to a planar model
        ./sample_consensus -f

Segmentation:
    - Perform iterative plane segmentation on real point cloud data 
        ./plane_segmentation
    - Visualize the output side-by-side with the original 
        pcd_viewer -multiview 1 ../data/table_scene_lms400.pcd table_scene_lms400_first_plane.pcd 
            table_scene_lms400_second_plane.pcd
    - Perform euclidean cluster extraction after removing the dominant planes in the scene
        ./euclidean_cluster_extraction
    - Visualize the output with all clusters in the same viewport 
        pcd_viewer cloud_cluster_0.pcd cloud_cluster_1.pcd cloud_cluster_2.pcd cloud_cluster_3.pcd 
            cloud_cluster_4.pcd 

Registration:
    - Perform iterative closest point to align two point clouds 
        ./icp ../data/robot1.pcd ../data/robot2.pcd 
    - Visualize aligned and combined point cloud beside originals 
        pcd_viewer -multiview 1 ../data/robot1.pcd ../data/robot2.pcd icp_aligned.pcd
    - Attempt to fit several point cloud templates to the target point cloud, output the best match
        ./template_matching ../data/object_templates.txt ../data/person.pcd 
    - Visualize the matched and aligned template against the target PC 
        pcd_viewer ../data/person.pcd template_aligned.pcd 
      you may need to press '1' several times to get a good color scheme for the two point clouds 
      to be visible



PCL has much, much more functionality than what is demonstrated in these introductory tutorials, and
its scope is constantly growing. I hope these examples give you a base from which to start
experimenting with 3D data processing. Also, I welcome any feedback on these examples (to jad12 at
buffalo dot edu).

-Jeff Delmerico

		
