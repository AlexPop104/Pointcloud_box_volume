#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <fstream>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmented (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr outliers (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr outliers_segmented (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  pcl::PCDReader reader;
  reader.read ("vedere.pcd", *cloud);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  // Segment dominant plane
  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);
  pcl::copyPointCloud<pcl::PointXYZ>(*cloud, *inliers, *cloud_segmented);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }
  
   int j;
   srand((unsigned) time(0));
   
   
  j=(rand() % 5321);

  PCL_INFO("Saving dominant plane in input cloud to: vedere_first_plane.pcd\n");
  pcl::PCDWriter writer;
 
  std::stringstream ss;
  
  ss << "plan_" <<j << ".pcd";
  
  writer.write<pcl::PointXYZ> (ss.str (), *cloud_segmented, false);
  
  std::stringstream fisier;
  
  fisier <<"coeficienti_plan_"<<j<<".txt";
  
  
  std::ofstream myfile;
  myfile.open (fisier.str());
  
  myfile<<coefficients->values[0] << '\n' 
  << coefficients->values[1] << "\n"
  << coefficients->values[2] << "\n "
  << coefficients->values[3] ; 
	myfile.close();

  return (0);
}
