#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <fstream>
#include <string>  

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);


	std::ifstream file("Input_inliers_PCD.txt");
	std::string str;

	std::getline(file, str);

	pcl::PCDReader reader;
	reader.read (str, *cloud);
  

  // Create a set of planar coefficients with X=Y=0,Z=1
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  coefficients->values.resize (4);
  
  std::ifstream file2("Input_inliers_Coeff.txt");
  std::string str2;
  
  std::getline(file2, str2);
  
  std::ifstream Coeficienti(str2);
  
  int t=0;
  
for (std::string line; std::getline(Coeficienti, line);){
	 std::istringstream in(line);
	 in>>coefficients->values[t];
	 //std::cout<<coefficients->values[t];
	 t++;
	}
  

  // Create the filtering object
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);


pcl::PCDWriter writer;
 
  std::stringstream ss;
  
  ss << "Projected_Item"<<str;
  
  writer.write<pcl::PointXYZ> (ss.str (), *cloud_projected, false);
  

  return (0);
}
