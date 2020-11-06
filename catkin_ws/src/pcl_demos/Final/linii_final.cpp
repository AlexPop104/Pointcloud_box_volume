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
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDWriter writer;


  
  std::string str;
  std::string str2;
  
  int i,j;
  
  
  
  
  for(i=1;i<4;i++){
	  
	std::cout<<"i="<<i<<" ";	  
    str="plan_"+std::to_string(i)+".pcd";

   // Fill in the cloud data
  pcl::PCDReader reader;
  reader.read (str, *cloud);


  // Create a set of planar coefficients with X=Y=0,Z=1
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  coefficients->values.resize (4);
  
  for(j=1;(j<4) ;j++){
  if (j!=i){
  
	std::cout<<"j="<<j<<"\n";
  str2="coeficienti_plan_"+std::to_string(j)+".txt";
  
  
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

                        
  PCL_INFO("Saving the projected Pointcloud \n");
  
 
  std::stringstream ss;
  
  ss << "Linie_"<<std::to_string(i)<<"_"<<std::to_string(j)<< ".pcd";
  
  writer.write<pcl::PointXYZ> (ss.str (), *cloud_projected, false);

  *cloud_final=*cloud_final+*cloud_projected;

 
		  
		  
		  
		  
	}	  
 }
}	

 std::stringstream ss2;
  
  ss2 << "All_lines.pcd";
  
  writer.write<pcl::PointXYZ> (ss2.str (), *cloud_final, false);


  return (0);
}
