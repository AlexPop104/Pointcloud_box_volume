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
  
  

int i,j;

int aux;

	for(i=1;i<3;i++){
		for(j=1;j<4;j++){
			
			
			
			if (i!=j){
				
				aux=(6-i-j);
	
	// Create a set of planar coefficients with X=Y=0,Z=1
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  coefficients->values.resize (4);
  
  std::stringstream str2;
  
  
    
  str2 << "coeficienti_plan_"<<aux<<".txt";
  
  //std::cout<<str2.str();
 
  std::ifstream Coeficienti(str2.str());
  
  int t=0;
  
for (std::string line; std::getline(Coeficienti, line);){
	 std::istringstream in(line);
	 in>>coefficients->values[t];
	 //std::cout<<coefficients->values[t];
	 t++;
	}
	
	int aux2;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_c (new pcl::PointCloud<pcl::PointXYZ>);
	
	for (int z=1;z<3;z++){
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
		 
		
		aux2=i;
		i=j;
		j=aux2;
	
	std::stringstream str;
    
    str << "Linie_"<<i<<"_"<<j<<".pcd";

	

	pcl::PCDReader reader;
	reader.read (str.str(), *cloud);

  // Create the filtering object
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);
  
  
*cloud_c += *cloud;
*cloud_c += *cloud_projected;


pcl::PCDWriter writer;
 
  std::stringstream ss;
  
  if (j<i){
	  str.str("");
	  str << "Linie_"<<j<<"_"<<i<<".pcd";
  }
	  
  
	    
  ss <<"Projected_Item"<<str.str();
  
  writer.write<pcl::PointXYZ> (ss.str (), *cloud_c, false);
  
  std::cout<<ss.str()<<"\n";
  
				
				
				
				}
			
			
			
			
			
			
			
			
			}
		
		
		
		}
	
	
  
	
	
}
  

  return (0);
}
