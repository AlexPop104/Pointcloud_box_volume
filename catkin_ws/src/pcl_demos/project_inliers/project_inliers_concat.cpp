#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <fstream>
#include <string>  
#include <math.h>

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_c (new pcl::PointCloud<pcl::PointXYZ>);

int i,j;

	std::cout<<"i=";
	std::cin>>i;
	std::cout<<"j=";
	std::cin>>j;
	
	int aux;
  
	
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
	for (int z=1;z<3;z++){
		
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
  
  
   
  
  
   // Copy the point cloud data
 

*cloud_c += *cloud;
*cloud_c += *cloud_projected;



  
}

pcl::PCDWriter writer;
 
  std::stringstream ss;
  
  ss <<"Projected_Item_"<<i<<j<<".pcd";
  
  writer.write<pcl::PointXYZ> (ss.str (), *cloud_c, false);
  
  std::cout<<ss.str()<<"\n";
  
 /* 
  
  pcl::PointXYZ minPt, maxPt;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr Test_minim (new pcl::PointCloud<pcl::PointXYZ>);
  
  pcl::getMinMax3D (*cloud_c, minPt, maxPt);
  
  Test_minim=*minPt;
  
  
  std::stringstream ss_test;
  
  ss_test <<"Test.pcd";
  
  writer.write<pcl::PointXYZ> (ss_test.str (), Test_minim, false);
  
  
  /*
  std::cout << "Max x: " << maxPt.x << std::endl;
  std::cout << "Max y: " << maxPt.y << std::endl;
  std::cout << "Max z: " << maxPt.z << std::endl;
  std::cout << "Min x: " << minPt.x << std::endl;
  std::cout << "Min y: " << minPt.y << std::endl;
  std::cout << "Min z: " << minPt.z << std::endl;
  
  
  double sortat[3];
  char litera[3];
  
  sortat[0]=abs(maxPt.x-minPt.x);
  sortat[1]=abs(maxPt.y-minPt.y);
  sortat[2]=abs(maxPt.z-minPt.z);
  
  litera[0]='x';
  litera[1]='y';
  litera[2]='z';
  
  double maximum=-50000000;
  double valoare_finala;
  j=0;
  
  for(i=0;i<3;i++){
	  std::cout<<sortat[i]<<"\n";
	  if(sortat[i]>maximum){
		  
		  maximum=sortat[i];
		  j=i;
		  }
	  }
	  
	valoare_finala=maximum;
	
	std::cout<<"maximum "<<litera[j]<<" :"<<valoare_finala;
	* 
	**/


  return (0);
}
