#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <math.h>
#include <pcl/point_types.h>
int 
main (int, char**)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
  
  float Volum=1;
  
  
  int i,j;

	for(i=1;i<3;i++){
		for(j=1;j<4;j++){
			
			if (i<j){
				
				std::stringstream str2;
	
str2 << "Projected_ItemLinie_"<<i<<"_"<<j<<".pcd";
  
  
 pcl::PCDReader reader;
reader.read (str2.str(), *cloud);


float minim_x=cloud->points[0].x;
int index_min_x=0;



float minim_y=cloud->points[0].y;
int index_min_y=0;


float minim_z=cloud->points[0].z;
int index_min_z=0;


float maxim_x=cloud->points[0].x;
int index_max_x=0;



float maxim_y=cloud->points[0].y;
int index_max_y=0;


float maxim_z=cloud->points[0].z;
int index_max_z=0;

/*
std::cout<<"Inceput minim x:"<<minim_x<<" Pozitie "<<index_min_x<<"\n";
std::cout<<"Inceput minim y:"<<minim_y<<" Pozitie "<<index_min_y<<"\n";
std::cout<<"Inceput minim z:"<<minim_z<<" Pozitie "<<index_min_z<<"\n";
std::cout<<"Inceput maxim x:"<<maxim_x<<" Pozitie "<<index_max_x<<"\n";
std::cout<<"Inceput maxim y:"<<maxim_y<<" Pozitie "<<index_max_y<<"\n";
std::cout<<"Inceput maxim z:"<<maxim_z<<" Pozitie "<<index_max_z<<"\n";
*/


for (int nIndex =0;nIndex <cloud ->points.size();nIndex++)
{
	if (minim_x>cloud->points[nIndex].x){
		minim_x=cloud->points[nIndex].x;
		index_min_x=nIndex;
		}
		
	if (minim_y>cloud->points[nIndex].y){
		minim_y=cloud->points[nIndex].y;
		index_min_y=nIndex;
		}
		
	if (minim_z>cloud->points[nIndex].z){
		minim_z=cloud->points[nIndex].z;
		index_min_z=nIndex;
		}
		
	if (maxim_x<cloud->points[nIndex].x){
		maxim_x=cloud->points[nIndex].x;
		index_max_x=nIndex;
		}
		
	if (maxim_y<cloud->points[nIndex].y){
		maxim_y=cloud->points[nIndex].y;
		index_max_y=nIndex;
		}
		
	if (maxim_z<cloud->points[nIndex].z){
		maxim_z=cloud->points[nIndex].z;
		index_max_z=nIndex;
		}
}
std::cout<<"\n";

/*
std::cout<<"Sfarsit minim x:"<<minim_x<<" Pozitie "<<index_min_x<<"\n";
std::cout<<"Sfarsit minim y:"<<minim_y<<" Pozitie "<<index_min_y<<"\n";
std::cout<<"Sfarsit minim z:"<<minim_z<<" Pozitie "<<index_min_z<<"\n";
std::cout<<"Sfarsit maxim x:"<<maxim_x<<" Pozitie "<<index_max_x<<"\n";
std::cout<<"Sfarsit maxim y:"<<maxim_y<<" Pozitie "<<index_max_y<<"\n";
std::cout<<"Sfarsit maxim z:"<<maxim_z<<" Pozitie "<<index_max_z<<"\n";
*/
std::cout<<"\n";


float Sortare[3];


Sortare[0]=abs(maxim_x-minim_x);
Sortare[1]=abs(maxim_y-minim_y);
Sortare[2]=abs(maxim_z-minim_z);

float maximum=Sortare[0];

float Puncte[2][3];

int t=0;

Puncte[0][0]=index_min_x;
Puncte[1][0]=index_max_x;
Puncte[0][1]=index_min_y;
Puncte[1][1]=index_max_y;
Puncte[0][2]=index_min_z;
Puncte[1][2]=index_max_z;


for(int q=0;q<3;q++){
	//std::cout<<Sortare[q]<<"\n";
	if(maximum<Sortare[q]) {
		maximum=Sortare[q];
		t=q;
		}
	}	

//std::cout<<"\n";
	
//std::cout<<t<<"\n";


int pozitie_min=Puncte[0][t];
int pozitie_max=Puncte[1][t];

//std::cout<<"Pozitie punct minim:"<<  pozitie_min  <<"\n";
//std::cout<<"Pozitie punctul maxim:"<<pozitie_max;

//std::cout<<"\n";


//std::cout<<"Coordonate punct minim:"<<cloud->points[pozitie_min].x<<" "<<cloud->points[pozitie_min].y<<" "<<cloud->points[pozitie_min].z<<"\n";
//std::cout<<"Coordonate punct maxim:"<<cloud->points[pozitie_max].x<<" "<<cloud->points[pozitie_max].y<<" "<<cloud->points[pozitie_max].z;

//std::cout<<"\n";

float distanta;

float distanta_x= (cloud->points[pozitie_max].x-cloud->points[pozitie_min].x);
std::cout<<"Componenta x:"<<distanta_x<<"\n";
distanta_x= distanta_x*distanta_x;
std::cout<<"Componenta x la patrat:"<<distanta_x<<"\n";

float distanta_y= (cloud->points[pozitie_max].y-cloud->points[pozitie_min].y);
std::cout<<"Componenta y:"<<distanta_y<<"\n";
distanta_y= distanta_y*distanta_y;
std::cout<<"Componenta y la patrat:"<<distanta_y<<"\n";

float distanta_z= (cloud->points[pozitie_max].y-cloud->points[pozitie_min].z);
std::cout<<"Componenta z:"<<distanta_z<<"\n";
distanta_z= distanta_z*distanta_z;
std::cout<<"Componenta z la patrat:"<<distanta_z<<"\n";

std::cout<<"\n";

distanta=sqrt(distanta_x+distanta_y+distanta_z);

std::cout<<"Distanta finala "<<i<<" "<<j<<" "<<distanta<<"\n";

Volum=Volum*distanta;


pcl::PointCloud<pcl::PointXYZ> new_cloud;

  // Fill in the cloud data
  new_cloud.width    = 2;
  new_cloud.height   = 1;
  new_cloud.is_dense = false;
  new_cloud.resize (new_cloud.width * new_cloud.height);

   new_cloud.points[0].x=cloud->points[pozitie_min].x;
   new_cloud.points[1].x=cloud->points[pozitie_max].x;
   
   new_cloud.points[0].y=cloud->points[pozitie_min].y;
   new_cloud.points[1].y=cloud->points[pozitie_max].y;
   
   new_cloud.points[0].z=cloud->points[pozitie_min].z;
   new_cloud.points[1].z=cloud->points[pozitie_max].z;
  

 
  std::stringstream ss;
  
  ss <<"Distance"<<i<<"_"<<j<<".pcd";
  
  pcl::io::savePCDFileASCII (ss.str(), new_cloud);
				
				
				
			}
			
		}
	}
	
std::cout<<"Volum final "<<Volum<<"\n";



  return (0);
}
