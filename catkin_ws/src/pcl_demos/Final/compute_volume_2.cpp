#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/project_inliers.h>




// Read in the cloud data
  



pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_linii (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_proiectii (new pcl::PointCloud<pcl::PointXYZ>);

pcl::SACSegmentation<pcl::PointXYZ> seg;
pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());

pcl::PointCloud<pcl::PointXYZ>::Ptr all_lines[4][4];
pcl::PointCloud<pcl::PointXYZ>::Ptr all_planes[4];
pcl::PointCloud<pcl::PointXYZ>::Ptr all_projected_lines[4][4];


float Coeficients[3][4];


void euclidean_segmenting(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
  
 
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
 
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      //////
      ///////////
      //////////
      // TREBUIE FUNCTIE DE OPRIRE
      /////////////
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Write the planar inliers to disk
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

 
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    cloud=cloud_cluster;
  }

  
  ////////////////////////////////////
}


void planar_segmenting(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int t) {


 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmented (new pcl::PointCloud<pcl::PointXYZ>);
 pcl::PointCloud<pcl::PointXYZ>::Ptr outliers (new pcl::PointCloud<pcl::PointXYZ>);
 pcl::PointCloud<pcl::PointXYZ>::Ptr outliers_segmented (new pcl::PointCloud<pcl::PointXYZ>);

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
  }

pcl::PCDWriter writer2;
 std::stringstream ss;
  
  /*
  ss << "plan_" <<t << ".pcd";
  
  writer2.write<pcl::PointXYZ> (ss.str (), *cloud_segmented, false); 


   std::stringstream fisier;

   
  
  fisier <<"coeficienti_plan_"<<t<<".txt";

  std::ofstream myfile;
  myfile.open (fisier.str());
 
 */

  Coeficients[t-1][0]=coefficients->values[0];
  Coeficients[t-1][1]=coefficients->values[1];
  Coeficients[t-1][2]=coefficients->values[2];
  Coeficients[t-1][3]=coefficients->values[3];

  /*
  myfile<<coefficients->values[0] << '\n' 
  << coefficients->values[1] << "\n"
  << coefficients->values[2] << "\n "
  << coefficients->values[3] ; 
	myfile.close();
  */
 
 
  *cloud_final+=*cloud_segmented;

  all_planes[t]=cloud_segmented;


}


void create_lines() {

 std::string str;
 std::string str2;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);


 int i,j;
  
  for(i=1;i<4;i++){

  /*  
	std::cout<<"i="<<i<<" ";	  
    str="plan_"+std::to_string(i)+".pcd";

   // Fill in the cloud data
  pcl::PCDReader reader;
  reader.read (str, *cloud);
  */

  cloud=all_planes[i];

  // Create a set of planar coefficients with X=Y=0,Z=1
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  coefficients->values.resize (4);
  
  for(j=1;j<4;j++){
  if (j!=i){

    std::cout<<"\n";
    std::cout<<"plan "<<j<<"\n";

    coefficients->values[0]=Coeficients[j-1][0];   
    coefficients->values[1]=Coeficients[j-1][1];
    coefficients->values[2]=Coeficients[j-1][2];
    coefficients->values[3]=Coeficients[j-1][3];

    

    for (int t=0;t<4;t++){

    std::cout<<coefficients->values[t]<<"\n";
    }


/*

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

  */

 

  // Create the filtering object
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);

                        
  PCL_INFO("Saving the projected Pointcloud \n");
  
 pcl::PCDWriter writer3;
 std::stringstream ss;
  
  ss << "Linie_"<<std::to_string(i)<<"_"<<std::to_string(j)<< ".pcd";

  all_lines[i][j]= cloud_projected;
  
  writer3.write<pcl::PointXYZ> (ss.str (),  *all_lines[i][j], false);
  //writer3.write<pcl::PointXYZ> (ss.str (),  *cloud_projected, false);

  *cloud_linii=*cloud_linii+*cloud_projected;

 }
}
}
}


void project_line_2_plane(){

    int i,j;

int aux;

	for(i=1;i<3;i++){
		for(j=1;j<4;j++){
			
			
			
			if (i!=j){
				
				aux=(6-i-j);
	
	// Create a set of planar coefficients with X=Y=0,Z=1
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  coefficients->values.resize (4);

  std::cout<<"\n";
    std::cout<<"plan "<<j<<"\n";

    coefficients->values[0]=Coeficients[aux-1][0];   
    coefficients->values[1]=Coeficients[aux-1][1];
    coefficients->values[2]=Coeficients[aux-1][2];
    coefficients->values[3]=Coeficients[aux-1][3];

    

    for (int t=0;t<4;t++){
    std::cout<<coefficients->values[t]<<"\n";
    }


 /*
  
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

  */
	
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


 
  std::stringstream ss;
  
  if (j<i){
	  str.str("");
	  str << "Linie_"<<j<<"_"<<i<<".pcd";
  }
	  
  pcl::PCDWriter writer4;
	    
  ss <<"Projected_Item"<<str.str();
  
  writer4.write<pcl::PointXYZ> (ss.str (), *cloud_c, false);
  
  std::cout<<ss.str()<<"\n";

  *cloud_proiectii+=*cloud_c;
  
				
				
				
				}
			
			
			
			
			
			
			
			
			}
		
		
		
		}
	
	
  
	
	
}
}


void compute_volume() {

 float Volum=1;

  float muchii[3];
  
  
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

std::cout<<"Linia "<<i<<"_"<<j<<"\n";


int pozitie_min=Puncte[0][t];
int pozitie_max=Puncte[1][t];

//std::cout<<"Pozitie punct minim:"<<  pozitie_min  <<"\n";
//std::cout<<"Pozitie punctul maxim:"<<pozitie_max;

//std::cout<<"\n";


std::cout<<"Coordonate punct minim:"<<cloud->points[pozitie_min].x<<" "<<cloud->points[pozitie_min].y<<" "<<cloud->points[pozitie_min].z<<"\n";
std::cout<<"Coordonate punct maxim:"<<cloud->points[pozitie_max].x<<" "<<cloud->points[pozitie_max].y<<" "<<cloud->points[pozitie_max].z;

std::cout<<"\n";

float distanta;

float distanta_x= (cloud->points[pozitie_max].x-cloud->points[pozitie_min].x);
//std::cout<<"Componenta x:"<<distanta_x<<"\n";
distanta_x= distanta_x*distanta_x;
//std::cout<<"Componenta x la patrat:"<<distanta_x<<"\n";

float distanta_y= (cloud->points[pozitie_max].y-cloud->points[pozitie_min].y);
//std::cout<<"Componenta y:"<<distanta_y<<"\n";
distanta_y= distanta_y*distanta_y;
//std::cout<<"Componenta y la patrat:"<<distanta_y<<"\n";

float distanta_z= (cloud->points[pozitie_max].z-cloud->points[pozitie_min].z);
//std::cout<<"Componenta z:"<<distanta_z<<"\n";
distanta_z= distanta_z*distanta_z;
//std::cout<<"Componenta z la patrat:"<<distanta_z<<"\n";

std::cout<<"\n";
std::cout<<"Componenta x la patrat:"<<distanta_x<<"\n";
std::cout<<"Componenta y la patrat:"<<distanta_y<<"\n";
std::cout<<"Componenta z la patrat:"<<distanta_z<<"\n";
std::cout<<"\n";

distanta=distanta_x+distanta_y+distanta_z;

//std::cout<<"Distanta inainte de SQRT Linia "<<i<<"_"<<j<<" "<<distanta<<"\n";

std::cout<<"\n";


distanta=sqrt(distanta_x+distanta_y+distanta_z);

std::cout<<"Distanta finala "<<i<<"_"<<j<<" "<<distanta<<"\n";

std::cout<<"\n";

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


	
std::cout<<"Volum final "<<Volum<<" m^3"<<"\n";

}





int 
main (int argc, char** argv)
{
	
  pcl::PCDReader reader;
  pcl::PCDWriter writer;


 reader.read ("vedere.pcd", *cloud);
 std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

  for (int t=1;t<4;t++){
euclidean_segmenting(cloud);
planar_segmenting(cloud_f,t);

cloud=cloud_f;
}



  create_lines();
  project_line_2_plane();
  compute_volume();


  std::stringstream ss2,ss3,ss4;
  
  ss2 << "All_planes"<< ".pcd";
  
  writer.write<pcl::PointXYZ> (ss2.str (), *cloud_final, false);

  ss3 << "All_lines"<< ".pcd";
  
  writer.write<pcl::PointXYZ> (ss3.str (), *cloud_linii, false);

  ss4 << "All_projections"<< ".pcd";
  
  writer.write<pcl::PointXYZ> (ss4.str (), *cloud_proiectii, false);

  return (0);
}
