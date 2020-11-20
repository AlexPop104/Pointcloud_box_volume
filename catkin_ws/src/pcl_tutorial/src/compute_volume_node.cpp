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

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

class ComputeVolumeNode
{
public:
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  ComputeVolumeNode()
  {

    bool ok2;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_linii(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_proiectii(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::PointCloud<pcl::PointXYZ> all_lines[4][4];
    pcl::PointCloud<pcl::PointXYZ>::Ptr all_planes[4];
    pcl::PointCloud<pcl::PointXYZ>::Ptr all_projected_lines[4][4];

    float Coeficients[3][4];

    float Volum=1;

    pub1_ = nh_.advertise<sensor_msgs::PointCloud2>("/output_plan", 1);
    pub2_ = nh_.advertise<sensor_msgs::PointCloud2>("/output_proiectii", 1);
    sub_ = nh_.subscribe("/pf_out", 1, &ComputeVolumeNode::cloudCallback, this);
    //config_server_.setCallback(boost::bind(&ComputeVolumeNode::dynReconfCallback, this, _1, _2));

    vis_pub = nh_.advertise<visualization_msgs::Marker>( "/Volum_final", 0 );

    
  }

  ~ComputeVolumeNode() {}

  void euclidean_segmenting(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f, bool &ok2)
  {


    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    



    vg.setInputCloud(cloud);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*cloud_filtered);
    // std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl; //*

    // Create the segmentation object for the planar model and set all the parameters

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.02);

    int i = 0, nr_points = (int)cloud_filtered->points.size();

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
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
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(false);

    // Write the planar inliers to disk
    extract.filter(*cloud_plane);
    //std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative(true);
    extract.filter(*cloud_f);
    *cloud_filtered = *cloud_f; //     HERE IS THE CLOUD FILTERED EXTRACTED

    if (cloud_filtered->size() != 0)
    {
      // Creating the KdTree object for the search method of the extraction
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud(cloud_filtered);

      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      ec.setClusterTolerance(0.02); // 2cm
      ec.setMinClusterSize(100);
      ec.setMaxClusterSize(25000);
      ec.setSearchMethod(tree);
      ec.setInputCloud(cloud_filtered);
      ec.extract(cluster_indices);

      for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
          cloud_cluster->points.push_back(cloud_filtered->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        cloud = cloud_cluster;
        ok2 = 1;
      }

      ////////////////////////////////////
    }
    else
    {
      ok2 = 0;
    }
  }

  void planar_segmenting(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,float Coeficients[3][4], pcl::PointCloud<pcl::PointXYZ>::Ptr all_planes[4],pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final , int t, bool &ok2)
  {
     pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmented(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr outliers(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr outliers_segmented(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    // Segment dominant plane
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, *inliers, *cloud_segmented);

    if (inliers->indices.size() == 0)
    {

      PCL_ERROR("Could not estimate a planar model for the given dataset.");
      ok2 = 0;
    }

    else
    {

      Coeficients[t - 1][0] = coefficients->values[0];
      Coeficients[t - 1][1] = coefficients->values[1];
      Coeficients[t - 1][2] = coefficients->values[2];
      Coeficients[t - 1][3] = coefficients->values[3];

      *cloud_final += *cloud_segmented;

      all_planes[t] = cloud_segmented;

      ok2 = 1;
    }
  }

  void create_lines(float Coeficients[3][4], pcl::PointCloud<pcl::PointXYZ>::Ptr all_planes[4],pcl::PointCloud<pcl::PointXYZ> all_lines[4][4],pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_linii,bool &ok2)
  {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    std::string str;
    std::string str2;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);

    int i, j;

    if (ok2 != 0)
    {

      for (i = 1; i < 4; i++)
      {

        cloud = all_planes[i];

        // Create a set of planar coefficients with X=Y=0,Z=1
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
        coefficients->values.resize(4);

        for (j = 1; j < 4; j++)
        {
          if (j != i)
          {
            /*
        std::cout << "\n";
        std::cout << "plan " << i << "\n";
         */
            coefficients->values[0] = Coeficients[j - 1][0];
            coefficients->values[1] = Coeficients[j - 1][1];
            coefficients->values[2] = Coeficients[j - 1][2];
            coefficients->values[3] = Coeficients[j - 1][3];

            /*  
        std::cout << "Projecting plane " << i << " to plane " << j << "\n";
        std::cout << "Saving line " << i << "_" << j << "\n";
         */
            // Create the filtering object
            pcl::ProjectInliers<pcl::PointXYZ> proj;
            proj.setModelType(pcl::SACMODEL_PLANE);
            proj.setInputCloud(cloud);
            proj.setModelCoefficients(coefficients);
            proj.filter(*cloud_projected);

            //PCL_INFO("Saving the projected Pointcloud \n");

            all_lines[i][j] = *cloud_projected;

            *cloud_linii = *cloud_linii + *cloud_projected;
          }
        }
      }
    }
    else
    {
      std::cout << "Cannot segment"
                << "\n";
    }
  }

  void project_line_2_plane(float Coeficients[3][4], pcl::PointCloud<pcl::PointXYZ>::Ptr all_planes[4],pcl::PointCloud<pcl::PointXYZ> all_lines[4][4],pcl::PointCloud<pcl::PointXYZ>::Ptr all_projected_lines[4][4],pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_proiectii)
  {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    int i, j;

    int aux;

    for (i = 1; i < 3; i++)
    {
      for (j = i; j < 4; j++)
      {
      }
    }

    for (i = 1; i < 3; i++)
    {
      for (j = i; j < 4; j++)
      {

        if (i != j)
        {

          aux = (6 - i - j);

          // Create a set of planar coefficients with X=Y=0,Z=1
          pcl::ModelCoefficients::Ptr coefficients2(new pcl::ModelCoefficients());
          coefficients2->values.resize(4);

          coefficients2->values[0] = Coeficients[aux - 1][0];
          coefficients2->values[1] = Coeficients[aux - 1][1];
          coefficients2->values[2] = Coeficients[aux - 1][2];
          coefficients2->values[3] = Coeficients[aux - 1][3];

          int aux2;
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_c(new pcl::PointCloud<pcl::PointXYZ>);

          for (int z = 1; z < 3; z++)
          {

            aux2 = i;
            i = j;
            j = aux2;

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
           

            *cloud = all_lines[i][j];

            // Create the filtering object
            pcl::ProjectInliers<pcl::PointXYZ> proj;
            proj.setModelType(pcl::SACMODEL_PLANE);
            proj.setInputCloud(cloud);
            proj.setModelCoefficients(coefficients2);
            proj.filter(*cloud_projected);

            *cloud_c += *cloud;
            *cloud_c += *cloud_projected;

            *cloud_proiectii += *cloud_c;
          }

          all_projected_lines[i][j] = cloud_c;
        }
      }
    }
  }

  void compute_volume(pcl::PointCloud<pcl::PointXYZ>::Ptr all_projected_lines[4][4],float &Volum)
  {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    float muchii[3];

    int i, j;

    for (i = 1; i < 3; i++)
    {
      for (j = 1; j < 4; j++)
      {

        if (i < j)
        {

          cloud = all_projected_lines[i][j];

          float minim_x = cloud->points[0].x;
          int index_min_x = 0;

          float minim_y = cloud->points[0].y;
          int index_min_y = 0;

          float minim_z = cloud->points[0].z;
          int index_min_z = 0;

          float maxim_x = cloud->points[0].x;
          int index_max_x = 0;

          float maxim_y = cloud->points[0].y;
          int index_max_y = 0;

          float maxim_z = cloud->points[0].z;
          int index_max_z = 0;

          for (int nIndex = 0; nIndex < cloud->points.size(); nIndex++)
          {
            if (minim_x > cloud->points[nIndex].x)
            {
              minim_x = cloud->points[nIndex].x;
              index_min_x = nIndex;
            }

            if (minim_y > cloud->points[nIndex].y)
            {
              minim_y = cloud->points[nIndex].y;
              index_min_y = nIndex;
            }

            if (minim_z > cloud->points[nIndex].z)
            {
              minim_z = cloud->points[nIndex].z;
              index_min_z = nIndex;
            }

            if (maxim_x < cloud->points[nIndex].x)
            {
              maxim_x = cloud->points[nIndex].x;
              index_max_x = nIndex;
            }

            if (maxim_y < cloud->points[nIndex].y)
            {
              maxim_y = cloud->points[nIndex].y;
              index_max_y = nIndex;
            }

            if (maxim_z < cloud->points[nIndex].z)
            {
              maxim_z = cloud->points[nIndex].z;
              index_max_z = nIndex;
            }
          }

          float Sortare[3];

          Sortare[0] = abs(maxim_x - minim_x);
          Sortare[1] = abs(maxim_y - minim_y);
          Sortare[2] = abs(maxim_z - minim_z);

          float maximum = Sortare[0];

          float Puncte[2][3];

          int t = 0;

          Puncte[0][0] = index_min_x;
          Puncte[1][0] = index_max_x;
          Puncte[0][1] = index_min_y;
          Puncte[1][1] = index_max_y;
          Puncte[0][2] = index_min_z;
          Puncte[1][2] = index_max_z;

          for (int q = 0; q < 3; q++)
          {
            if (maximum < Sortare[q])
            {
              maximum = Sortare[q];
              t = q;
            }
          }

          int pozitie_min = Puncte[0][t];
          int pozitie_max = Puncte[1][t];

          float distanta;

          float distanta_x = (cloud->points[pozitie_max].x - cloud->points[pozitie_min].x);
          //std::cout<<"Componenta x:"<<distanta_x<<"\n";
          distanta_x = distanta_x * distanta_x;
          //std::cout<<"Componenta x la patrat:"<<distanta_x<<"\n";

          float distanta_y = (cloud->points[pozitie_max].y - cloud->points[pozitie_min].y);
          //std::cout<<"Componenta y:"<<distanta_y<<"\n";
          distanta_y = distanta_y * distanta_y;
          //std::cout<<"Componenta y la patrat:"<<distanta_y<<"\n";

          float distanta_z = (cloud->points[pozitie_max].z - cloud->points[pozitie_min].z);
          //std::cout<<"Componenta z:"<<distanta_z<<"\n";
          distanta_z = distanta_z * distanta_z;
          //std::cout<<"Componenta z la patrat:"<<distanta_z<<"\n";
          /*
        std::cout << "\n";
        std::cout << "Componenta x la patrat:" << distanta_x << "\n";
        std::cout << "Componenta y la patrat:" << distanta_y << "\n";
        std::cout << "Componenta z la patrat:" << distanta_z << "\n";
        std::cout << "\n";
          */
          distanta = distanta_x + distanta_y + distanta_z;

          //std::cout<<"Distanta inainte de SQRT Linia "<<i<<"_"<<j<<" "<<distanta<<"\n";

          //std::cout << "\n";

          distanta = sqrt(distanta_x + distanta_y + distanta_z);

          std::cout << "Distanta finala " << i << "_" << j << " " << distanta << "\n";

          //std::cout << "\n";

          Volum = Volum * distanta;
        }
      }
    }

    std::cout << "Volum final " << Volum << " m^3"
              << "\n";
  }

void compute_all(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_proiectii,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_linii,float &Volum)
  {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
   
    float Coeficients[3][4];
  
    pcl::PointCloud<pcl::PointXYZ> all_lines[4][4];
    pcl::PointCloud<pcl::PointXYZ>::Ptr all_planes[4];
    pcl::PointCloud<pcl::PointXYZ>::Ptr all_projected_lines[4][4];
  


    bool ok=1;

    bool ok2;

    pcl::PCDWriter writer;

    if (cloud->size()==0){
      ok=0;
    }

    for (int t = 1; (t < 4) && ok; t++)
    {
      
      
      euclidean_segmenting(cloud,cloud_f,ok2);

      if (cloud_f->size()==0){
      ok=0;
      }

      if (ok2!=0) {
      planar_segmenting(cloud_f,Coeficients,all_planes,cloud_final, t, ok2);  //cloud_f is global, so the modifications stay

      cloud = cloud_f;  // Cloud is now the extracted pointcloud

      if (cloud->size()==0){
      ok=0;
      }
      }

      
    }

    if (ok && ok2){
 
      /*     
      std::stringstream ss2, ss3, ss4;
     
      ss2 << "All_planes"
      << ".pcd";

      writer.write<pcl::PointXYZ>(ss2.str(), *cloud_final, false);

      */

      create_lines(Coeficients,all_planes,all_lines,cloud_linii,ok2);

      /*
      ss3 << "All_lines"
      << ".pcd";

      writer.write<pcl::PointXYZ>(ss3.str(), *cloud_linii, false);
      */

      project_line_2_plane(Coeficients,all_planes,all_lines,all_projected_lines,cloud_proiectii);

         /*
       ss4 << "All_projections"
        << ".pcd";

       writer.write<pcl::PointXYZ>(ss4.str(), *cloud_proiectii, false);
                  */


      compute_volume(all_projected_lines,Volum);
    }
    else {
      /*std::cout<<"Not enough planes";*/
    }

    
   
  }


  /*
  void
  dynReconfCallback(pcl_tutorial::compute_volume_nodeConfig &config, uint32_t level)
  {

  }
  */

  void
  cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
  {
    float Volum=1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_proiectii(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_linii(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ> cloud_Test;
    pcl::fromROSMsg(*cloud_msg, cloud_Test);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPTR(new pcl::PointCloud<pcl::PointXYZ>);
    *cloudPTR = cloud_Test;

    compute_all(cloudPTR,cloud_final,cloud_proiectii,cloud_linii,Volum);

    sensor_msgs::PointCloud2 tempROSMsg;
    sensor_msgs::PointCloud2 tempROSMsg2;

  


    pcl::toROSMsg(*cloud_final, tempROSMsg);
    pcl::toROSMsg(*cloud_proiectii, tempROSMsg2);


    

    tempROSMsg.header.frame_id = "camera_depth_optical_frame";
    tempROSMsg2.header.frame_id = "camera_depth_optical_frame";

    std::stringstream ss;

    ss<<"Volumul este "<<Volum;



      visualization_msgs::Marker marker;
       marker.header.frame_id = "camera_depth_optical_frame";
      marker.header.stamp = ros::Time::now();
      marker.pose.position.x = 1;
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.text=ss.str();
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose.position.y = 1;
      marker.pose.position.z = 1;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.a = 1.0; // Don't forget to set the alpha! Otherwise it is invisible
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.lifetime = ros::Duration();



 

    //Publish the data
          
    pub1_.publish(tempROSMsg);
    pub2_.publish(tempROSMsg2);
    vis_pub.publish( marker);

    cloud_final->clear();
    cloud_linii->clear();
    cloud_proiectii->clear();

    Volum=1;
  }

private:

bool ok2;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;


pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_linii;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_proiectii;

pcl::SACSegmentation<pcl::PointXYZ> seg;
pcl::PointIndices::Ptr inliers;
pcl::ModelCoefficients::Ptr coefficients;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane;

 pcl::PointCloud<pcl::PointXYZ> all_lines[4][4];
  pcl::PointCloud<pcl::PointXYZ>::Ptr all_planes[4];
  pcl::PointCloud<pcl::PointXYZ>::Ptr all_projected_lines[4][4];

  float Coeficients[3][4];

  float Volum;




  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub1_;
  ros::Publisher pub2_;

  ros::Publisher vis_pub;
  //dynamic_reconfigure::Server<pcl_tutorial::compute_volume_nodeConfig> config_server_;

  //ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "compute_volume_node");

  ComputeVolumeNode vf;

  ros::spin();
}
