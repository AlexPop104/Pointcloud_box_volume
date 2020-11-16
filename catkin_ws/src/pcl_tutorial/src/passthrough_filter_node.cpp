/*
 * passthrough_filter.cpp
 *
 *  Created on: 06.09.2013
 *      Author: goa
 */

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_tutorial/passthrough_filter_nodeConfig.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>


class PassthroughFilterNode
{
public:
  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointCloud<Point> PointCloud;

  PassthroughFilterNode()
  {
    pub_ = nh_.advertise<PointCloud>("pf_out",1);
    sub_ = nh_.subscribe ("point_cloud_in", 1,  &PassthroughFilterNode::cloudCallback, this);
    config_server_.setCallback(boost::bind(&PassthroughFilterNode::dynReconfCallback, this, _1, _2));

    double z_upper_limit, z_lower_limit;
    double x_upper_limit, x_lower_limit;
    double y_upper_limit, y_lower_limit;

    // "~" means, that the node hand is opened within the private namespace (to get the "own" paraemters)
    ros::NodeHandle private_nh("~");

    //read parameters with default value
    private_nh.param("z_lower_limit", z_lower_limit, 2.);
    private_nh.param("z_upper_limit", z_upper_limit, 5.);

    pt_.setFilterFieldName ("z");
    pt_.setFilterLimits (z_lower_limit, z_upper_limit);
    
     //read parameters with default value
    private_nh.param("x_lower_limit", x_lower_limit, 2.);
    private_nh.param("x_upper_limit", x_upper_limit, 5.);

    pt_.setFilterFieldName ("x");
    pt_.setFilterLimits (x_lower_limit, x_upper_limit);
    
    //read parameters with default value
    private_nh.param("y_lower_limit", y_lower_limit, 2.);
    private_nh.param("y_upper_limit", y_upper_limit, 5.);

    pt_.setFilterFieldName ("y");
    pt_.setFilterLimits (y_lower_limit, y_upper_limit);
    
  }

  ~PassthroughFilterNode() {}

  void
  dynReconfCallback(pcl_tutorial::passthrough_filter_nodeConfig &config, uint32_t level)
  {
    pt_.setFilterLimits(config.z_lower_limit, config.z_upper_limit);
  }

  void
  cloudCallback(const PointCloud::ConstPtr& cloud_in)
  {
    pt_.setInputCloud(cloud_in);
    PointCloud cloud_out;
    pt_.filter(cloud_out);
    pub_.publish(cloud_out);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  dynamic_reconfigure::Server<pcl_tutorial::passthrough_filter_nodeConfig> config_server_;

  pcl::PassThrough<Point> pt_;

};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "voxel_filter_node");

  PassthroughFilterNode vf;

  ros::spin();
}

