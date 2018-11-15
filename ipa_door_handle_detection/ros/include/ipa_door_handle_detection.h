#include <iostream>
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/sac_model_plane.h>


#include <pcl/common/projection_matrix.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/thread/mutex.hpp>


class pointCloudImport
{

public:
	pointCloudImport(ros::NodeHandle nh, sensor_msgs::PointCloud2::Ptr point_cloud_out_msg);

	void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg);

	//functions to change pointcloud
	//change color of pc
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr changePointCloudColor(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
	//detect planes of pc
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr detectPlaneInPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);



private:
	ros::Publisher pub_;
	ros::NodeHandle nh_;
	sensor_msgs::PointCloud2::Ptr point_cloud_out_msg_;
	ros::Subscriber point_cloud_sub_;
};