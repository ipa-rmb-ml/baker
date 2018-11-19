#include <iostream>
#include <ros/ros.h>
#include <utility>

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


#include <pcl/common/projection_matrix.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/sac_model_plane.h>


#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

#include <boost/thread/mutex.hpp>

// struct for storing point cloud plane information

	struct planeInformation
	{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_point_cloud;
	pcl::ModelCoefficients::Ptr plane_coeff;
	};

// class to perform segmentation procedures to differentiate btw door plane and object in the foreground
class PointCloudSegmentation
{
public:

	PointCloudSegmentation(ros::NodeHandle nh, sensor_msgs::PointCloud2::Ptr point_cloud_out_msg);

	void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg);

	//functions to change pointcloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr changePointCloudColor(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr minimizePointCloudToObject(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_pc,pcl::ModelCoefficients::Ptr plane_coeff);
	planeInformation detectPlaneInPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr findClustersByRegionGrowing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr reduced_pc,pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_pc);

private:
	ros::Publisher pub_;
	ros::NodeHandle nh_;
	sensor_msgs::PointCloud2::Ptr point_cloud_out_msg_;
	ros::Subscriber point_cloud_sub_;
};


