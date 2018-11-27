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
#include <pcl/filters/project_inliers.h>

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

// creates vector with pointclouds where each represents a cluster idintified by region growing
class PointCloudSegmentation
{
public:

	PointCloudSegmentation();

	//main function og the segmentation class
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > segmentPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr changePointCloudColor(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);

	// detect door plane in pointcloud
	planeInformation detectPlaneInPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);

	// removing all object points that are too distant from the door plane
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr minimizePointCloudToObject(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_pc,pcl::ModelCoefficients::Ptr plane_coeff);
	
	// apply region growing to identifie all cluster in front of the detected door
	std::vector <pcl::PointIndices> findClustersByRegionGrowing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr reduced_pc);

	// generate vector of pointclouds representing each of the cluster
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > generateAlignmentObject(std::vector <pcl::PointIndices> clusters,pcl::PointCloud<pcl::PointXYZRGB>::Ptr reduced_pc,pcl::ModelCoefficients::Ptr plane_coeff);

	// used to project cluster points on detected plane to remove all addition points of the point cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr removePlaneOutlierByClusterOnPlaneProjection(pcl::PointXYZRGB clusterPoint,pcl::ModelCoefficients::Ptr plane_coeff);

private:
};

