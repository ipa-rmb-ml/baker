
#include "ipa_database_generation.h"
#include "ipa_door_handle_segmentation.h"



DoorhandleDatabaseGeneration::DoorhandleDatabaseGeneration(ros::NodeHandle nh, sensor_msgs::PointCloud2::Ptr point_cloud_out_msg) :
nh_(nh), point_cloud_out_msg_(point_cloud_out_msg)
{
	initCameraNode(nh,point_cloud_out_msg);	
}


void DoorhandleDatabaseGeneration::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg)
{
	pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

	//create new point cloud in pcl format: pointcloud_in_pcl_format
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_rgb_trans(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::ModelCoefficients::Ptr plane_coeff (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

	//transform imported pointcloud point_cloud_msg to pointcloud_in_pcl_format
	pcl::fromROSMsg(*point_cloud_msg, *point_cloud);

	PointCloudSegmentation seg;

	planeInformation planeData = seg.detectPlaneInPointCloud(point_cloud);
	inliers = planeData.plane_point_cloud_indices;
 	plane_coeff = planeData.plane_coeff;

	point_cloud_rgb=seg.minimizePointCloudToObject(point_cloud,inliers,plane_coeff);


	/// interpretation of ros data
	pcl::PointXYZRGB pclPoint;
	for (size_t i = 0; i < point_cloud_rgb->points.size (); ++i)
	{
		pclPoint.x = -point_cloud_rgb->points[i].y;
		pclPoint.y = -point_cloud_rgb->points[i].x;
		pclPoint.z = point_cloud_rgb->points[i].z;

		point_cloud_rgb_trans->points.push_back(pclPoint);
	}
	//std::cout << "<<< Ending PC color change." << std::endl;



	pcl::toROSMsg(*point_cloud_rgb_trans, *point_cloud_out_msg_);
	point_cloud_out_msg_->header.frame_id = "camera_link";
	pub_.publish(point_cloud_out_msg_);
}


void DoorhandleDatabaseGeneration::initCameraNode(ros::NodeHandle nh, sensor_msgs::PointCloud2::Ptr point_cloud_out_msg)
{
	std::cout << "Initialising DoorhandleDatabaseGeneration Constructor." << std::endl;
	pub_ = nh_.advertise<sensor_msgs::PointCloud2>("point_cloud_output",1);
	(pub_) ? std::cout << "Pub is valid." << std::endl : std::cout << "Pub is not valid." << std::endl;
	ros::Subscriber point_cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, &DoorhandleDatabaseGeneration::pointcloudCallback, this);
	ros::Duration(1).sleep();

	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	if (!ros::ok()){
		std::cout << "Quit publishing" << std::endl;
	}
	std::cout << "DoorhandleDatabaseGeneration Constructor Initialised." << std::endl;
}
	

// =================================================0
int main(int argc, char **argv)
{		

	ros::init(argc, argv, "my_pcl_image_listener");
	ros::NodeHandle nh;
	sensor_msgs::PointCloud2::Ptr point_cloud_out_msg(new sensor_msgs::PointCloud2);
	DoorhandleDatabaseGeneration DoorhandleDatabaseGeneration(nh, point_cloud_out_msg);

	return 0;
}

