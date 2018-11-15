#include <ipa_door_handle_detection.h>



PointCloudImport::PointCloudImport(ros::NodeHandle nh, sensor_msgs::PointCloud2::Ptr point_cloud_out_msg) :
		nh_(nh), point_cloud_out_msg_(point_cloud_out_msg)
{
	std::cout << "Initialising PointCloudImport Constructor." << std::endl;

	pub_ = nh_.advertise<sensor_msgs::PointCloud2>("point_cloud_output",1);
	(pub_) ? std::cout << "Pub is valid." << std::endl : std::cout << "Pub is not valid." << std::endl;
	ros::Subscriber point_cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, &PointCloudImport::pointcloudCallback, this);
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

	std::cout << "PointCloudImport Constructor Initialised." << std::endl;
}




void PointCloudImport::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg)
{

	//create new point cloud in pcl format: pointcloud_in_pcl_format
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_in_pcl_format(new pcl::PointCloud<pcl::PointXYZ>);

	//transform imported pointcloud point_cloud_msg to pointcloud_in_pcl_format
	pcl::fromROSMsg(*point_cloud_msg, *pointcloud_in_pcl_format);
	pointcloud_in_pcl_format->header.frame_id = "pointcloud_in_pcl_format";
	//====change point cloud here======================================

	//create new point cloud in pcl format: point_cloud_colored
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
	// apply color change and store the new pc in point_cloud_colored
	point_cloud_colored = changePointCloudColor(pointcloud_in_pcl_format);
	point_cloud_colored->header.frame_id = "colored_pc";

	//create new point cloud in pcl format: point_cloud_plane
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>);
	point_cloud_plane = detectPlaneInPointCloud(point_cloud_colored);
	point_cloud_plane->header.frame_id = "plane_pc";

	// write result in final point cloud which is going to be published

	//===changed point cloud===========================================

	//transform point_cloud_colored too ROS message to publish it 
	pcl::toROSMsg(*point_cloud_plane, *point_cloud_out_msg_);


	// assigne header id: important to visualize in rviz 
	point_cloud_out_msg_->header.frame_id = "camera_link";
	pub_.publish(point_cloud_out_msg_);

}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudImport::changePointCloudColor(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
	 //std::cout << ">>> Starting PC color change." << std::endl;
	uint8_t r = 0, g = 0, b = 255;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointXYZRGB pclPoint;


	pcl::PointCloud<pcl::PointXYZ>::iterator it;
	for(it = input_cloud->points.begin();it < input_cloud->points.end();++it)
	{
		pclPoint.x = it->x;
		pclPoint.y = it->y;
		pclPoint.z = it->z;

		pclPoint.r = r;
		pclPoint.g = g;
		pclPoint.b = b;

		pointcloud_xyzrgb->points.push_back(pclPoint);
	}

	//std::cout << "<<< Ending PC color change." << std::endl;
	return pointcloud_xyzrgb;
}



pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudImport::detectPlaneInPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud)
{
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_point_cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;

	// color of plane points
 	uint8_t r = 255, g = 0, b = 0;

	pcl::ModelCoefficients::Ptr plane_coeff (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);

	seg.setInputCloud (input_cloud);
	// get inliers
	seg.segment (*inliers, *plane_coeff);


	pcl::PointXYZRGB pclPoint_plane;


	// visualize planes in red
	for (size_t i = 0; i < inliers->indices.size (); ++i)
	{
		pclPoint_plane.x = input_cloud->points[inliers->indices[i]].x;
		pclPoint_plane.y = input_cloud->points[inliers->indices[i]].y;
		pclPoint_plane.z = input_cloud->points[inliers->indices[i]].z;
		pclPoint_plane.r = r;
		pclPoint_plane.b = g;
		pclPoint_plane.g = b;

		segmented_point_cloud_rgb->points.push_back(pclPoint_plane);
	}


	// calculate point to plane distance

	pcl::PointXYZRGB pp_PC;
	double min_dist=0.1;

  	for (size_t i = 0; i < input_cloud->points.size (); ++i)
	{

		pp_PC.x = input_cloud->points[i].x;
		pp_PC.y = input_cloud->points[i].y;
		pp_PC.z = input_cloud->points[i].z;

		double p2p_distance =  pcl::pointToPlaneDistanceSigned (pp_PC, plane_coeff->values[0], plane_coeff->values[1], plane_coeff->values[2], plane_coeff->values[3]);

		if (p2p_distance > min_dist){

			pp_PC.r = 0;																								
			pp_PC.b = 255;
			pp_PC.g =0;								

			segmented_point_cloud_rgb->points.push_back(pp_PC);

		}
	}

	return segmented_point_cloud_rgb;
}




//===main=================================================================================================================
int main(int argc, char **argv)
{
	ros::init(argc, argv, "my_pcl_image_listener");
	ros::NodeHandle nh;


	sensor_msgs::PointCloud2::Ptr point_cloud_out_msg(new sensor_msgs::PointCloud2);
	PointCloudImport PointCloudImport(nh, point_cloud_out_msg);
	return 0;
}

