
#include "ipa_start_detection.h"
#include "ipa_door_handle_segmentation.h"
#include "ipa_template_generation.h"

StartHandleDetection::StartHandleDetection(ros::NodeHandle nh, sensor_msgs::PointCloud2::Ptr point_cloud_out_msg) :
nh_(nh), point_cloud_out_msg_(point_cloud_out_msg)
{
	
	initCameraNode(nh,point_cloud_out_msg);		
}


void StartHandleDetection::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg)
{
	//create new point cloud in pcl format: pointcloud_in_pcl_format
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_in_pcl_format(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr published_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> >clusters_vec_pc,clusters_templates_vec_pc;

	//transform imported pointcloud point_cloud_msg to pointcloud_in_pcl_format
	pcl::fromROSMsg(*point_cloud_msg, *pointcloud_in_pcl_format);


	//========SEGMENTATION==================
		
		// create segmentation object
		PointCloudSegmentation seg;
		// segment point cloud and detect planes
		clusters_vec_pc = seg.segmentPointCloud(pointcloud_in_pcl_format);

	//===========TEMPLATEALIGNMENT===========
		// load templates into pc vector




	//============VISUALIZATION =========

		// publish changed point cloud
		if (!clusters_vec_pc.empty())
		{	
			// concentrate all cluster for visualization
			for (int numCluster = 0; numCluster < clusters_vec_pc.size (); ++numCluster)
			{			
				*published_pc+= *clusters_vec_pc[numCluster];
			}

			pcl::toROSMsg(*published_pc, *point_cloud_out_msg_);
			point_cloud_out_msg_->header.frame_id = "camera_link";
			pub_.publish(point_cloud_out_msg_);
		}
}


void StartHandleDetection::initCameraNode(ros::NodeHandle nh, sensor_msgs::PointCloud2::Ptr point_cloud_out_msg)

{
	std::cout << "Initialising StartHandleDetection Constructor." << std::endl;
	pub_ = nh_.advertise<sensor_msgs::PointCloud2>("point_cloud_output",1);
	(pub_) ? std::cout << "Pub is valid." << std::endl : std::cout << "Pub is not valid." << std::endl;
	ros::Subscriber point_cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, &StartHandleDetection::pointcloudCallback, this);
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

	std::cout << "StartHandleDetection Constructor Initialised." << std::endl;
}
