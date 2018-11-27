
#include "ipa_start_detection.h"
#include "ipa_door_handle_segmentation.h"
#include "ipa_door_handle_template_alignment.h"


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


	//transform imported pointcloud point_cloud_msg to pointcloud_in_pcl_format
	pcl::fromROSMsg(*point_cloud_msg, *pointcloud_in_pcl_format);


	// ==================== ACTUAL CALCULATION:START =====================================================
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> >template_door_handles_pc_vec;
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> >assumed_door_handles_pc_vec;
	
	
	// file path containing pcd files with stored templates
	const std::string filePath = "/home/rmb-ml/Desktop/PointCloudData/templateData/";


	//========SEGMENTATION==================
		
		// create segmentation object
		PointCloudSegmentation segObj;
		// segment point cloud and detect planes
		assumed_door_handles_pc_vec = segObj.segmentPointCloud(pointcloud_in_pcl_format);




	//===========TEMPLATEALIGNMENT===========
		// load templates into pc vector

		FeatureCloudGeneration featureObj;
		template_door_handles_pc_vec = featureObj.loadGeneratedTemplatePCLFiles(filePath);

	   // Iterate over assumed_door_handles_pc_vec and template_door_handles_pc_vec to compare by...
	   // 1. ICP
	   // 2. 3D Features
		for (int numClusterAssumed = 0; numClusterAssumed < assumed_door_handles_pc_vec.size (); ++numClusterAssumed) // 
		{
			for (int numClusterTemplate = 0; numClusterTemplate < template_door_handles_pc_vec.size (); ++numClusterTemplate) // template loop
			{


				if (assumed_door_handles_pc_vec[numClusterAssumed]->points.size() > 0)
				{	
					featureObj.templateAlignmentByICP(assumed_door_handles_pc_vec[numClusterAssumed],template_door_handles_pc_vec[numClusterTemplate]);
					//featureObj.templateAlignmentBy3DFeatures(assumed_door_handles_pc_vec[numClusterAssumed],template_door_handles_pc_vec[numClusterTemplate]);
					// if fit return
				}



			}
		}





	//============VISUALIZATION =========
	// publish changed point cloud
		if (!assumed_door_handles_pc_vec.empty())
		{	
			// concentrate all cluster for visualization
			for (int numCluster = 0; numCluster < assumed_door_handles_pc_vec.size (); ++numCluster)
			{			
				*published_pc+= *assumed_door_handles_pc_vec[numCluster];
			}

			pcl::toROSMsg(*published_pc, *point_cloud_out_msg_);
			point_cloud_out_msg_->header.frame_id = "camera_link";
			pub_.publish(point_cloud_out_msg_);
		}

	//============VISUALIZATION =========



	//=================ACTUAL CALCULATION:END=========================================================


	
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
