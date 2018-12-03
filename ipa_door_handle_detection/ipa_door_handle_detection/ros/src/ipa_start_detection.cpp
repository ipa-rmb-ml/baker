
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
		
	pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
	//create new point cloud in pcl format: pointcloud_in_pcl_format
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_in_pcl_format(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr published_pc(new pcl::PointCloud<pcl::PointXYZRGB>);

	//transform imported pointcloud point_cloud_msg to pointcloud_in_pcl_format
	pcl::fromROSMsg(*point_cloud_msg, *pointcloud_in_pcl_format);

	// ==================== ACTUAL CALCULATION:START ==========================================================================================================

	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> >assumed_door_handles_pc_vec;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr  assumed_handle_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);


	// ======================= TEMPLATE =========================================================================================================================
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr  template_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr  template_cloud_normals (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr template_cloud_features (new pcl::PointCloud<pcl::FPFHSignature33>);
	
	std::vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> >template_door_handles_pc_vec_features;
	std::vector<pcl::PointCloud<pcl::Normal>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<pcl::Normal>::Ptr> >template_door_handles_pc_vec_normals;
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> >template_door_handles_pc_vec_xyz;
	// ==========================================================================================================================================================
	
	
	// file path containing pcd files with stored templates
	const std::string filePathXYZRGB = "/home/rmb-ml/Desktop/PointCloudData/templateDataXYZRGB/";
	const std::string filePathNormals = "/home/rmb-ml/Desktop/PointCloudData/templateDataNormals/";
	const std::string filePathFeatures = "/home/rmb-ml/Desktop/PointCloudData/templateDataFeatures/";


	//========SEGMENTATION==================
		
		// create segmentation object
		PointCloudSegmentation segObj;
		// segment point cloud and detect planes
		assumed_door_handles_pc_vec = segObj.segmentPointCloud(pointcloud_in_pcl_format);

	//=====CYLINDER FITTING ========================00

		planeInformation planeData = segObj.detectPlaneInPointCloud(pointcloud_in_pcl_format);
		pcl::ModelCoefficients::Ptr cylinder_coefficients;
		pcl::ModelCoefficients::Ptr plane_coefficients;
		plane_coefficients = planeData.plane_coeff;

	//===========TEMPLATEALIGNMENT===========
		// load templates into pc vector

		FeatureCloudGeneration featureObj;

		// decide on which template you would like to operate
		// 1. XYZ
		// 2. Normals and Features 

		// 1. xyz
		template_door_handles_pc_vec_xyz = featureObj.loadGeneratedTemplatePCLXYZ(filePathXYZRGB);

		// 2. normals
		template_door_handles_pc_vec_features = featureObj.loadGeneratedTemplatePCLFeatures(filePathFeatures);

		// 3. features
		template_door_handles_pc_vec_normals = featureObj.loadGeneratedTemplatePCLNormals(filePathNormals);


	   // Iterate over assumed_door_handles_pc_vec and template_door_handles_pc_vec to compare by...
	   // 1. ICP
	   // 2. 3D Features

	// check if normal vec size is equal to to feature vec size
	if (template_door_handles_pc_vec_features.size()==template_door_handles_pc_vec_normals.size())
	{
		int template_vec_size  = template_door_handles_pc_vec_features.size();

		for (int numClusterAssumed = 0; numClusterAssumed < assumed_door_handles_pc_vec.size (); ++numClusterAssumed) // 
		{
			for (int numClusterTemplate = 0; numClusterTemplate < template_vec_size; ++numClusterTemplate) // template loop
			{

				if (assumed_door_handles_pc_vec[numClusterAssumed]->points.size() > 0)
				{	
						template_cloud_normals = template_door_handles_pc_vec_normals[numClusterTemplate];
						template_cloud_features = template_door_handles_pc_vec_features[numClusterTemplate];
						template_cloud = template_door_handles_pc_vec_xyz[numClusterTemplate];

						assumed_handle_cloud = assumed_door_handles_pc_vec[numClusterAssumed];

					  	assumed_handle_cloud = featureObj.downSamplePointCloud(assumed_handle_cloud);


					pcl::PointCloud<pcl::Normal>:: Ptr assumed_handle_cloud_normals =  featureObj.calculateSurfaceNormals(assumed_handle_cloud);

					bool handle_detected_icp = featureObj.icpBasedTemplateAlignment(assumed_handle_cloud,template_cloud);
					bool handle_detected_nf  = featureObj.featureBasedTemplateAlignment(assumed_handle_cloud,assumed_handle_cloud_normals,template_cloud,template_cloud_features,template_cloud_normals);

					
					if ((handle_detected_icp && handle_detected_nf) ==1)	
					{
						std::cout << "Door handle detected" << std::endl;
						pcl::ModelCoefficients::Ptr cylinder_coefficients = segObj.alignCylinderToPointCloud(assumed_handle_cloud,assumed_handle_cloud_normals);
						segObj.checkOrientationAndGeometryOfCylinder(cylinder_coefficients,plane_coefficients);

				
				
					}
					else
					{
						std::cout<< "Template does not match" << std::endl;
						continue;
						
					}

				}
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
