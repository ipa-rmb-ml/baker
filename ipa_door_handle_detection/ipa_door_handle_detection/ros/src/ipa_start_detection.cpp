
#include "ipa_start_detection.h"
#include "ipa_door_handle_segmentation.h"
#include "ipa_door_handle_template_alignment.h"



StartHandleDetection::StartHandleDetection(ros::NodeHandle nh, sensor_msgs::PointCloud2::Ptr point_cloud_out_msg) :
nh_(nh), point_cloud_out_msg_(point_cloud_out_msg)
{
	filePathXYZRGB_ = "/home/rmb-ml/Desktop/PointCloudData/templateDataPCAXYZRGB/"; // only for testing -> change later
	filePathNormals_ = "/home/rmb-ml/Desktop/PointCloudData/templateDataNormals/"; 
	filePathFeatures_ = "/home/rmb-ml/Desktop/PointCloudData/templateDataFeatures/";
	filePathPCATransformations_ = "/home/rmb-ml/Desktop/PointCloudData/templateDataPCATrafo/";

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


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cylinder_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr assumed_handle_cloud_pca (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr template_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);


	//========SEGMENTATION==================
		
		// create segmentation object
		PointCloudSegmentation segObj;
		// segment point cloud and detect planes
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
		Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > assumed_door_handles_pc_vec = segObj.segmentPointCloud(pointcloud_in_pcl_format);

	//=====CYLINDER FITTING ========================00

		planeInformation planeData = segObj.detectPlaneInPointCloud(pointcloud_in_pcl_format);

		pcl::ModelCoefficients::Ptr plane_coefficients = planeData.plane_coeff;

	//===========TEMPLATEALIGNMENT===========
		// load templates into pc vector

		FeatureCloudGeneration featureObj;

		// decide on which template you would like to operate
		// 1. XYZ
		// 2. Normals and Features 

		// 1. xyz
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
		Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > template_door_handles_pc_vec_xyz = featureObj.loadGeneratedTemplatePCLXYZ(filePathXYZRGB_);

		// 2. normals
		std::vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr,
		Eigen::aligned_allocator<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> > template_door_handles_pc_vec_features = featureObj.loadGeneratedTemplatePCLFeatures(filePathFeatures_);

		// 3. features
		std::vector<pcl::PointCloud<pcl::Normal>::Ptr,
		Eigen::aligned_allocator<pcl::PointCloud<pcl::Normal>::Ptr> >template_door_handles_pc_vec_normals = featureObj.loadGeneratedTemplatePCLNormals(filePathNormals_);

		// 4. PCA
		std::vector<Eigen::Matrix4f> template_door_handles_pca_vec = featureObj.loadGeneratedPCATransformations(filePathPCATransformations_);

	   // Iterate over assumed_door_handles_pc_vec and template_door_handles_pc_vec to compare by...
	   // 1. ICP
	   // 2. 3D Features

	// check if normal vec size is equal to to feature vec size
	if (template_door_handles_pc_vec_features.size()==template_door_handles_pc_vec_normals.size())
	{
		int template_vec_size  = template_door_handles_pc_vec_features.size();

		for (int num_cluster_assumed = 0; num_cluster_assumed < assumed_door_handles_pc_vec.size (); ++num_cluster_assumed) // 
		{
			for (int num_cluster_template = 0; num_cluster_template < template_vec_size; ++num_cluster_template) // template loop
			{
				if (assumed_door_handles_pc_vec[num_cluster_assumed]->points.size() > 0)
				{	
					pcl::PointCloud<pcl::Normal>::Ptr template_cloud_normals = template_door_handles_pc_vec_normals[num_cluster_template];
					pcl::PointCloud<pcl::FPFHSignature33>::Ptr template_cloud_features = template_door_handles_pc_vec_features[num_cluster_template];
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr template_cloud = template_door_handles_pc_vec_xyz[num_cluster_template];

					pcl::PointCloud<pcl::PointXYZRGB>::Ptr assumed_handle_cloud = assumed_door_handles_pc_vec[num_cluster_assumed];		
					assumed_handle_cloud = featureObj.downSamplePointCloud(assumed_handle_cloud);
					pcl::PointCloud<pcl::Normal>:: Ptr assumed_handle_cloud_normals =  featureObj.calculateSurfaceNormals(assumed_handle_cloud);


					// perform PCA to estimate the axes with highst variance
					// pointcloud transformed --> centroid of point cloud new coordinate system
					Eigen::Matrix4f pca_transform_assumed =  segObj.calculatePCA(assumed_handle_cloud);
					// apply transformation
					pcl::transformPointCloud (*assumed_handle_cloud, *assumed_handle_cloud_pca, pca_transform_assumed);



					// load pca_transformation from TXT File

					//Eigen::Matrix4f pca_transform_template = template_door_handles_pca_vec[num_cluster_template];







			//	if (icp_transformation.setZero())	
				//	{
						std::cout << "Door handle detected" << std::endl;
						cylinder_point_cloud = segObj.alignCylinderToPointCloud(assumed_handle_cloud,assumed_handle_cloud_normals,plane_coefficients);
				//	}
				//	else
				//	{
						//continue;	
					//}


				*published_pc+= *assumed_handle_cloud_pca;//*assumed_door_handles_pc_vec[numCluster];
			
				*published_pc+= *template_cloud;

				}
			}
		}

	}

	//============VISUALIZATION =========
	// publish changed point cloud
	//	if (!assumed_door_handles_pc_vec.empty())
		//{	
			// concentrate all cluster for visualization
		//	for (int numCluster = 0; numCluster < assumed_door_handles_pc_vec.size (); ++numCluster)
		//	{			
	


				
			
		//	}

			pcl::toROSMsg(*published_pc, *point_cloud_out_msg_);
			point_cloud_out_msg_->header.frame_id = "camera_link";
			pub_.publish(point_cloud_out_msg_);
//		}

	//============VISUALIZATION ========
	
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
				//	Eigen::Matrix4f icp_transformation = featureObj.icpBasedTemplateAlignment(assumed_handle_cloud,template_cloud);
				//	Eigen::Matrix4f icp_norm_transformation  = featureObj.featureBasedTemplateAlignment(assumed_handle_cloud,assumed_handle_cloud_normals,template_cloud,template_cloud_features,template_cloud_normals);

