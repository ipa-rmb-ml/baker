
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
	filePathBBInformations_ = "/home/rmb-ml/Desktop/PointCloudData/templateDataBB/";

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


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr assumed_handle_cloud_pca (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr assumed_handle_cloud_pca_normal (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr template_cloud_pca(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr template_cloud_pca_best(new pcl::PointCloud<pcl::PointXYZRGB>);

	//========SEGMENTATION==================
		
	// create segmentation object
	PointCloudSegmentation segObj;
	// segment point cloud and detect planes
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
	Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > assumed_door_handles_pc_vec = segObj.segmentPointCloud(pointcloud_in_pcl_format);

	planeInformation planeData = segObj.detectPlaneInPointCloud(pointcloud_in_pcl_format);
	pcl::ModelCoefficients::Ptr plane_coefficients = planeData.plane_coeff;

	//===========TEMPLATEALIGNMENT===========
		// load templates into pc vector

	FeatureCloudGeneration featureObj;

	std::vector<double> sum_squared_err_vec;
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
	Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > template_cloud_pca_best_vec,assumed_handle_cloud_pca_best_vec;

	double sum_squared_err_min = 0.1;
	double points_ratio_min = 0.8;

	Eigen::Matrix4f final_transformation;

		for (int num_cluster_assumed = 0; num_cluster_assumed < assumed_door_handles_pc_vec.size (); ++num_cluster_assumed) // 
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr assumed_handle_cloud = assumed_door_handles_pc_vec[num_cluster_assumed];		


			// estimate preconditions
			pcaInformation pcaData =  segObj.calculatePCA(assumed_handle_cloud);

			Eigen::Matrix4f pca_assumed = pcaData.pca_transformation;
			Eigen::Vector3f bb_3D = pcaData.bounding_box_3D;


			
		//	double BB_mean =  sqrt(bb_3D(0) * bb_3D(0) + bb_3D (1) * bb_3D (1) + bb_3D (2)* bb_3D (2));

		//	std::cout<<"cluster_diag " << BB_mean<<std::endl;
				
			assumed_handle_cloud = featureObj.downSamplePointCloud(assumed_handle_cloud);

			// apply transformation on assumed handle cloud
			pcl::transformPointCloud (*assumed_handle_cloud, *assumed_handle_cloud_pca, pca_assumed);

			// check for cylinder 
			assumed_handle_cloud_pca_normal =  featureObj.calculateSurfaceNormals(assumed_handle_cloud_pca);
			bool isCylinder = segObj.alignCylinderToPointCloud(assumed_handle_cloud_pca,assumed_handle_cloud_pca_normal,plane_coefficients);
					
				if (!isCylinder)
				{
					continue;
				}

			assumed_handle_cloud_pca_best_vec.push_back(assumed_handle_cloud_pca);
			// determine size of the BB here to define in which template subfolder to go
				// double size ....

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
	
			std::vector<Eigen::Vector3f> template_door_handle_BB_3D = featureObj.loadGeneratedBBInformation(filePathBBInformations_);

			for (int num_cluster_template = 0; num_cluster_template < template_door_handles_pc_vec_xyz.size(); ++num_cluster_template) // template loop
				{
					if (assumed_door_handles_pc_vec[num_cluster_assumed]->points.size() > 0)
					   {	
							pcl::PointCloud<pcl::Normal>::Ptr template_cloud_normals = template_door_handles_pc_vec_normals[num_cluster_template];
							pcl::PointCloud<pcl::FPFHSignature33>::Ptr template_cloud_features = template_door_handles_pc_vec_features[num_cluster_template];
							// already tranformed appling PCA
							pcl::PointCloud<pcl::PointXYZRGB>::Ptr template_cloud_pca = template_door_handles_pc_vec_xyz[num_cluster_template];
							Eigen::Matrix4f pca_template =	 template_door_handles_pca_vec[num_cluster_template]; // rotation and translatoion to origin
							Eigen::Vector3f BB_3D = template_door_handle_BB_3D[num_cluster_template];

							//double BB_mean =  sqrt(BB_3D(0) * BB_3D(0) + BB_3D (1) * BB_3D (1) + BB_3D (2)* BB_3D (2));
							//std::cout<<"template_diag " << BB_mean<<std::endl;

							// Eigen::Matrix4f pca_assumed -> rotation and tranlation to origin
							// find rot btw PCA's coordinates systems
							//  template * R = assumed
							// R = assumed * template^T;

							Eigen::Matrix4f transform_hndl;
							transform_hndl.setIdentity();
							transform_hndl.block<3,3>(0,0) = pca_assumed.block<3,3>(0,0) * pca_template.block<3,3>(0,0).transpose();

							pcl::transformPointCloud (*template_cloud_pca,*template_cloud_pca, transform_hndl);

							icpInformation icp_data = featureObj.icpBasedTemplateAlignment(assumed_handle_cloud_pca,template_cloud_pca);
							double fitness_score = icp_data.icp_fitness_score;
							Eigen::Matrix4f icp_transformation = icp_data.icp_transformation;

							pcl::transformPointCloud (*template_cloud_pca,*template_cloud_pca, icp_transformation);

							std::vector<int> indices = featureObj.estimateCorrespondences(assumed_handle_cloud_pca, template_cloud_pca);

							int number_correspondences = indices.size();
							int number_points_cluster = assumed_handle_cloud_pca->points.size();
							double points_ratio = (double )number_correspondences / (double) number_points_cluster;

							// get transformation 
							double sum_squared_err =  sqrt(icp_transformation(0,3) * icp_transformation(0,3) + icp_transformation (1,3) * icp_transformation (1,3) + icp_transformation (2,3)* icp_transformation (2,3));
				
							// for each assumed cluster  create a vector of 
							// template 1 -- icp trafo
							// template 2 -- icp trafo

									if ((sum_squared_err < sum_squared_err_min) && (points_ratio > points_ratio_min))
									{
										sum_squared_err_min = sum_squared_err;
										points_ratio_min = points_ratio;
										template_cloud_pca_best = template_cloud_pca;
									}
						}	//end if cluster size > 0
				} // end for

			sum_squared_err_vec.push_back(sum_squared_err_min);
			template_cloud_pca_best_vec.push_back(template_cloud_pca_best);
		} // end for assumed handle cloud
		// find best cluster template match

	if (!sum_squared_err_vec.empty())
	{
		int pos = 0;
		double mic = sum_squared_err_vec[0];

		for (int k = 0; k < sum_squared_err_vec.size(); k++)
		{
			 if(sum_squared_err_vec[k] < mic)
			 {
				mic = sum_squared_err_vec[k];
				int pos = k;
			 }
		}
		// top template custer match based in prior translatoion error estimation
			assumed_handle_cloud_pca = assumed_handle_cloud_pca_best_vec[pos];
			template_cloud_pca = template_cloud_pca_best_vec[pos];

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_filt(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::PointXYZRGB pp_filt;

		if (template_cloud_pca->points.size() > 0)
		{
			std::vector<int> indices = featureObj.estimateCorrespondences(assumed_handle_cloud_pca,template_cloud_pca);

			for (int i = 0; i < indices.size(); i++)
				{		
				pp_filt.x=assumed_handle_cloud_pca->points[i].x; 
				pp_filt.y=assumed_handle_cloud_pca->points[i].y; 
				pp_filt.z=assumed_handle_cloud_pca->points[i].z; 
				pp_filt.r = 0;
				pp_filt.g = 255;
				pp_filt.b = 0;
				pc_filt->push_back(pp_filt);
				}			

			icpInformation icp_data_nest_fit = featureObj.icpBasedTemplateAlignment(pc_filt,template_cloud_pca);

			double fitness_score = icp_data_nest_fit.icp_fitness_score;
			Eigen::Matrix4f icp_transformation_best_fit = icp_data_nest_fit.icp_transformation;

			pcl::transformPointCloud (*template_cloud_pca,*template_cloud_pca, icp_transformation_best_fit);

			std::cout<<"Is Handle!"<< std::endl;
			*published_pc+= *assumed_handle_cloud_pca;
		} // end if check

	}
			*published_pc+= *template_cloud_pca;
			pcl::toROSMsg(*published_pc, *point_cloud_out_msg_);
			point_cloud_out_msg_->header.frame_id = "camera_link";
			pub_.publish(point_cloud_out_msg_);
}
// end void callback

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
		