
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

	// correspondence estimation function
	max_dist_1_ = 0.01; //first crit
	max_dist_2_ = 0.005; // refinement after best template
	overlap_ratio_ =0.9; // overlp ration of template and cluster during corresp matching

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

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_pca (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr cluster_pca_normals (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr template_pca(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr template_pca_best(new pcl::PointCloud<pcl::PointXYZRGB>);

	//========SEGMENTATION==================
		
	// create segmentation object
	PointCloudSegmentation segObj;
	// segment point cloud and detect planes
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
	Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > cluster_vec = segObj.segmentPointCloud(pointcloud_in_pcl_format);

	planeInformation planeData = segObj.detectPlaneInPointCloud(pointcloud_in_pcl_format);
	pcl::ModelCoefficients::Ptr plane_coefficients = planeData.plane_coeff;

	//===========TEMPLATEALIGNMENT===========
		// load templates into pc vector

	FeatureCloudGeneration featureObj;

	std::vector<double> sum_squared_err_vec;
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
	Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > template_pca_best_vec,cluster_pca_best_vec;

	double sum_squared_err_min = 0.1;
	double points_ratio_min = 0.8;

	Eigen::Matrix4f final_transformation;

	// ==================================================ITERATION OVER CLUSTERS ===============================================

		for (int num_cluster = 0; num_cluster < cluster_vec.size (); ++num_cluster) // 
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster = cluster_vec[num_cluster];		

			// estimate preconditions
			cluster = featureObj.downSamplePointCloud(cluster);
			pcaInformation pcaData =  segObj.calculatePCA(cluster);

			Eigen::Matrix4f cluster_pca_trafo = pcaData.pca_transformation;
			Eigen::Vector3f bb_3D_cluster = pcaData.bounding_box_3D;
			
			double BB_Diag_3D =  sqrt(bb_3D_cluster(0) * bb_3D_cluster(0) + bb_3D_cluster (1) * bb_3D_cluster (1) + bb_3D_cluster (2)* bb_3D_cluster (2));

			std::cout<<BB_Diag_3D<<std::endl;

			// if diagonal to small to large:continue
			// else : decide in which subdirectory to go 

			// apply transformation on assumed handle cloud
			pcl::transformPointCloud (*cluster, *cluster_pca, cluster_pca_trafo);

			// check for cylinder ===================================================
			cluster_pca_normals =  featureObj.calculateSurfaceNormals(cluster_pca);
			bool isCylinder = segObj.alignCylinderToPointCloud(cluster_pca,cluster_pca_normals,plane_coefficients);
					
				if (!isCylinder)
				{
					continue;
				}

			// ====================CHANGE TO PCAs BOUNDING BOX CRIT ==================================

			cluster_pca_best_vec.push_back(cluster_pca);
			// determine size of the BB here to define in which template subfolder to go
				// double size ....

			std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
			Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > template_vec_xyz = featureObj.loadGeneratedTemplatePCLXYZ(filePathXYZRGB_);
												// 2. normals
			std::vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr,
			Eigen::aligned_allocator<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> > template_vec_features = featureObj.loadGeneratedTemplatePCLFeatures(filePathFeatures_);
														// 3. features
			std::vector<pcl::PointCloud<pcl::Normal>::Ptr,
			Eigen::aligned_allocator<pcl::PointCloud<pcl::Normal>::Ptr> >template_vec_normals = featureObj.loadGeneratedTemplatePCLNormals(filePathNormals_);
														// 4. PCA
			std::vector<Eigen::Matrix4f> template_pca_trafo_vec = featureObj.loadGeneratedPCATransformations(filePathPCATransformations_);
	
			std::vector<Eigen::Vector3f> template_BB_3D = featureObj.loadGeneratedBBInformation(filePathBBInformations_);


			// ================================== ITERATION OVER TEMPLATES =============================================================

			for (int num_template = 0; num_template < template_vec_xyz.size(); ++num_template) // template loop
				{
					if (cluster_vec[num_cluster]->points.size() > 0)
					   {	
							pcl::PointCloud<pcl::Normal>::Ptr template_normals = template_vec_normals[num_template];
							pcl::PointCloud<pcl::FPFHSignature33>::Ptr template_features = template_vec_features[num_template];
							// already tranformed appling PCA
							pcl::PointCloud<pcl::PointXYZRGB>::Ptr template_pca = template_vec_xyz[num_template];
							Eigen::Matrix4f pca_template =	 template_pca_trafo_vec[num_template]; // rotation and translatoion to origin
							Eigen::Vector3f BB_3D = template_BB_3D[num_template];

							double BB_mean =  sqrt(BB_3D(0) * BB_3D(0) + BB_3D (1) * BB_3D (1) + BB_3D (2)* BB_3D (2));
							std::cout<<"template_diag " << BB_mean<<std::endl;

							// Eigen::Matrix4f pca_assumed -> rotation and tranlation to origin
							// find rot btw PCA's coordinates systems
							//  template * R = assumed
							// R = assumed * template^T;


							Eigen::Matrix4f transform_hndl;
							transform_hndl.setIdentity();
							transform_hndl.block<3,3>(0,0) = cluster_pca_trafo.block<3,3>(0,0) * pca_template.block<3,3>(0,0).transpose();

							pcl::transformPointCloud (*template_pca,*template_pca, transform_hndl);

							icpInformation icp_data = featureObj.icpBasedTemplateAlignment(cluster_pca,template_pca);
							double fitness_score = icp_data.icp_fitness_score;
							Eigen::Matrix4f icp_transformation = icp_data.icp_transformation;

							pcl::transformPointCloud (*template_pca,*template_pca, icp_transformation);

							std::vector<int> indices = featureObj.estimateCorrespondences(cluster_pca, template_pca,max_dist_1_,overlap_ratio_);

							int number_correspondences = indices.size();
							int number_points_cluster = cluster_pca->points.size();
							double points_ratio = (double) number_correspondences /  (double) number_points_cluster;

							// get transformation 
							double sum_squared_err =  sqrt(icp_transformation(0,3) * icp_transformation(0,3) + icp_transformation (1,3) * icp_transformation (1,3) + icp_transformation (2,3)* icp_transformation (2,3));
				
							// for each assumed cluster  create a vector of 
							// template 1 -- icp trafo
							// template 2 -- icp trafo

								if ((sum_squared_err < sum_squared_err_min) && (points_ratio > points_ratio_min))
								{
									sum_squared_err_min = sum_squared_err;
									points_ratio_min = points_ratio;
									template_pca_best = template_pca;
								}
						}	//end if cluster size > 0
				} // end for

			sum_squared_err_vec.push_back(sum_squared_err_min);
			template_pca_best_vec.push_back(template_pca_best);
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
			cluster_pca = cluster_pca_best_vec[pos];
			template_pca = template_pca_best_vec[pos];

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_filt(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::PointXYZRGB pp_filt;

		if (template_pca->points.size() > 0)
		{
			std::vector<int> indices = featureObj.estimateCorrespondences(cluster_pca,template_pca,max_dist_2_,overlap_ratio_);

			for (int i = 0; i < indices.size(); i++)
				{		
				pp_filt.x=cluster_pca->points[i].x; 
				pp_filt.y=cluster_pca->points[i].y; 
				pp_filt.z=cluster_pca->points[i].z; 
				pp_filt.r = 0;
				pp_filt.g = 255;
				pp_filt.b = 0;
				pc_filt->push_back(pp_filt);
				}			

			icpInformation icp_data_nest_fit = featureObj.icpBasedTemplateAlignment(pc_filt,template_pca);

			double fitness_score = icp_data_nest_fit.icp_fitness_score;
			Eigen::Matrix4f icp_transformation_best_fit = icp_data_nest_fit.icp_transformation;

			pcl::transformPointCloud (*template_pca,*template_pca, icp_transformation_best_fit);

			// B  Distance --> paper

			std::cout<<"Is Handle!"<< std::endl;

			
			*published_pc+= *cluster_pca;
		} // end if check

	}
			*published_pc+= *template_pca;
			pcl::toROSMsg(*published_pc, *point_cloud_out_msg_);
			point_cloud_out_msg_->header.frame_id = "camera_link";
			pub_.publish(point_cloud_out_msg_);
}
// end void callback

void StartHandleDetection::initCameraNode(ros::NodeHandle nh, sensor_msgs::PointCloud2::Ptr point_cloud_out_msg)
{
	std::cout << "Initialising StartHandleDetection Constructor." << std::endl;
	pub_ = nh_.advertise<sensor_msgs::PointCloud2>(TOPIC_POINT_CLOUD_OUT,1);
	(pub_) ? std::cout << "Pub is valid." << std::endl : std::cout << "Pub is not valid." << std::endl;
	ros::Subscriber point_cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(TOPIC_POINT_CLOUD_IN, 1, &StartHandleDetection::pointcloudCallback, this);
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
		