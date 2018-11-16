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
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr published_pc(new pcl::PointCloud<pcl::PointXYZRGB>);

	//transform imported pointcloud point_cloud_msg to pointcloud_in_pcl_format
	pcl::fromROSMsg(*point_cloud_msg, *pointcloud_in_pcl_format);

	// segment point cloud and detect planes
	published_pc = segmentPointCloud(pointcloud_in_pcl_format);


	// publish changed point cloud
	pcl::toROSMsg(*published_pc, *point_cloud_out_msg_);
	point_cloud_out_msg_->header.frame_id = "camera_link";
	pub_.publish(point_cloud_out_msg_);

}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudImport::segmentPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_pc(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::ModelCoefficients::Ptr plane_coeff;

	// PLANE DETECTION 
	planeInformation planeData = detectPlaneInPointCloud(input_cloud);


	plane_coeff = planeData.plane_coeff;
	plane_pc = planeData.plane_point_cloud;

	// coloredPC: PC colored in blue
	// plane_pc: detected plane ccolored in red
	// plane coeff: coeffs of the detected plane
	segmented_pc=minimizePointCloudToObject(input_cloud,plane_pc,plane_coeff);
	clustered_pc=findClustersByRegionGrowing(segmented_pc);

 return segmented_pc;
}	



pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudImport::changePointCloudColor(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
	 //std::cout << ">>> Starting PC color change." << std::endl;
	uint8_t r = 0, g = 0, b = r;

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


 	planeInformation PointCloudImport::detectPlaneInPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	pcl::ModelCoefficients::Ptr plane_coeff (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);
	seg.setInputCloud (input_cloud);
	// get inliers and plane coeff
	seg.segment (*inliers, *plane_coeff);


	// iterater over inlier (pllane points) to color them red 
	pcl::PointXYZRGB pclPoint_plane;

	for (size_t i = 0; i < inliers->indices.size (); ++i)
	{
		pclPoint_plane.x = input_cloud->points[inliers->indices[i]].x;
		pclPoint_plane.y = input_cloud->points[inliers->indices[i]].y;
		pclPoint_plane.z = input_cloud->points[inliers->indices[i]].z;
		pclPoint_plane.r = 255;
		pclPoint_plane.b = 0;
		pclPoint_plane.g = 0;

		plane_pc->points.push_back(pclPoint_plane);
	}

	planeInformation planeData;
	planeData.plane_point_cloud = plane_pc;
 	planeData.plane_coeff = plane_coeff;


	return planeData;

}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudImport::minimizePointCloudToObject(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_pc,pcl::ModelCoefficients::Ptr plane_coeff){

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_pc(new pcl::PointCloud<pcl::PointXYZRGB>);

	// calculate point to plane distance
	pcl::PointXYZRGB pp_PC;
	double min_dist = 0.05;
	double max_dist = 0.15;

	segmented_pc= plane_pc;

  	for (size_t i = 0; i < input_cloud->points.size (); ++i)
	{
		pp_PC.x = input_cloud->points[i].x;
		pp_PC.y = input_cloud->points[i].y;
		pp_PC.z = input_cloud->points[i].z;

		double p2p_distance =  pcl::pointToPlaneDistance (pp_PC, plane_coeff->values[0], plane_coeff->values[1], plane_coeff->values[2], plane_coeff->values[3]);
	if (p2p_distance > min_dist){
			if  (p2p_distance < max_dist){

				pp_PC.r = 0;																								
				pp_PC.b = 255;
				pp_PC.g =0;								
	
				segmented_pc->points.push_back(pp_PC);
			}
		}
	}
	return segmented_pc;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudImport::findClustersByRegionGrowing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_pc(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);

  // computing normals
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;	
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (input_cloud);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (input_cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);


  pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
  reg.setMinClusterSize (5000);
  reg.setMaxClusterSize (10000);


  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (input_cloud);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);

  reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (1.0);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);


 // put into new function --> visualizing/counting clusters 
  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;


	return clustered_pc;
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

