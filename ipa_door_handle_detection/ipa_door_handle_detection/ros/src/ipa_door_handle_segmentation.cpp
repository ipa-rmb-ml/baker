
#include "ipa_door_handle_segmentation.h"


PointCloudSegmentation::PointCloudSegmentation()
{

}

// main segmentation process including coloring the pointcloud and the plane detection
// ================================================================================================================================
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> >  PointCloudSegmentation::segmentPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr reduced_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr published_pc(new pcl::PointCloud<pcl::PointXYZRGB>);


	pcl::ModelCoefficients::Ptr plane_coeff;
	std::vector <pcl::PointIndices> clusters;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> >clusters_vec_pc;

	// PLANE DETECTION 
	planeInformation planeData = detectPlaneInPointCloud(input_cloud);

	plane_coeff = planeData.plane_coeff;
	plane_pc = planeData.plane_point_cloud;

	// coloredPC: PC colored in blue
	// plane_pc: detected plane ccolored in red
	// plane coeff: coeffs of the detected plane

	reduced_pc=minimizePointCloudToObject(input_cloud,plane_pc,plane_coeff);

	clusters=findClustersByRegionGrowing(reduced_pc);
    clusters_vec_pc = generateAlignmentObject(clusters,reduced_pc,plane_coeff);


	// concentrate plane_pc with cluster_pc for visualizatuion
 return clusters_vec_pc;
}	
//=====================================================================================================================================


pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudSegmentation::changePointCloudColor(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
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



 	planeInformation PointCloudSegmentation::detectPlaneInPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud){
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
};


pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudSegmentation::minimizePointCloudToObject(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_pc,pcl::ModelCoefficients::Ptr plane_coeff){

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr reduced_pc(new pcl::PointCloud<pcl::PointXYZRGB>);

	// calculate point to plane distance
	pcl::PointXYZRGB pp_PC;
	double min_dist = 0.05;
	double max_dist = 0.15;


	// height of door handle: due to DIN 85 cm to 105 cm
	double min_height = 0.85;
	double max_height = 1.05;

  	for (size_t i = 0; i < input_cloud->points.size (); ++i)
	{
		pp_PC.x = input_cloud->points[i].x;
		pp_PC.y = input_cloud->points[i].y;
		pp_PC.z = input_cloud->points[i].z;

			// storing and coloring data from only outside the plane
		
		double point2plane_distance =  pcl::pointToPlaneDistance (pp_PC, plane_coeff->values[0], plane_coeff->values[1], plane_coeff->values[2], plane_coeff->values[3]);


		// check if assumed door handle in height range
		if ((pp_PC.z > min_height) && (pp_PC.z < max_height) )
		{
			if ((point2plane_distance > min_dist) && (point2plane_distance < max_dist))
			{
						pp_PC.r = 0;																								
						pp_PC.b = 255;
						pp_PC.g =0;								
			
						reduced_pc->points.push_back(pp_PC);				
				}

		}
	}
	return reduced_pc;
}


std::vector <pcl::PointIndices> PointCloudSegmentation::findClustersByRegionGrowing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr reduced_pc)
{
  pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
  
  // computing normals
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;	
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (reduced_pc);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);


  pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
  reg.setMinClusterSize (500);
  reg.setMaxClusterSize (1000000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (reduced_pc);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);

  // most important: 
  // sets angle that will be used as the allowable range for the normals deviation
  reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
  // curvature threshold if two points have a small normals deviation then the disparity btw their curvatures is tested
  reg.setCurvatureThreshold (1.0);
  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

 // put into new function --> visualizing/counting clusters 

  return clusters;
}

	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
	Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > PointCloudSegmentation::generateAlignmentObject(std::vector <pcl::PointIndices> clusters,pcl::PointCloud<pcl::PointXYZRGB>::Ptr reduced_pc, pcl::ModelCoefficients::Ptr plane_coeff)
{

    pcl::PointXYZRGB clusteredPP;
   //write each cluster to as point cloud
   //vector to store clusters
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
	Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> >clusterVec_pc;

	// avoid crashing
	if(clusters.size() > 0)
	{
	// iterate over cluster
		for (int numCluster =0; numCluster < clusters.size(); numCluster=numCluster +1)
		{
			// randomize cluster color in pc for visualization

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_pc(new pcl::PointCloud<pcl::PointXYZRGB>);

				// iterate over points in cluster to color them in diferent colors
				for (size_t i = 0; i < clusters[numCluster].indices.size (); ++i)
				{
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_pt_proj_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
					pcl::PointXYZRGB clusteredPP_proj;

					clusteredPP.x=reduced_pc->points[clusters[numCluster].indices[i]].x;
					clusteredPP.y=reduced_pc->points[clusters[numCluster].indices[i]].y;
					clusteredPP.z=reduced_pc->points[clusters[numCluster].indices[i]].z; 

					clusteredPP.r =0;
					clusteredPP.g =0;
					clusteredPP.b =255;

					// adding single points to point cloud cluster, these are the object point lying outsidee the plane 
					cluster_pc->points.push_back(clusteredPP);

					// removing point outside the cluster projection on the plane
					cluster_pt_proj_pc=removePlaneOutlierByClusterOnPlaneProjection(clusteredPP,plane_coeff);

					clusteredPP_proj.x = cluster_pt_proj_pc->points[0].x;
					clusteredPP_proj.y = cluster_pt_proj_pc->points[0].y;	
					clusteredPP_proj.z = cluster_pt_proj_pc->points[0].z;
					clusteredPP_proj.r = 255;
					clusteredPP_proj.g = 0;
					clusteredPP_proj.b = 0;	
					cluster_pc->points.push_back(clusteredPP_proj);
				} 

				clusterVec_pc.push_back(cluster_pc);

		} // end clusters

		std::cout << "Number of clusters: " << clusters.size () << std::endl;
		return clusterVec_pc;

	}; //end if
    
	std::cout << "No cluster found"<< std::endl;
	return clusterVec_pc; 
};


pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudSegmentation::removePlaneOutlierByClusterOnPlaneProjection(pcl::PointXYZRGB clusterPoint,pcl::ModelCoefficients::Ptr plane_coeff)
{
   pcl::PointXYZRGB clusterPoint_proj;
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cache_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterProj(new pcl::PointCloud<pcl::PointXYZRGB>);

   cache_pc->points.push_back(clusterPoint);

	// plane coefficients: plane_coeff
	// creating ProjectInliners object --> using plane_coeffs to describe projection plane geometrically
	pcl::ProjectInliers<pcl::PointXYZRGB> proj;
	proj.setModelType (pcl::SACMODEL_PLANE);
	proj.setInputCloud (cache_pc);
	proj.setModelCoefficients (plane_coeff);
	proj.filter (*clusterProj);

	return clusterProj;

};
