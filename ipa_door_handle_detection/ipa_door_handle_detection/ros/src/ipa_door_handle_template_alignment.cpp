#include "ipa_door_handle_template_alignment.h"

FeatureCloudGeneration::FeatureCloudGeneration()
{
}


std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > FeatureCloudGeneration::loadGeneratedTemplatePCLFiles(const std::string filePath)
{

pcl::PointCloud<pcl::PointXYZRGB>::Ptr template_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > doorhandle_template_vec;

 DIR *pDIR;
        struct dirent *entry;
        if(pDIR=opendir(filePath.c_str()))
		{
                while(entry = readdir(pDIR)){
                        if( strcmp(entry->d_name,filePath.c_str()) != 0 && strcmp(entry->d_name, "..") != 0 &&  strcmp(entry->d_name, ".") != 0)
							{

							//load PCD File and perform segmentation
								std::string filePathTemplatePCD =  filePath + entry->d_name;

								if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (filePathTemplatePCD, *template_cloud) == -1) //* load the file
										PCL_ERROR ("Couldn't read PCD file. \n");
										doorhandle_template_vec.push_back(template_cloud);
							}
                }
                closedir(pDIR);
        }
		return doorhandle_template_vec;

}




void FeatureCloudGeneration::icpBasedTemplateAlignment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_point_cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_template_point_cloud)
{


  double transformEps = 1e-5;

  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  icp.setInputSource(input_point_cloud);
  icp.setInputTarget(input_template_point_cloud);
  icp.setTransformationEpsilon (transformEps);
  pcl::PointCloud<pcl::PointXYZRGB> Final;
  icp.align(Final);


  double score = icp.getFitnessScore();

  if (score < transformEps )
  {
      std::cout << "Has converged:"  << " score: " << score << std::endl;
      Eigen::Matrix4f transformation = icp.getFinalTransformation ();

      printf ("\n");
      printf ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
      printf ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
      printf ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
      printf ("\n");
      printf ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation(2,3));

  }else
  {
       std::cout << "Has not converged" << " score: " << score << std::endl;
  }
  
}



void FeatureCloudGeneration::featureBasedTemplateAlignment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_point_cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_template_point_cloud)
{
   
  	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_aligned(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointCloud<pcl::Normal>::Ptr  input_point_cloud_normals;
    pcl::PointCloud<pcl::Normal>::Ptr  input_template_point_cloud_normals;

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr input_point_cloud_features;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr input_template_point_cloud_features;

    // calculate surface normals for pointcloud and template
    input_point_cloud_normals =  calculateSurfaceNormals(input_point_cloud);
    input_template_point_cloud_normals = calculateSurfaceNormals(input_template_point_cloud);

   //calculate features for point clpud and template based oon xyzrgb and priorly estimated surface normals
    input_point_cloud_features = calculate3DFeatures(input_point_cloud,input_point_cloud_normals);
    input_template_point_cloud_features = calculate3DFeatures(input_template_point_cloud,input_template_point_cloud_normals);


    pcl::SampleConsensusPrerejective<pcl::PointXYZRGB, pcl:: PointXYZRGB, pcl::FPFHSignature33> align;
    align.setInputSource (input_point_cloud);
    align.setSourceFeatures (input_point_cloud_features);

    align.setInputTarget (input_template_point_cloud);
    align.setTargetFeatures (input_template_point_cloud_features);

    align.setMaximumIterations (30000); // Number of RANSAC iterations
    align.setNumberOfSamples (5); // Number of points to sample for generating/prerejecting a pose
    align.setCorrespondenceRandomness (5); // Number of nearest features to use
    align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
    align.setMaxCorrespondenceDistance (2.5f * 0.005f); // Inlier threshold
    align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis

      //pcl::ScopeTime t("Alignment");
    align.align (*cloud_aligned);
    std::cout << align.getFitnessScore() <<  std::endl;
    
    std::cout << align.getFinalTransformation() <<  std::endl;
    

  //  if (align.hasConverged ())
  {
    // Print results
    printf ("\n");
    Eigen::Matrix4f transformation = align.getFinalTransformation ();
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
    pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), cloud_aligned->size ());
  }

  

}


pcl::PointCloud<pcl::Normal>::Ptr  FeatureCloudGeneration::calculateSurfaceNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_point_cloud)
{

      // Create the normal estimation class, and pass the input dataset to it
      pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
      ne.setInputCloud (input_point_cloud);

      // Create an empty kdtree representation, and pass it to the normal estimation object.
      // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
      pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
      ne.setSearchMethod (tree);

      // Output datasets
      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

      // Use all neighbors in a sphere of radius 3cm
      ne.setRadiusSearch (0.03);

      // Compute the features
      ne.compute (*cloud_normals);
      // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
      return cloud_normals;
  }

 pcl::PointCloud<pcl::FPFHSignature33>::Ptr FeatureCloudGeneration::calculate3DFeatures(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_point_cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{

      pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
      fpfh_est.setInputCloud (input_point_cloud);
      fpfh_est.setInputNormals (cloud_normals);

      pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
      pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhFeatures(new pcl::PointCloud<pcl::FPFHSignature33>);

      fpfh_est.setSearchMethod (tree);
      fpfh_est.setRadiusSearch (0.03);
      fpfh_est.compute (*fpfhFeatures);

      return fpfhFeatures;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr FeatureCloudGeneration::downSamplePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_point_cloud)
{

        // ... and downsampling the point cloud
      const float voxel_grid_size = 0.005f;
      pcl::VoxelGrid<pcl::PointXYZRGB> vox_grid;
      vox_grid.setInputCloud (input_point_cloud);
      vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZRGB>); 
      vox_grid.filter (*tempCloud);
      input_point_cloud = tempCloud; 

      return input_point_cloud;

}













