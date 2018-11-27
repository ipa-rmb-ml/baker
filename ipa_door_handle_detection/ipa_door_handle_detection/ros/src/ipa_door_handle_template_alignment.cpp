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




void FeatureCloudGeneration::templateAlignmentByICP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_point_cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_template_point_cloud)
{
	//This creates an instance of an IterativeClosestPoint
	// “icp.setInputSource(input_point_cloud);” sets input_point_cloud as the PointCloud to begin from 
	//and “icp.setInputTarget(input_template_point_cloud);” sets input_template_point_cloud as the PointCloud which we want cloud_in to look like.

  pcl::PointCloud<pcl::Normal>::Ptr input_point_cloud_normal (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Normal>::Ptr input_template_point_cloud_normal (new pcl::PointCloud<pcl::Normal>);


  input_point_cloud_normal = estimateNormalsFromPointCloud(input_point_cloud);

  input_template_point_cloud_normal = estimateNormalsFromPointCloud(input_template_point_cloud);


}




pcl::PointCloud<pcl::Normal>::Ptr FeatureCloudGeneration::estimateNormalsFromPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_point_cloud)
{
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normals;
  normals.setInputCloud (input_point_cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  normals.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr input_point_cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  normals.setRadiusSearch (0.1);

  // Compute the features
  normals.compute (*input_point_cloud_normals);

  

  return input_point_cloud_normals;


}