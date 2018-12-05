#include "ipa_door_handle_segmentation.h"
#include "ipa_template_generation.h"
#include "ipa_door_handle_template_alignment.h"


DoorHandleTemplateGeneration::DoorHandleTemplateGeneration(std::string file_path_to_point_clouds )
{

	targetPathXYZRGB_  = "/home/rmb-ml/Desktop/PointCloudData/templateDataXYZRGB/";
	targetPathNormals_ = "/home/rmb-ml/Desktop/PointCloudData/templateDataNormals/";
	targetPathFeatures_ = "/home/rmb-ml/Desktop/PointCloudData/templateDataFeatures/";
	
	generateTemplatePCLFiles(file_path_to_point_clouds);
}

// OFFLINE PART -> TEMPLATE GENERATION	

void  DoorHandleTemplateGeneration::generateTemplatePCLFiles(std:: string file_path_to_point_clouds)
{

PointCloudSegmentation seg;
FeatureCloudGeneration feat;

pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr template_cloud_reduced (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::Normal>::Ptr  template_cloud_normals (new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<pcl::FPFHSignature33>::Ptr template_cloud_features (new pcl::PointCloud<pcl::FPFHSignature33>);


std::vector <pcl::PointIndices> door_handle_cluster;	
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> >template_cluster_vec;

FeatureCloudGeneration featureObj;

 DIR *pDIR;
        struct dirent *entry;


        if(pDIR=opendir(file_path_to_point_clouds.c_str()))
		{
                while(entry = readdir(pDIR)){

                        if( strcmp(entry->d_name,file_path_to_point_clouds.c_str()) != 0 && strcmp(entry->d_name, "..") != 0 )
                        std::cout << entry->d_name << "\n";
						
						
						//load PCD File and perform segmentation

							std::string filePathPCDRead = file_path_to_point_clouds + entry->d_name;
							std::string filePathPCDWriteXYZRGB = targetPathXYZRGB_ + entry->d_name;
							std::string filePathPCDWriteNormals = targetPathNormals_ + entry->d_name;
							std::string filePathPCDWriteFeatures = targetPathFeatures_ + entry->d_name;

							
							if (pcl::io::loadPCDFile<pcl::PointXYZ> (filePathPCDRead, *template_cloud) == -1) //* load the file
									{
										PCL_ERROR ("Couldn't read PCD file. \n");
									}

							planeInformation planeData = seg.detectPlaneInPointCloud(template_cloud);
							pcl::ModelCoefficients::Ptr plane_coeff = planeData.plane_coeff;
							pcl::PointIndices::Ptr plane_pc_indices = planeData.plane_point_cloud_indices;


							
							template_cloud_reduced=seg.minimizePointCloudToObject(template_cloud,plane_pc_indices,plane_coeff);


								if (template_cloud_reduced->size() > 0)
									{
										door_handle_cluster=seg.findClustersByRegionGrowing(template_cloud_reduced);
										// only one object suppose to be the handle
										if (door_handle_cluster.size()== 1)
										{
											template_cluster_vec= seg.generateAlignmentObject(door_handle_cluster,template_cloud_reduced,plane_coeff);
											// calculate xyzrgb point cloud
											*template_cloud_reduced = *template_cluster_vec[0];
											template_cloud_reduced->width = 1;
											template_cloud_reduced->height = template_cloud_reduced->points.size();


											// downsample point cloud for better performance 
											template_cloud_reduced=feat.downSamplePointCloud(template_cloud_reduced);
											
											// calculate normals based on template_cloud_reduced
											template_cloud_normals = featureObj.calculateSurfaceNormals(template_cloud_reduced);
											
											//calculate features based on template_cloud_reduced
											template_cloud_features = featureObj.calculate3DFeatures(template_cloud_reduced,template_cloud_normals);


										
										std::cout << "Writing XYZ..." << std::endl;
										pcl::io::savePCDFileASCII (filePathPCDWriteXYZRGB,*template_cloud_reduced);

										std::cout << "Writing Normals..." << std::endl;
										pcl::io::savePCDFileASCII (filePathPCDWriteNormals,*template_cloud_normals);

										std::cout << "Writing Features..." << std::endl;
										pcl::io::savePCDFileASCII (filePathPCDWriteFeatures,*template_cloud_features);

										}

									}
									else
									{
												std::cout << "Not sufficient points for normal estimation." << std::endl;
									}

                }
                closedir(pDIR);
        }


}


// =================================================0
int main(int argc, char **argv)
{		
	std::string file_path_to_point_clouds = "/home/rmb-ml/Desktop/PointCloudData/unprocessed/";
	DoorHandleTemplateGeneration DoorHandleTemplateGeneration(file_path_to_point_clouds);

	return 0;
}

