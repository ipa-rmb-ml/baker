#include "ipa_door_handle_template_alignment.h"

PointCloudTemplateDatabase::PointCloudTemplateDatabase()
{
}




std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > PointCloudTemplateDatabase::loadGeneratedTemplatePCLFiles(const std::string filePath)
{

pcl::PointCloud<pcl::PointXYZRGB>::Ptr template_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > doorhandle_template_vec;
std::string filePathTemplate = "/home/rmb-ml/Desktop/PointCloudData/templateData/";

 DIR *pDIR;
        struct dirent *entry;

        if(pDIR=opendir(filePathTemplate.c_str()))
		{
			std::cout << "Start: Init Template Database" << std::endl;

                while(entry = readdir(pDIR)){

                        if( strcmp(entry->d_name,filePathTemplate.c_str()) != 0 && strcmp(entry->d_name, "..") != 0 )

						//load PCD File and perform segmentation

							std::string filePathTemplatePCD = filePathTemplate + entry->d_name;

								std::cout << filePathTemplate << std::endl;

								std::cout << entry->d_name << std::endl;

							//if (pcl::io::loadPCDFile<pcl::PointXYZ> (filePathTemplatePCD, *template_cloud) == -1) //* load the file
								{
								//	PCL_ERROR ("Couldn't read PCD file. \n");
								}


								//	doorhandle_template_vec.push_back(template_cloud);

                }
                closedir(pDIR);
        }


		std::cout << "End: Init Template Database" << std::endl;

		return doorhandle_template_vec;

}
