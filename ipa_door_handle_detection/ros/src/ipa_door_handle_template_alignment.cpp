#include "ipa_door_handle_template_alignment.h"
#include "ipa_template_generation.h"



FeatureCloudGeneration::FeatureCloudGeneration()
{

alignment_eps_ = 1e-6;
alignment_thres_ = 1e-4;

 max_num_iter_icp_ref_ = 2;
 corresp_dist_step_ = 0.01;
 max_num_iter_ = 1000;
 similarity_thres_ = 0.9f;

 rad_search_dist_ = 0.03;
 voxel_grid_size_ = 0.001f;
}


templateInformation FeatureCloudGeneration::loadGeneratedTemplatePCLXYZ(std::string filePath, std::string file_name)
{


std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > doorhandle_template_vec;

std::vector<std::string> handle_type_name_vec;

// list all folder in filePath directory
// each folder represents one door handle type
// LOOP over folders to load handle pcd file with the given filename


 DIR *pDIR;
        struct dirent *entry;
        if(pDIR=opendir(filePath.c_str()))
		{
              // obtaining folders in directory
                while(entry = readdir(pDIR))
                {
                        if( strcmp(entry->d_name,filePath.c_str()) != 0 && strcmp(entry->d_name, "..") != 0 &&  strcmp(entry->d_name, ".") != 0)
							          {
                              //load PCD File and perform segmentation
                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr template_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
                            std::string filePathTemplatePCD =  filePath + entry->d_name + "/door_handle_type_" + entry->d_name + file_name;                       
 
                          if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (filePathTemplatePCD, *template_cloud) == -1) 
                            {
                              PCL_ERROR ("Couldn't read PCD file. \n");
                            }

                          doorhandle_template_vec.push_back(template_cloud);
                          handle_type_name_vec.push_back(entry->d_name);

                          
                        } 
                }
                closedir(pDIR);
        }

    templateInformation handle_template_information;   

    handle_template_information.doorhandle_template_vec = doorhandle_template_vec;
    handle_template_information.handle_type_name_vec = handle_type_name_vec;

		return handle_template_information;
}


std::vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> > FeatureCloudGeneration::loadGeneratedTemplatePCLFeatures(std::string filePath,std::string file_name)
{

pcl::PointCloud<pcl::FPFHSignature33>::Ptr template_cloud (new pcl::PointCloud<pcl::FPFHSignature33>);
std::vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> > doorhandle_template_vec;

 DIR *pDIR;
        struct dirent *entry;
        if(pDIR=opendir(filePath.c_str()))
		{
                while(entry = readdir(pDIR)){
                        if( strcmp(entry->d_name,filePath.c_str()) != 0 && strcmp(entry->d_name, "..") != 0 &&  strcmp(entry->d_name, ".") != 0)
							{

							//load PCD File and perform segmentation
								std::string filePathTemplatePCD =  filePath + entry->d_name;

								if (pcl::io::loadPCDFile<pcl::FPFHSignature33> (filePathTemplatePCD, *template_cloud) == -1) 
                {
										PCL_ERROR ("Couldn't read PCD file. \n");
                }//* load the file

										doorhandle_template_vec.push_back(template_cloud);
							}
                }
                closedir(pDIR);
        }
      
		return doorhandle_template_vec;
}

std::vector<pcl::PointCloud<pcl::Normal>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<pcl::Normal>::Ptr> > FeatureCloudGeneration::loadGeneratedTemplatePCLNormals(std::string filePath)
{

pcl::PointCloud<pcl::Normal>::Ptr template_cloud (new pcl::PointCloud<pcl::Normal>);
std::vector<pcl::PointCloud<pcl::Normal>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<pcl::Normal>::Ptr> > doorhandle_template_vec;

 DIR *pDIR;
        struct dirent *entry;
        if(pDIR=opendir(filePath.c_str()))
		{
                while(entry = readdir(pDIR)){
                 if( strcmp(entry->d_name,filePath.c_str()) != 0 && strcmp(entry->d_name, "..") != 0 &&  strcmp(entry->d_name, ".") != 0)
							    {
							     //load PCD File and perform segmentation
								    std::string filePathTemplatePCD =  filePath + entry->d_name;

                    if (pcl::io::loadPCDFile<pcl::Normal> (filePathTemplatePCD, *template_cloud) == -1)
                    {
                        PCL_ERROR ("Couldn't read PCD file. \n");
                    } //* load the file

                        doorhandle_template_vec.push_back(template_cloud);
						    	}
                }
                closedir(pDIR);
        }
        
		return doorhandle_template_vec;

}


std::vector<Eigen::Matrix4f> FeatureCloudGeneration::loadGeneratedPCATransformations(std::string filePath,std::string file_name)
{
  std::vector<Eigen::Matrix4f> pca_transformation_vec;
  DIR *pDIR;

  struct dirent *entry;

    if(pDIR=opendir(filePath.c_str()))
		{
      while(entry = readdir(pDIR)){

          if( strcmp(entry->d_name,filePath.c_str()) != 0 && strcmp(entry->d_name, "..") != 0 &&  strcmp(entry->d_name, ".") != 0)
							{

                // remove file extension and and .txt
                boost::filesystem::path pathObj(file_name);
			          std::string file_name =  pathObj.stem().string();

							//load PCD File and perform segmentation
								std::string txtFile =  filePath + entry->d_name + "/door_handle_type_" + entry->d_name + file_name + ".txt";

                std::ifstream indata;
                indata.open(txtFile.c_str());

                std::vector <double> mat_element_vec;
                double mat_element;
                int rows = 0;
                int cols = 0;
                int counter = 0;

            while(!indata.eof())
            {
              std::string line = "";
              std::getline(indata,line);

              if (!line.empty())
              {
                mat_element_vec.push_back(std::atof(line.c_str()));
              }
						} // end if

              Eigen::Matrix4f trafo_pca(4,4);
            // fill trafo_pca matrix with data from the txt file
              for (int row = 0; row < 4; row++)
              {
                for (int col = 0; col < 4; col ++)
                {
                    trafo_pca(row,col) = mat_element_vec.at(counter);
                    counter += 1;
                }
              }
                // push traf_pca data into final vector
                pca_transformation_vec.push_back(trafo_pca);

						} // end if
       }// end while
            closedir(pDIR);
    }

		return pca_transformation_vec;
}



std::vector<Eigen::Vector3f> FeatureCloudGeneration::loadGeneratedBBInformation(std::string filePath,std::string file_name)
{
  std::vector<Eigen::Vector3f> BB_3D_vec;
  DIR *pDIR;

  struct dirent *entry;

    if(pDIR=opendir(filePath.c_str()))
		{
      while(entry = readdir(pDIR)){

          if( strcmp(entry->d_name,filePath.c_str()) != 0 && strcmp(entry->d_name, "..") != 0 &&  strcmp(entry->d_name, ".") != 0)
						{


                // remove file extension and and .txt
                boost::filesystem::path pathObj(file_name);
			          std::string file_name =  pathObj.stem().string();

								std::string txtFile =  filePath + entry->d_name + "/door_handle_type_" + entry->d_name + file_name + ".txt";

                std::ifstream indata;
                indata.open(txtFile.c_str());
                std::string line;

                Eigen::Vector3f BB_3D(3,1);
                std::vector <double> mat_element_vec;
                double mat_element;
                Eigen::Vector3f vec(3);

            // read out lines
           

            while(!indata.eof())
            {

              std::string line = "";
              std::getline(indata,line);

              if (!line.empty())
              {
                mat_element_vec.push_back(std::atof(line.c_str()));
              }
						} // end if

            for (int i = 0; i < mat_element_vec.size(); i++)
            {
                vec(i) =   mat_element_vec.at(i);

            }
            BB_3D_vec.push_back(vec);
          }
       }// end while
       closedir(pDIR);
    }


		return BB_3D_vec;
}


icpInformation FeatureCloudGeneration::icpBasedTemplateAlignment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_point_cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_template_point_cloud)
{

  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

 //Set the maximum distance between two correspondences (src<->tgt) to 10cm
  icp.setMaxCorrespondenceDistance (0.0001);
  icp.setTransformationEpsilon (alignment_eps_);

  pcl::PointCloud<pcl::PointXYZRGB> Final;
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;

 for (int i = 0; i < max_num_iter_icp_ref_; ++i)
  {
    // Estimate
    icp.setInputSource (input_template_point_cloud);
    icp.setInputTarget(input_point_cloud);
    icp.align (Final);

		//accumulate transformation between each Iteration
    Ti = icp.getFinalTransformation () * Ti;
    prev = icp.getLastIncrementalTransformation ();


		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
    if (fabs ((icp.getLastIncrementalTransformation () - prev).sum ()) < icp.getTransformationEpsilon ())
    {
      icp.setMaxCorrespondenceDistance (icp.getMaxCorrespondenceDistance () - corresp_dist_step_);
    }
    
    prev = icp.getLastIncrementalTransformation ();

    if (icp.getFitnessScore() < alignment_thres_ )
    {
      break;
    }
  }

  // DECLARE STRUCT
  icpInformation icp_data;
	icp_data.icp_transformation = icp.getFinalTransformation ();
 	icp_data.icp_fitness_score = icp.getFitnessScore();
  
  //
     // std::cout << "================ICP================ " << std::endl;
      //std::cout << "Fitness Score ICP: "<< score << std::endl; 
     // printf ("\n");
     // printf ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
     // printf ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
      //printf ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
    //  printf ("\n");
    //  printf ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation(2,3));
    //  printf ("\n");
   return icp_data;
}


Eigen::Matrix4f FeatureCloudGeneration::featureBasedTemplateAlignment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_point_cloud,
pcl::PointCloud<pcl::Normal>::Ptr input_cloud_normals,pcl::PointCloud<pcl::PointXYZRGB>::Ptr template_cloud,
pcl::PointCloud<pcl::FPFHSignature33>::Ptr template_cloud_features,
pcl::PointCloud<pcl::Normal>::Ptr template_cloud_normals)
{

  Eigen::Matrix4f transformation = transformation.setZero(4,4);

  return transformation;
    
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
      ne.setRadiusSearch (rad_search_dist_);

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
      fpfh_est.setRadiusSearch (rad_search_dist_);
      fpfh_est.compute (*fpfhFeatures);

 return fpfhFeatures;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr FeatureCloudGeneration::downSamplePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_point_cloud)
{

        // ... and downsampling the point cloud
  pcl::VoxelGrid<pcl::PointXYZRGB> vox_grid;
  vox_grid.setInputCloud (input_point_cloud);
  vox_grid.setLeafSize (voxel_grid_size_, voxel_grid_size_, voxel_grid_size_);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZRGB>); 
  vox_grid.filter (*tempCloud);
  input_point_cloud = tempCloud; 

return input_point_cloud;

}



double FeatureCloudGeneration::estimateCorrespondences(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_pc, pcl::PointCloud<pcl::PointXYZRGB>::Ptr template_pc, double max_dist,double overlap_ratio)
{
    boost::shared_ptr<pcl::Correspondences> cor_all_ptr (new pcl::Correspondences),
																						cor_remaining_ptr (new pcl::Correspondences),
																						cor_remaining_ptr_nrm (new pcl::Correspondences);

		// correspondence rejection for ICP
		/// determine all corresp
		pcl::registration::CorrespondenceEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB> corEst;
		corEst.setInputSource (template_pc);
		corEst.setInputTarget (cluster_pc); 
		corEst.determineCorrespondences (*cor_all_ptr);
														
		//Correspondence Rejection methods
		// trimmed: use only k% best correspondances
		boost::shared_ptr<pcl::Correspondences> corr_res_trimmed (new pcl::Correspondences);
		pcl::registration::CorrespondenceRejectorTrimmed corr_rej_trimmed;
		corr_rej_trimmed.setOverlapRatio(overlap_ratio);
		corr_rej_trimmed.setInputCorrespondences(cor_all_ptr);
		corr_rej_trimmed.getCorrespondences(*corr_res_trimmed);

		// distance threshold: 
		pcl::registration::CorrespondenceRejectorDistance rejector;
		rejector.setInputCorrespondences (corr_res_trimmed);
		rejector.setMaximumDistance (max_dist);
		rejector.getRemainingCorrespondences(*corr_res_trimmed,*cor_remaining_ptr); 

		std::vector<int> indices;
		pcl::ExtractIndices<pcl::PointXYZRGB> ex;
		pcl::registration::getQueryIndices (*cor_remaining_ptr,indices);

    int number_correspondences = indices.size();
    int number_points_template = template_pc->points.size();

		double points_ratio = (double) number_correspondences /  (double) number_points_template;

  return points_ratio;

}


std::string FeatureCloudGeneration::getFilePathFromParameter(int dist, int angle_XZ, int angle_YZ)
{

      int num_dist = 2;
			int distances[] = {55-dist,70-dist};

			int num_angle_xz = 3;
			int angle_xz[] = {-20-angle_XZ,0-angle_XZ,20-angle_XZ}; //deg

			int num_angle_yz = 3;
			int angle_yz[] = {-25-angle_YZ,0-angle_YZ,25-angle_YZ}; // deg

		// setup vec contain this info: angle_xz, angle_yz, distance to door plane

			//distance loop
 			int index_d = 0 ;
			int n_d = abs(distances[0]);
				for (int i = 1; i < num_dist; i++)
				{
					if (distances[i] < n_d)
					{
						n_d = abs(distances[i]); 
						index_d = i ;
					}
				}
			int final_distance = distances[index_d]+dist;

			int index_a1 = 0 ;
			int n_a1 = abs(angle_xz[0]);
			for (int i = 1; i < num_angle_xz; i++)
			{
				if (angle_xz[i] < n_a1)
				{
					n_a1 = abs(angle_xz[i]); 
					index_a1 = i ;
				}
			}
			int final_angleXZ = angle_xz[index_a1]+angle_XZ;

			int index_a2 = 0 ;
			int n_a2 = abs(angle_yz[0]);
			for (int i = 1; i < num_angle_yz; i++)
			{
				if (angle_yz[i] < n_a2)
				{
					n_a2 = abs(angle_yz[i]); 
					index_a2 = i ;
				}
			}
			int final_angleYZ = angle_yz[index_a2]+angle_YZ;

		std::stringstream str1, str2, str3;

		str1 << final_angleXZ;
		str2 << final_angleYZ;
		str3 << final_distance;

		std::string angle_1_str = str1.str();
		std::string angle_2_str = str2.str();
		std::string dist_str = str3.str();

		std::string name_pcd  = "_distance_" + dist_str + "cm_" + "angleXZ_" + angle_1_str + "°_" + "angleYZ_"+ angle_2_str + "°_ROI.pcd";

		return name_pcd;
}








