#include <iostream>
#include <ros/ros.h>
#include <utility>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/common/projection_matrix.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#include <iostream>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/sample_consensus_prerejective.h>

#include <boost/thread/mutex.hpp>
#include <pcl/kdtree/kdtree_flann.h>

// based on PointCloudDataImport class 
// generating the template databasa for various door handle types 
class FeatureCloudGeneration
{
public:

FeatureCloudGeneration();

// file load different types
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> >  loadGeneratedTemplatePCLXYZ(std::string filePath);
std::vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> >  loadGeneratedTemplatePCLFeatures(std::string filePath);
std::vector<pcl::PointCloud<pcl::Normal>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<pcl::Normal>::Ptr> >  loadGeneratedTemplatePCLNormals(std::string filePath);

bool icpBasedTemplateAlignment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_point_cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_template_point_cloud);

bool featureBasedTemplateAlignment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_point_cloud,pcl::PointCloud<pcl::Normal>::Ptr input_cloud_normals,pcl::PointCloud<pcl::PointXYZRGB>::Ptr template_cloud,pcl::PointCloud<pcl::FPFHSignature33>::Ptr template_cloud_features,pcl::PointCloud<pcl::Normal>::Ptr template_cloud_normals);

pcl::PointCloud<pcl::Normal>::Ptr calculateSurfaceNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_point_cloud);

pcl::PointCloud<pcl::FPFHSignature33>::Ptr calculate3DFeatures(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_point_cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr downSamplePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_point_cloud);



private:

double alignment_eps_; //
double alignment_thres_;
double max_num_iter_; //1000
double similarity_thres_; //0.9f

double rad_search_dist_; //0.03
float voxel_grid_size_; //0.005f;

};
