#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGB PointT;
class CloudHandler
{
public:
  CloudHandler() : nh_()
  {
    point_cloud_sub_ = nh_.subscribe("input_topic", 1, &CloudHandler::cloudCallback, this);
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
  {
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Downsample the point cloud
    pcl::PointCloud<PointT>::Ptr filtered_cloud_(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*filtered_cloud_);

    pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>);
    pass.setInputCloud(filtered_cloud_);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, 1.5);
    pass.filter(*filtered_cloud);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointT>::Ptr cloud_explane(new pcl::PointCloud<PointT>);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);

    ne.setSearchMethod(tree);
    ne.setInputCloud(filtered_cloud);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    // RANSAC PLANE
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(0.1);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(10);
    seg.setDistanceThreshold(0.1);
    seg.setInputCloud(filtered_cloud);
    seg.setInputNormals(cloud_normals);
    seg.segment(*inliers_plane, *coefficients_plane);

    extract.setInputCloud(filtered_cloud);
    extract.setIndices(inliers_plane);
    extract.setNegative(false);
    pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
    extract.filter(*cloud_plane);

    extract.setNegative(true);
    extract.filter(*cloud_explane);
    extract_normals.setNegative(true);
    extract_normals.setInputCloud(cloud_normals);
    extract_normals.setIndices(inliers_plane);
    extract_normals.filter(*cloud_normals2);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.1);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.01);
    seg.setRadiusLimits(0.01, 0.05);
    seg.setInputCloud(cloud_explane);
    seg.setInputNormals(cloud_normals2);
    seg.segment(*inliers_cylinder, *coefficients_cylinder);
    pcl::PointCloud<PointT>::Ptr cylinder_pointcloud(new pcl::PointCloud<PointT>);
    pcl::ExtractIndices<PointT> extract__;

    extract__.setInputCloud(cloud_explane);
    extract__.setIndices(inliers_cylinder);
    extract__.setNegative(false);
    extract__.filter(*cylinder_pointcloud);

    std::cout << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

    sensor_msgs::PointCloud2 pc2_plane, pc2_cylinder;
    pcl::toROSMsg(*cloud_plane, pc2_plane);
    pc2_pub.publish(pc2_plane);

    pcl::toROSMsg(*cylinder_pointcloud, pc2_cylinder);
    pc2_pub__.publish(pc2_cylinder);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber point_cloud_sub_;
  ros::Publisher pc2_pub = nh_.advertise<sensor_msgs::PointCloud2>("output_topic", 1);
  ros::Publisher pc2_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("output_topic_", 1);
  ros::Publisher pc2_pub__ = nh_.advertise<sensor_msgs::PointCloud2>("output_topic__", 1);

  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::PassThrough<PointT> pass;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_mesh_generator");
  CloudHandler handler;
  ros::spin();
}
