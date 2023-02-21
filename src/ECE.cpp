#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
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
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>


typedef pcl::PointXYZRGB PointT;
class CloudHandler
{
public:
  CloudHandler() : nh_()
  {
    point_cloud_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &CloudHandler::cloudCallback, this);
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
  {
    pcl::PCDReader reader;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>), cloud_f(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Downsample the point cloud
    pcl::PointCloud<PointT>::Ptr filtered_cloud_(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.005f, 0.005f, 0.005f);
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
    extract.setNegative(true);
    pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
    extract.filter(*filtered_cloud);


    // int i = 0, nr_points = (int)filtered_cloud->size();
    // while (filtered_cloud->size() > 0.3 * nr_points)
    // {
    //   // Segment the largest planar component from the remaining cloud
    //   seg.setInputCloud(filtered_cloud);
    //   seg.segment(*inliers, *coefficients);
    //   if (inliers->indices.size() == 0)
    //   {
    //     std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    //     break;
    //   }

      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<PointT> extract;
      extract.setInputCloud(filtered_cloud);
      extract.setIndices(inliers);
      extract.setNegative(false);

      // Get the points associated with the planar surface
      extract.filter(*cloud_plane);
      std::cout << "PointCloud representing the planar component: " << cloud_plane->size() << " data points." << std::endl;

      // Remove the planar inliers, extract the rest
      extract.setNegative(true);
      extract.filter(*cloud_f);
      *filtered_cloud = *cloud_f;
    // Creating the KdTree object for the search method of the extraction
    tree->setInputCloud(filtered_cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.01); // 2cm
    ec.setMinClusterSize(200);
    ec.setMaxClusterSize(2000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(filtered_cloud);
    ec.extract(cluster_indices);

    std::vector<Eigen::Vector3i> colors = {
    Eigen::Vector3i(255, 0, 0),    // Red
    Eigen::Vector3i(0, 255, 0),    // Green
    Eigen::Vector3i(0, 0, 255),    // Blue
    Eigen::Vector3i(255, 255, 0),  // Yellow
    Eigen::Vector3i(0, 255, 255),  // Cyan
    Eigen::Vector3i(255, 0, 255),  // Magenta
    Eigen::Vector3i(255, 255, 255) // White
};

    // for (size_t i = 0; i < cluster_indices.size(); ++i)
    // {
    //   colors[i](0) = rand() % 256;
    //   colors[i](1) = rand() % 256;
    //   colors[i](2) = rand() % 256;
    // }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);

    // Set the frame ID and timestamp
    output.header.frame_id = cloud_msg->header.frame_id;
    output.header.stamp = ros::Time::now();

    // Set the point cloud fields
    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
pcl::PointCloud<PointT>::Ptr cloud_cluster_all(new pcl::PointCloud<PointT>);
    // Set the color for each point in each cluster
    for (size_t i = 0; i < cluster_indices.size(); ++i)
    {
      pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
      for (std::vector<int>::const_iterator pit = cluster_indices[i].indices.begin(); pit != cluster_indices[i].indices.end(); ++pit)
        cloud_cluster->push_back((*filtered_cloud)[*pit]);

// Choose a color for the cluster
    Eigen::Vector3i color = colors[i % colors.size()]; // Select a color from the color table
    uint8_t r = color(0);
    uint8_t g = color(1);
    uint8_t b = color(2);

    // Apply the color to all points in the cluster
    for (size_t j = 0; j < cloud_cluster->size(); ++j)
    {
        cloud_cluster->points[j].r = r;
        cloud_cluster->points[j].g = g;
        cloud_cluster->points[j].b = b;
    }
      *cloud_cluster_all += *cloud_cluster;
    }
    // Convert the combined point cloud to ROS message and publish it
pcl::toROSMsg(*cloud_cluster_all, output);
output.header.frame_id = cloud_msg->header.frame_id;
output.header.stamp = ros::Time::now();
pc2_pub.publish(output);

      sensor_msgs::PointCloud2 pc2;
      pcl::toROSMsg(*filtered_cloud, pc2);
      pc2_pub_.publish(pc2);
    
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