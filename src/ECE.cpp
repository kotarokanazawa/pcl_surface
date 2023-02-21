#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>

ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *cloud);

  // Create a KdTree for search
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);

  // Create a vector for storing cluster indices
  std::vector<pcl::PointIndices> cluster_indices;

  // Create ECE object and set its parameters
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
  ece.setClusterTolerance(0.02); // 2cm
  ece.setMinClusterSize(100);
  ece.setMaxClusterSize(25000);
  ece.setSearchMethod(tree);
  ece.setInputCloud(cloud);

  // Extract clusters and store indices in cluster_indices
  ece.extract(cluster_indices);

  // Create new cloud for each cluster and publish it
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      cloud_cluster->push_back((*cloud)[*pit]); //*

    // Publish each cluster as a new point cloud message
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_cluster, output);
    output.header.frame_id = input->header.frame_id;
    pub.publish(output);

    j++;
  }
}

int main(int argc, char** argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "ece_node");
  ros::NodeHandle nh;

  // Subscribe to input point cloud topic
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/input_cloud", 1, cloud_cb);

  // Advertise output point cloud topic
  pub = nh.advertise<sensor_msgs::PointCloud2>("/output_cloud_topic", 1);

  // Spin
  ros::spin();
  return 0;
}