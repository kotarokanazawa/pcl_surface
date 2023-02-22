#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/dbscan.h>

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Convert sensor_msgs/PointCloud2 to PCL/PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud_msg, *cloud);

  // DBSCAN parameters
  double eps = 0.1;
  int min_pts = 10;

  // Perform DBSCAN clustering
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::DBSCAN<pcl::PointXYZ> dbscan;
  dbscan.setEps(eps);
  dbscan.setMinPts(min_pts);
  dbscan.setInputCloud(cloud);
  dbscan.cluster(cluster_indices);

  // Output cluster indices
  for (auto& indices : cluster_indices) {
    ROS_INFO("Cluster with %d points", indices.indices.size());
    for (auto& index : indices.indices) {
      ROS_INFO("  Point (%f, %f, %f)", cloud->points[index].x, cloud->points[index].y, cloud->points[index].z);
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dbscan_node");
  ros::NodeHandle nh;

  // Subscribe to point cloud topic
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("point_cloud_topic", 1, pointCloudCallback);

  ros::spin();

  return 0;
}