#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/pcd_io.h>

class CloudHandler
{
  public:
    CloudHandler() : nh_()
    {
      point_cloud_sub_ = nh_.subscribe("input_topic", 1, &CloudHandler::cloudCallback, this);
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(*cloud_msg, *cloud);

      // Downsample the point cloud
      pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setInputCloud(cloud);
      vg.setLeafSize(0.01f, 0.01f, 0.01f);
      vg.filter(*filtered_cloud);

      // Compute surface normal
      pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
      pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
      mls.setInputCloud(filtered_cloud);
      mls.setSearchRadius(0.03);
      mls.setPolynomialOrder(2);
      mls.process(*cloud_with_normals);

      // Generate mesh
      pcl::PolygonMesh mesh;
      pcl_msgs::PolygonMesh rosmesh;
      pcl::Poisson<pcl::PointNormal> poisson;
      poisson.setDepth(9);
      poisson.setInputCloud(cloud_with_normals);
      poisson.reconstruct(mesh);

      pcl::PolygonArray array;
      pcl_msgs::PolygonArray arra
      mesh_to_polygon_array(mesh,array);
      // Publish mesh
      pcl_conversions::fromPCL(mesh,rosmesh);
      mesh_pub_.publish(rosmesh);
    }

void mesh_to_polygon_array(const pcl::PolygonMesh& mesh, pcl::PolygonArray& polygon_array)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromPCLPointCloud2(mesh.cloud, cloud);
  
  for (size_t i = 0; i < mesh.polygons.size(); i++)
  {
    pcl::Vertices vertices;
    vertices.vertices = mesh.polygons[i].vertices;
    polygon_array.polygons.push_back(vertices);
  }

  polygon_array.header = mesh.header;
  polygon_array.point_representation.reset(new pcl::DefaultPointRepresentation<pcl::PointXYZ>());
  polygon_array.setVertices(cloud);
}

  private:
    ros::NodeHandle nh_;
    ros::Subscriber point_cloud_sub_;
    ros::Publisher mesh_pub_ = nh_.advertise<pcl_msgs::PolygonMesh>("output_topic", 1);
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_mesh_generator");
  CloudHandler handler;
  ros::spin();
}
