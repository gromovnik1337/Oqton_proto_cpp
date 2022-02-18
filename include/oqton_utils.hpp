#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// Define relevant point cloud types to simplify declarations and improve code readability
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;  

// Auxiliary function used to scale the cloud
void scaleTheCloud(double scaling_factor, PointCloudT::Ptr cloud_in) {

    for(int i = 0; i < cloud_in->points.size(); i++) {

    cloud_in->points[i].x = cloud_in->points[i].x / scaling_factor;
    cloud_in->points[i].y = cloud_in->points[i].y / scaling_factor;
    cloud_in->points[i].z = cloud_in->points[i].z / scaling_factor;
  }

} 
  