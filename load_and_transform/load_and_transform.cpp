/*
This programe is the first step in creating a a pipeline for the ICP manipulation of two different RPD models.
It is based upon PCL library, and it it's main purpose is to learn and explore manipulation of 3D data in C++ environment.
It loades the .stl file and converts it into .pcd file. Subsequently, it transforms the cloud using a simple matrix and finally,
it visualizes the output.

Created by: Vice, 31.01.2022
*/

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/io/vtk_lib_io.h>
#include <pcl/PolygonMesh.h>

#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>

#include <pcl/visualization/cloud_viewer.h>


int main (int argc, char **argv) {

// Input file
const std::string mesh_input = "../../data/RPD_data.stl";
// Define a PolygonMesh object
pcl::PolygonMesh mesh;
// Load the file
pcl::io::loadPolygonFileSTL(mesh_input, mesh);

// Define XYZ point cloud object - Pointer
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
// Convert from the PolygonMesh to the PointCloud2
pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
std::cout << "Cloud loaded and converted. Size: "  
          << cloud->size()
          << std::endl;

// Define transformation matrix
/*
|-------> This column is the translation
| 1 0 0 x |  \
| 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
| 0 0 1 z |  /
| 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)
*/

Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity(); // Identiy matrix, to be filled up

// Define a rotation (see https://en.wikipedia.org/wiki/Rotation_matrix)
float theta = M_PI/4; // The angle of rotation in radians
transform_1(0, 0) = std::cos (theta);
transform_1(0, 1) = -sin(theta);
transform_1(1, 0) = sin (theta);
transform_1(1, 1) = std::cos (theta);

// Define a translation in X axis
transform_1(0, 3) = 0.1;

// Print out the transformation
printf("Transformation using a Matrix4f:\n");
std::cout << transform_1 << std::endl;

// Execute the transformation
pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::transformPointCloud(*cloud, *transformed_cloud, transform_1);

// Visualization
printf("\nPoint cloud colors: white  = original point cloud\n"
       "                      red  = transformed point cloud\n");

pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");
 // Define R,G,B colors for the point cloud, add the cloud to the viewer and pass the colour handelr
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloud, 255, 255, 255);
viewer.addPointCloud (cloud, source_cloud_color_handler, "original_cloud");
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (transformed_cloud, 230, 20, 20); // Red
viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

viewer.addCoordinateSystem (1.0, "cloud", 0);
viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to dark grey
viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
//viewer.setPosition(800, 400); // Setting visualiser window position

while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
  viewer.spinOnce ();
}
return 0;

}