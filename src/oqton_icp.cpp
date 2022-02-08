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
#include <pcl/console/time.h>

#include <pcl/registration/icp.h>

#include <pcl/visualization/cloud_viewer.h>

// Define relevant point cloud types to simplify declarations and improve code readability
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int main (int argc, char **argv) {

  // Input files
  const std::string mesh_model = "../../data/RPD_model.stl";
  const std::string mesh_data = "../../data/RPD_data.stl";

  // Define a PolygonMesh objects
  pcl::PolygonMesh mesh_model_obj;
  pcl::PolygonMesh mesh_data_obj;

  // Load the files
  pcl::io::loadPolygonFileSTL(mesh_model, mesh_model_obj);
  pcl::io::loadPolygonFileSTL(mesh_data, mesh_data_obj);

  // Define XYZ point cloud objects (pointers)
  PointCloudT::Ptr cloud_model (new PointCloudT);
  PointCloudT::Ptr cloud_data (new PointCloudT);

  // Convert from the PolygonMesh to the PointCloud2
  pcl::fromPCLPointCloud2(mesh_model_obj.cloud, *cloud_model);
  pcl::fromPCLPointCloud2(mesh_data_obj.cloud, *cloud_data);
  std::cout << "Clouds loaded and converted. \n" 
            << "Model cloud size, points: " << cloud_model->size() << "\n"
            << "Data cloud size, points: " << cloud_data->size() << "\n"
            << std::endl;

  // Scale down the point clouds, into the milimiter range, for intuitive reasons
  //TODO Refactor!
  double scaling_factor = 1000;

  for (int i = 0; i < cloud_model->points.size(); i++) {

    cloud_model->points[i].x = cloud_model->points[i].x / scaling_factor;
    cloud_model->points[i].y = cloud_model->points[i].y / scaling_factor;
    cloud_model->points[i].z = cloud_model->points[i].z / scaling_factor;
  }

   for (int i = 0; i < cloud_data->points.size(); i++) {

    cloud_data->points[i].x = cloud_data->points[i].x / scaling_factor;
    cloud_data->points[i].y = cloud_data->points[i].y / scaling_factor;
    cloud_data->points[i].z = cloud_data->points[i].z / scaling_factor;
  }

  // Define transformation matrix
  /*
  |-------> This column is the translation
  | 1 0 0 x |  \
  | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
  | 0 0 1 z |  /
  | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)
  */

  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity(); // Identiy matrix, to be filled up

  // Define a rotation 
  float theta = M_PI / 4; // The angle of rotation in radians
  transform_1(0, 0) = std::cos (theta);
  transform_1(0, 1) = -sin(theta);
  transform_1(1, 0) = sin (theta);
  transform_1(1, 1) = std::cos (theta);

  // Define a translation in X axis
  transform_1(0, 3) = 50;

  // Print out the transformation
  printf("Transformation using a Matrix4f:\n");
  std::cout << transform_1 << std::endl;

  // Execute the transformation
  PointCloudT::Ptr transformed_cloud (new PointCloudT);
  pcl::transformPointCloud(*cloud_data, *transformed_cloud, transform_1);

  // Save the transformed cloud in .pcd format, if required
  //pcl::io::savePCDFile<pcl::PointXYZ>("transformed_cloud.pcd", *transformed_cloud); 

  // Visualization
  //TODO Refactor
  printf("\nPoint cloud colors: white  = Model cloud\n"
        "                      red  = Data cloud, transformed\n");

  pcl::visualization::PCLVisualizer viewer_initial ("Initial state");

  // Define R,G,B colors for the point cloud, add the cloud to the viewer and pass the colour handler
  pcl::visualization::PointCloudColorHandlerCustom<PointT> data_cloud_color_handler (cloud_data, 255, 255, 255); // White
  viewer_initial.addPointCloud (cloud_data, data_cloud_color_handler, "data_cloud");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> model_cloud_color_handler (transformed_cloud, 230, 20, 20); // Red
  viewer_initial.addPointCloud (transformed_cloud, model_cloud_color_handler, "model_cloud");

  // Define auxiliary parameters of the viewer
  viewer_initial.addCoordinateSystem (1.0, "cloud", 0);
  viewer_initial.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to dark grey
  viewer_initial.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "data_cloud");
  viewer_initial.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "model_cloud");
  //viewer.setPosition(800, 400); // Setting visualiser window position

  while (!viewer_initial.wasStopped ()) { // Display the visualiser until 'q' key is pressed

    viewer_initial.spinOnce ();
  }

  // The Iterative Closest Point algorithm
  int iterations = 500;
  PointCloudT::Ptr cloud_icp (new PointCloudT);
  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

  // Count the execution time
  pcl::console::TicToc time;
  time.tic ();

  //Create the ICP object
  pcl::IterativeClosestPoint<PointT, PointT> icp;

  icp.setMaximumIterations (iterations);
  icp.setInputSource (cloud_icp);
  icp.setInputTarget (cloud_model);
  icp.align (*cloud_icp);
  icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function

  if (icp.hasConverged ()) {

    std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
    std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
    transformation_matrix = icp.getFinalTransformation ().cast<double>();
    // Print out the transformation
    printf("ICP transformation matrix:\n");
    std::cout << transformation_matrix << std::endl;
  }

  else {

    PCL_ERROR ("\nICP has not converged.\n");
    return (-1);
  }

  return 0;
}