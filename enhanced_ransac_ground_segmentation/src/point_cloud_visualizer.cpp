#include "point_cloud_visualizer.h"

PointCloudVisualizer::PointCloudVisualizer()
{
    viewer_ = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("PointCloud Viewer"));
}

PointCloudVisualizer::~PointCloudVisualizer()
{
    if (viewer_)
    {
        viewer_->close();
    }
}

void PointCloudVisualizer::initVisualizer()
{
    // Set default point cloud rendering properties
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    viewer_->addCoordinateSystem(1.0); // Add coordinate system for reference
    viewer_->initCameraParameters(); // Initialize camera parameters for the viewer

    // Set the camera distance from the origin (adjust as necessary)
    float camera_distance = 25.0f;  // Distance from the origin (camera to object)
    float angle = 160.0f;           // Camera rotation angle in degrees around y-axis (horizontal rotation)

    // Calculate the camera position based on rotation around the y-axis
    float x_position = camera_distance * cos(angle * M_PI / 180.0f);  // x position after rotation
    float z_position = camera_distance * sin(angle * M_PI / 180.0f);  // z position after rotation
    float y_position = 0.0f;  // Camera height (adjust as necessary)

    // Set the camera position and make it look at the origin (0, 0, 0)
    viewer_->setCameraPosition(x_position, y_position, z_position,  // Camera position (x, y, z)
                               0.0, 0.0, 0.0,                   // Look at the origin (0, 0, 0)
                               1.0, 0.0, 0.0);                  // Up direction along the x-axis
}

void PointCloudVisualizer::updateVisualizer(const PointCloud::Ptr &cloud)
{
    // If the point cloud is already added, update it
    if (!viewer_->updatePointCloud<Point>(cloud, "cloud"))
    {
        // If the point cloud is not yet added, add it to the visualizer
        viewer_->addPointCloud<Point>(cloud, "cloud");
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    }

    // Spin the viewer once to update the display (this is done in the loop)
    viewer_->spinOnce(100);  // 100 milliseconds per loop iteration
}
