#include "point_cloud_visualizer.h"

PointCloudVisualizer::PointCloudVisualizer(const YAML::Node &config)
{
    // Initialize the PCL visualizer with the configuration settings
    camera_distance_ = config["point_cloud_visualizer"]["camera_distance"].as<float>(25.0f);
    angle_ = config["point_cloud_visualizer"]["angle"].as<float>(160.0f);
    refresh_interval_ = config["point_cloud_visualizer"]["refresh_interval"].as<int>(10);

    // Log the initialization parameters
    std::cout << "[PointCloudVisualizer] Initialized with camera_distance: " << camera_distance_
              << ", angle: " << angle_ << ", refresh_interval: " << refresh_interval_ << " ms" << std::endl;

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
    float camera_distance = camera_distance_;
    float angle = angle_;

    // Calculate the camera position based on rotation around the y-axis
    float x_position = camera_distance * cos(angle * M_PI / 180.0f);  // x position after rotation
    float z_position = camera_distance * sin(angle * M_PI / 180.0f);  // z position after rotation
    float y_position = 0.0f;  // Camera height (adjust as necessary)

    // Set the camera position and make it look at the origin (0, 0, 0)
    viewer_->setCameraPosition(x_position, y_position, z_position,  // Camera position (x, y, z)
                               0.0, 0.0, 0.0,                   // Look at the origin (0, 0, 0)
                               1.0, 0.0, 0.0);                  // Up direction along the x-axis
}

void PointCloudVisualizer::updateVisualizer(
    const PointCloud::Ptr &cloud, const std::vector<float> &ground_plane)
{
    // If no valid plane, fallback to single cloud visualization
    if (ground_plane.size() != 4)
    {
        if (!viewer_->updatePointCloud<Point>(cloud, "cloud"))
        {
            viewer_->addPointCloud<Point>(cloud, "cloud");
            viewer_->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
        }

        viewer_->spinOnce(refresh_interval_);
        return;
    }

    // Use helper function to split cloud
    auto [above_cloud, below_cloud] = splitCloudByPlane(cloud, ground_plane);

    // Visualize points above the plane (white)
    if (!viewer_->updatePointCloud<Point>(above_cloud, "above_cloud"))
    {
        viewer_->addPointCloud<Point>(above_cloud, "above_cloud");
        viewer_->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "above_cloud");
        viewer_->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "above_cloud");
    }

    // Visualize points on/below the plane (green)
    if (!viewer_->updatePointCloud<Point>(below_cloud, "below_cloud"))
    {
        viewer_->addPointCloud<Point>(below_cloud, "below_cloud");
        viewer_->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "below_cloud");
        viewer_->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "below_cloud");
    }

    viewer_->spinOnce(refresh_interval_);
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
    viewer_->spinOnce(refresh_interval_);
}

std::pair<PointCloud::Ptr, PointCloud::Ptr> PointCloudVisualizer::splitCloudByPlane(
    const PointCloud::Ptr& cloud, const std::vector<float>& plane_coeffs, float threshold)
{
    PointCloud::Ptr above(new PointCloud);
    PointCloud::Ptr below(new PointCloud);

    float a = plane_coeffs[0];
    float b = plane_coeffs[1];
    float c = plane_coeffs[2];
    float d = plane_coeffs[3];

    for (const auto& point : cloud->points)
    {
        float distance = a * point.x + b * point.y + c * point.z + d;
        if (distance > threshold)
            above->points.push_back(point);
        else
            below->points.push_back(point);
    }

    above->width = static_cast<uint32_t>(above->points.size());
    below->width = static_cast<uint32_t>(below->points.size());
    above->height = below->height = 1;
    above->is_dense = below->is_dense = true;

    return {above, below};
}
