/*
 * Enhanced RANSAC Ground Segmentation
 * Copyright (c) 2025 Menghao Woods
 *
 * Licensed under the MIT License. See LICENSE file in the project root for details.
 */

#include "constant.h"
#include "point_cloud_loader.h"
#include "point_cloud_visualizer.h"
#include "box_filter.h"
#include "voxel_filter.h"
#include "noise_filter.h"
#include "ground_estimation.h"

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <config_path>" << std::endl;
        return 1;
    }
    std::string config_path = argv[1];

    // Load configuration
    YAML::Node config = YAML::LoadFile(config_path);
    bool verbose = config["verbose"].as<bool>(false);
    bool visualization = config["point_cloud_visualizer"]["enable"].as<bool>(true);
    std::string point_cloud_path = config["point_cloud_path"].as<std::string>("/path/to/point_cloud.pcd");

    // Logs
    std::cout << "Verbose: " << (verbose ? "true" : "false") << std::endl;
    std::cout << "visualization: " << (visualization ? "true" : "false") << std::endl;

    // Instantiate
    PointCloudLoader loader(config);
    PointCloudVisualizer visualizer(config);
    BoxFilter box_filter(config);
    VoxelFilter voxel_filter(config);
    GroundEstimation ground_estimator(config);
    // Create NoiseFilter instance
    NoiseFilter noise_filter(config);

    if (visualization)
    {
        visualizer.initVisualizer();
    }

    PointCloudPtr cloud(new PointCloud());
    std::vector<float> ground_plane;


    while (loader.loadNextPointCloud(cloud))
    {
        // Create a copy of the point cloud for visualization
        PointCloudPtr cloud_copy(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::copyPointCloud(*cloud, *cloud_copy); // Copy data from loaded cloud to the copy
        // std::cout << "test" << std::endl;

        // Filters
        // Apply the filter (in-place modification)
        box_filter.applyFilter(cloud);
        voxel_filter.applyFilter(cloud);
        noise_filter.applyFilter(cloud);

        // ground estimation:  Ransac, wall filter, moving average, or kalman filter
        ground_estimator.estimateGround(cloud, ground_plane);

        std::cout << "Estimated ground plane: " << ground_plane[0] << "x + "
              << ground_plane[1] << "y + " << ground_plane[2] << "z + "
              << ground_plane[3] << " = 0" << std::endl;

        // Visualize the point cloud if visualization is enabled
        if (visualization && cloud->size() > 0)
        {
            // Update the visualizer with the current point cloud
            visualizer.updateVisualizer(cloud_copy);
        }
    }

    return 0;
}
