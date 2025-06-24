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

#include <chrono>


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
    bool timer = config["timer"].as<bool>(false);
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
        // Init
        auto frame_start = std::chrono::steady_clock::now();
        if (timer) std::cout << "------------------- Frame -------------------" << std::endl;

        // Create a copy of the point cloud for visualization
        PointCloudPtr cloud_copy(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::copyPointCloud(*cloud, *cloud_copy); // Copy data from loaded cloud to the copy
        // std::cout << "test" << std::endl;

        // Filters
        // Apply the filter (in-place modification)
        auto t0 = std::chrono::steady_clock::now();
        box_filter.applyFilter(cloud);
        if (timer)
        {
            auto t1 = std::chrono::steady_clock::now();
            std::cout << "[Timer] Box Filter: "
                    << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                    << " ms" << std::endl;
            t0 = t1;
        }

        voxel_filter.applyFilter(cloud);
        if (timer)
        {
            auto t1 = std::chrono::steady_clock::now();
            std::cout << "[Timer] Voxel Filter: "
                    << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                    << " ms" << std::endl;
            t0 = t1;
        }

        noise_filter.applyFilter(cloud);
        if (timer)
        {
            auto t1 = std::chrono::steady_clock::now();
            std::cout << "[Timer] Noise Filter: "
                    << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                    << " ms" << std::endl;
            t0 = t1;
        }

        // ground estimation:  Ransac, wall filter, moving average, or kalman filter
        // ground_estimator.estimateGround(cloud, ground_plane);
        if (ground_estimator.estimateGround(cloud, ground_plane))
        {
            if (verbose)
            {
                std::cout << "\t[GroundEstimation] Estimated plane coefficients: "
                          << ground_plane[0] << ", " << ground_plane[1] << ", "
                          << ground_plane[2] << ", " << ground_plane[3] << std::endl;
            }
        }
        else
        {
            std::cout << "\t[GroundEstimation] Failed to estimate ground plane." << std::endl;
        }

        if (timer)
        {
            auto t1 = std::chrono::steady_clock::now();
            std::cout << "[Timer] Ground Estimation: "
                    << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                    << " ms" << std::endl;
            t0 = t1;
        }

        // Visualize the point cloud if visualization is enabled
        if (visualization && cloud->size() > 0)
        {
            // Update the visualizer with the current point cloud
            visualizer.updateVisualizer(cloud_copy);
            if (timer)
            {
                auto t1 = std::chrono::steady_clock::now();
                std::cout << "[Timer] Visualization: "
                        << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                        << " ms" << std::endl;
            }
        }

        if (timer)
        {
            auto frame_end = std::chrono::steady_clock::now();
            auto frame_duration = std::chrono::duration_cast<std::chrono::milliseconds>(frame_end - frame_start).count();
            std::cout << "[Timer] Total Frame Time: " << frame_duration << " ms" << std::endl;
        }
    }

    return 0;
}
