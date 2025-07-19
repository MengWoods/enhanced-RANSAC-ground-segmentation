/*
 * Enhanced RANSAC Ground Segmentation
 * Copyright (c) 2025 Menghao Woods
 *
 * Licensed under the MIT License. See LICENSE file in the project root for details.
 */

#include "constant.h"

#include <chrono>
#include <thread>

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
    bool timer = config["timer"].as<bool>(false);
    float frequency = config["frequency"].as<float>(10.0f); // Default frequency is 10 Hz
    bool visualization = config["point_cloud_visualizer"]["enable"].as<bool>(true);
    bool ground_estimation = config["ground_estimation"]["enable"].as<bool>(true);

    auto target_frame_duration = std::chrono::duration<double>(1.0 / frequency);

    // Logs
    std::cout << "Verbose: " << (verbose ? "true" : "false") << std::endl;
    std::cout << "Timer: " << (timer ? "true" : "false") << std::endl;
    std::cout << "Frequency: " << frequency << " Hz" << std::endl;
    std::cout << "Target frame duration: " << target_frame_duration.count() * 1000 << " ms" << std::endl;
    std::cout << "visualization: " << (visualization ? "true" : "false") << std::endl;
    std::cout << "ground_estimation: " << (ground_estimation ? "true" : "false") << std::endl;

    std::cout << "Waiting for 5 seconds before starting..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Initialize components
    PointCloudLoader point_cloud_loader(config);
    PointCloudVisualizer point_cloud_visualizer(config);
    BoxFilter box_filter(config);
    VoxelFilter voxel_filter(config);
    GroundEstimation ground_estimator(config);
    NoiseFilter noise_filter(config);

    PointCloudPtr cloud(new PointCloud());
    std::vector<float> ground_plane;

    while (point_cloud_loader.loadNextPointCloud(cloud))
    {
        // Init
        auto frame_start = std::chrono::steady_clock::now();
        if (timer) std::cout << "------------------- Frame -------------------" << std::endl;

        // Create a copy of the point cloud for visualization
        PointCloudPtr cloud_copy(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::copyPointCloud(*cloud, *cloud_copy); // Copy data from loaded cloud to the copy

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
        bool ground_estimated = ground_estimator.estimateGround(cloud, ground_plane);
        if (ground_estimated)
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
            if (verbose)
            {
                std::cout << "\t[GroundEstimation] Failed to estimate ground plane." << std::endl;
            }
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
            // Update the point_cloud_visualizer with the current point cloud and/or ground plane
            if (ground_estimated)
            {
                point_cloud_visualizer.pushFrame(cloud_copy, ground_plane);
            }
            else
            {
                point_cloud_visualizer.pushFrame(cloud_copy);
            }
        }

        if (timer)
        {
            auto t1 = std::chrono::steady_clock::now();
            std::cout << "[Timer] Visualization: "
                    << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                    << " ms" << std::endl;
            t0 = t1;
        }


        // --- FREQUENCY CONTROL LOGIC --- //
        auto processing_end = std::chrono::steady_clock::now();
        auto processing_duration = processing_end - frame_start;

        // If the frame was processed faster than our target, wait for the remaining time
        if (processing_duration < target_frame_duration)
        {
            auto wait_duration = target_frame_duration - processing_duration;
            std::this_thread::sleep_for(wait_duration);
        }
        // If processing took too long, the loop will just continue immediately.

        if (timer)
        {
            auto frame_end = std::chrono::steady_clock::now();
            auto frame_duration = std::chrono::duration_cast<std::chrono::milliseconds>(frame_end - frame_start).count();
            std::cout << "[Timer] Total Frame Time: " << frame_duration << " ms" << std::endl;
        }
        if (timer) std::cout << "------------------- End ---------------------" << std::endl;
        std::cout << std::flush;
    }

    return 0;
}
