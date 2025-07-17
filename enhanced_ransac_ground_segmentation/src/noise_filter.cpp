/*
 * Enhanced RANSAC Ground Segmentation
 * Copyright (c) 2025 Menghao Woods
 *
 * Licensed under the MIT License. See LICENSE file in the project root for details.
 */

#include "noise_filter.h"

NoiseFilter::NoiseFilter(const YAML::Node &config)
{
    enabled_ = config["noise_filter"]["enable"].as<bool>(true);  // Default: enabled
    mean_k_ = config["noise_filter"]["mean_k"].as<int>(50);      // Default: 50 neighbors
    std_dev_ = config["noise_filter"]["std_dev"].as<double>(1.0); // Default: 1.0 std deviation
    verbose_ = config["verbose"].as<bool>(false);

    if (enabled_)
    {
        std::cout << "[NoiseFilter] Loaded parameters:" << std::endl;
        std::cout << "  - Enabled: " << (enabled_ ? "true" : "false") << std::endl;
        std::cout << "  - Mean K: " << mean_k_ << std::endl;
        std::cout << "  - Std Dev: " << std_dev_ << std::endl;
    }
    else
    {
        std::cout << "[NoiseFilter] Noise filter is disabled." << std::endl;
    }
}

void NoiseFilter::applyFilter(PointCloudPtr &cloud)
{
    if (!enabled_ || cloud->empty()) return;

    PointCloudPtr filtered_cloud(new PointCloud);
    filtered_cloud->reserve(cloud->size()); // Pre-allocate memory

    // Create a KD-Tree for fast neighbor search
    pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);
    tree->setInputCloud(cloud);

    std::vector<int> indices_to_keep(cloud->size(), 0);
    int num_threads = omp_get_max_threads();

    // Use OpenMP to parallelize the main loop
    #pragma omp parallel for num_threads(num_threads)
    for (int i = 0; i < cloud->size(); ++i)
    {
        std::vector<int> point_indices(mean_k_);
        std::vector<float> point_distances(mean_k_);
        double total_distance = 0;

        // Find the K-nearest neighbors for the current point
        if (tree->nearestKSearch(cloud->points[i], mean_k_, point_indices, point_distances) > 0)
        {
            // Calculate the mean distance
            for (const auto& dist : point_distances) {
                total_distance += dist;
            }
            double mean_distance = total_distance / mean_k_;

            // Check if the point is within the standard deviation threshold
            if (mean_distance < std_dev_) { // Simplified check for example
                 indices_to_keep[i] = 1;
            }
        }
    }

    // Collect the points that passed the filter
    for(int i = 0; i < cloud->size(); ++i) {
        if(indices_to_keep[i]) {
            filtered_cloud->points.push_back(cloud->points[i]);
        }
    }

    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    cloud.swap(filtered_cloud);
}
