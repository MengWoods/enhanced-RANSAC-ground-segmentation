/*
 * Enhanced RANSAC Ground Segmentation
 * Copyright (c) 2025 Menghao Woods
 *
 * Licensed under the MIT License. See LICENSE file in the project root for details.
 */

#include "filter/noise_filter.h"

NoiseFilter::NoiseFilter(const YAML::Node &config)
{
    enabled_ = config["noise_filter"]["enable"].as<bool>(true);  // Default: enabled
    mean_k_ = config["noise_filter"]["mean_k"].as<int>(50);      // Default: 50 neighbors
    std_dev_ = config["noise_filter"]["std_dev"].as<double>(1.0); // Default: 1.0 std deviation
    verbose_ = config["noise_filter"]["verbose"].as<bool>(false);

    if (enabled_)
    {
        std::cout << "NoiseFilter enabled with mean_k: " << mean_k_
                  << ", std_dev: " << std_dev_ << std::endl;
    }
}

void NoiseFilter::applyFilter(PointCloudPtr &cloud)
{
    if (!enabled_) return;  // Skip filtering if disabled

    pcl::StatisticalOutlierRemoval<Point> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(mean_k_);
    sor.setStddevMulThresh(std_dev_);

    PointCloudPtr filtered_cloud(new PointCloud);
    sor.filter(*filtered_cloud);

    cloud.swap(filtered_cloud); // Replace input cloud with filtered version

    if (verbose_)
    {
        std::cout << "\t[NoiseFilter] Filtered cloud size: " << cloud->size() << " points." << std::endl;
    }
}
