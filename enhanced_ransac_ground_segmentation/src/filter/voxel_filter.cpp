/*
 * Enhanced RANSAC Ground Segmentation
 * Copyright (c) 2025 Menghao Woods
 *
 * Licensed under the MIT License. See LICENSE file in the project root for details.
 */
#include "filter/voxel_filter.h"

VoxelFilter::VoxelFilter(const YAML::Node &config)
{
    enabled_ = config["voxel_filter"]["enable"].as<bool>(true);  // Default: enabled
    voxel_size_ = config["voxel_filter"]["voxel_size"].as<float>(0.1f); // Default voxel size: 0.1
    verbose_ = config["verbose"].as<bool>(false);

    if (enabled_)
    {
        std::cout << "VoxelFilter enabled with voxel size: " << voxel_size_ << " meters." << std::endl;
    }
}


void VoxelFilter::applyFilter(PointCloudPtr &cloud)
{
    if (!enabled_)
    {
        return;
    }

    pcl::VoxelGrid<Point> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(voxel_size_, voxel_size_, voxel_size_);

    PointCloudPtr filtered_cloud(new PointCloud);
    sor.filter(*filtered_cloud);

    cloud.swap(filtered_cloud); // Replace input cloud with filtered version

    if (verbose_)
    {
        std::cout << "\t[VoxelFilter] Filtered cloud size: " << cloud->size() << " points." << std::endl;
    }
}
