/*
 * Enhanced RANSAC Ground Segmentation
 * Copyright (c) 2025 Menghao Woods
 *
 * Licensed under the MIT License. See LICENSE file in the project root for details.
 */
#pragma once
#include "common/constant.h"

#include <pcl/filters/voxel_grid.h>


/**
 * @class VoxelFilter
 * @brief A class for downsampling point clouds using a voxel grid filter.
 *
 * This class applies a voxel grid filter to reduce the number of points in a
 * point cloud while preserving its structure. The voxel size is configurable via YAML.
 */
class VoxelFilter
{
public:
    /**
     * @brief Constructor that initializes the voxel filter with parameters from a YAML node.
     * @param config YAML node containing the voxel size and verbosity flag.
     */
    explicit VoxelFilter(const YAML::Node &config);

    /**
     * @brief Applies the voxel filter to the input point cloud in-place.
     * @param cloud The point cloud to be filtered (modified in-place).
     */
    void applyFilter(PointCloudPtr &cloud);

private:
    bool enabled_;     ///< Flag to enable or disable filtering.
    float voxel_size_; ///< The size of the voxels for downsampling.
    bool verbose_;     ///< Verbosity flag to enable logging.
};
