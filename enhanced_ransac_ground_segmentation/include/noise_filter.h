/*
 * Enhanced RANSAC Ground Segmentation
 * Copyright (c) 2025 Menghao Woods
 *
 * Licensed under the MIT License. See LICENSE file in the project root for details.
 */
#pragma once
#include "constant.h"

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/search/kdtree.h>

#include <omp.h>

/**
 * @class NoiseFilter
 * @brief A filter to remove noise and outliers from point clouds.
 */
class NoiseFilter
{
public:
    /**
     * @brief Constructor that loads filter parameters from a YAML node.
     * @param config YAML node containing filter parameters.
     */
    explicit NoiseFilter(const YAML::Node &config);

    /**
     * @brief Applies the noise filter in-place on the input point cloud.
     * @param cloud Point cloud to be filtered (modified in-place).
     */
    void applyFilter(PointCloudPtr &cloud);

private:
    bool enabled_;      ///< Flag to enable/disable the filter
    int mean_k_;        ///< Number of nearest neighbors to analyze for each point
    double std_dev_;    ///< Standard deviation multiplier threshold
    bool verbose_;      ///< Enable logging of filter details
};
