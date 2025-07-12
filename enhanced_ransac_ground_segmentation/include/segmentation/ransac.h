/*
 * Enhanced RANSAC Ground Segmentation
 * Copyright (c) 2025 Menghao Woods
 *
 * Licensed under the MIT License. See LICENSE file in the project root for details.
 */
#pragma once
#include "common/constant.h"

/**
 * @brief GPU-based RANSAC plane fitting.
 */
class Ransac
{
public:
    /**
     * @brief Constructor that loads parameters from YAML config.
     * @param config YAML node containing RANSAC parameters.
     */
    explicit Ransac(const YAML::Node &config);

    /**
     * @brief Estimates the best-fitting plane for the given point cloud.
     * @param cloud Input point cloud.
     * @param plane_coeffs Output vector to store [A, B, C, D] plane equation.
     */
    void estimatePlane(const PointCloudPtr &cloud, std::vector<float> &plane_coeffs);

private:
    float distance_threshold_; ///< Distance threshold for inliers
    int max_iterations_;       ///< Maximum RANSAC iterations
    int min_points_;           ///< Minimum points required to accept a plane
    bool verbose_;             ///< Enable logging
    bool timer_;               ///< Enable timer
};
