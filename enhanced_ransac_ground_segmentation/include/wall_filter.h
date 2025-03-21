/*
 * Enhanced RANSAC Ground Segmentation
 * Copyright (c) 2025 Menghao Woods
 *
 * Licensed under the MIT License. See LICENSE file in the project root for details.
 */
#pragma once
#include "constant.h"

/**
 * @class WallFilter
 * @brief Filters vertical planes (walls) from a point cloud based on plane coefficients.
 */
class WallFilter 
{
public:
    /**
     * @brief Constructor.
     * @param threshold Distance threshold for wall filtering.
     */
    WallFilter(float threshold);

    /**
     * @brief Applies the wall filter to the input point cloud.
     * @param cloud Input point cloud (filtered in-place).
     * @param plane_coeffs Plane coefficients (A, B, C, D).
     */
    void applyFilter(PointCloudPtr &cloud, const std::vector<float> &plane_coeffs);

private:
    float threshold_;        ///< Distance threshold for filtering.
};
