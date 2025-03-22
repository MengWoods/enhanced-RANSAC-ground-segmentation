/*
 * Enhanced RANSAC Ground Segmentation
 * Copyright (c) 2025 Menghao Woods
 *
 * Licensed under the MIT License. See LICENSE file in the project root for details.
 */
#include "wall_filter.h"
#include <cmath>

WallFilter::WallFilter(float threshold)
    : threshold_(threshold)
{
}


void WallFilter::applyFilter(PointCloudPtr &cloud, const std::vector<float> &plane_coeffs)
{
    if (plane_coeffs.size() != 4)
    {
        std::cerr << "[WallFilter] Invalid plane coefficients size!" << std::endl;
        return;
    }

    float A = plane_coeffs[0];
    float B = plane_coeffs[1]; // Vertical component of normal
    float C = plane_coeffs[2];
    float D = plane_coeffs[3];

    // Compute the normal angle relative to the Y-axis
    float norm = std::sqrt(A * A + B * B + C * C);
    float angle = std::acos(B / norm) * 180.0f / M_PI; // Convert to degrees

    PointCloudPtr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());

    for (const auto &point : cloud->points)
    {
        // Compute distance to the plane: Ax + By + Cz + D
        float distance = std::abs(A * point.x + B * point.y + C * point.z + D) / norm;

        // Keep only points that are NOT within the threshold of the vertical wall
        if (distance > threshold_)
        {
            filtered_cloud->points.push_back(point);
        }
    }

    // Swap filtered cloud with original
    cloud->swap(*filtered_cloud);
    std::cout << "[WallFilter] Filtered cloud size: " << cloud->size() << " points." << std::endl;
}