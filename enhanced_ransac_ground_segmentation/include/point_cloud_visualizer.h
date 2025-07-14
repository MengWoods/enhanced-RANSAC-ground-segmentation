/*
 * Enhanced RANSAC Ground Segmentation
 * Copyright (c) 2025 Menghao Woods
 *
 * Licensed under the MIT License. See LICENSE file in the project root for details.
 */
#pragma once
#include "constant.h"

#include <pcl/visualization/pcl_visualizer.h>

class PointCloudVisualizer
{
public:
    /**
     * @brief Default constructor initializes the PCL visualizer.
     * @param config YAML configuration node containing visualizer settings.
     */
    PointCloudVisualizer(const YAML::Node &config);

    /**
     * @brief Destructor that ensures visualizer resources are released.
     */
    ~PointCloudVisualizer();

    /**
     * @brief Initializes the visualizer, adds coordinate system and sets camera parameters.
     */
    void initVisualizer();

    /**
     * @brief Updates the visualizer with the new point cloud.
     *        If the point cloud is not added yet, it will add the point cloud.
     * @param cloud The point cloud to visualize.
     */
    void updateVisualizer(const PointCloud::Ptr &cloud);

    /**
     * @brief Updates the visualizer with the new point cloud and ground plane coefficients.
     *        If the ground plane is valid, it will visualize points into different colors based on the ground plane.
     * @param cloud The point cloud to visualize.
     * @param ground_plane Coefficients of the ground plane (a, b, c, d).
     */
    void updateVisualizer(const PointCloud::Ptr &cloud, const std::vector<float> &ground_plane);

private:
    pcl::visualization::PCLVisualizer::Ptr viewer_; ///< PCL visualizer instance
    float camera_distance_;                         ///< Distance from the origin (camera to object)
    float angle_;                                   ///< Camera rotation angle in degrees around y-axis (horizontal rotation)
    int refresh_interval_;                          ///< Time interval in ms for refreshing the visualization

    /**
     * @brief Splits the point cloud into two parts based on the ground plane coefficients.
     *        Points above the plane are considered as non-ground, and points below are considered as ground.
     * @param cloud The point cloud to split.
     * @param plane_coeffs Coefficients of the ground plane (a, b, c, d).
     * @param threshold Distance threshold to consider a point as above or below the plane.
     * @return A pair of point clouds: first is above the plane, second is below the plane.
     */
    std::pair<PointCloud::Ptr, PointCloud::Ptr> splitCloudByPlane(
        const PointCloud::Ptr& cloud,
        const std::vector<float>& plane_coeffs,
        float threshold = 0.1f);
};
