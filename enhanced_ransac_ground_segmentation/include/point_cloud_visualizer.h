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
     */
    PointCloudVisualizer();

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

private:
    pcl::visualization::PCLVisualizer::Ptr viewer_; ///< PCL visualizer instance
};