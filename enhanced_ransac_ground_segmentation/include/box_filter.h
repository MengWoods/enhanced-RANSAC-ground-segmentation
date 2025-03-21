/*
 * Enhanced RANSAC Ground Segmentation
 * Copyright (c) 2025 Menghao Woods
 *
 * Licensed under the MIT License. See LICENSE file in the project root for details.
 */
#pragma once
#include "constant.h"

#include <pcl/filters/crop_box.h>


/**
 * @class BoxFilter
 * @brief Class for filtering point clouds within a bounding box.
 */
class BoxFilter
{
public:
    /**
     * @brief Constructs a BoxFilter object with the given YAML configuration.
     * 
     * @param config YAML node containing filter configuration parameters.
     */
    BoxFilter(const YAML::Node& config);

    /**
     * @brief Applies the bounding box filter to the input point cloud (in-place).
     * @param cloud The point cloud to be filtered.
     */
    void applyFilter(PointCloudPtr& cloud);

private:
    bool enable_;       ///< Flag to enable or disable the filter.
    float xmin_;        ///< Minimum X value for the bounding box.
    float xmax_;        ///< Maximum X value for the bounding box.
    float ymin_;        ///< Minimum Y value for the bounding box.
    float ymax_;        ///< Maximum Y value for the bounding box.
    float zmin_;        ///< Minimum Z value for the bounding box.
    float zmax_;        ///< Maximum Z value for the bounding box.
    bool verbose_;      ///< Flag for verbose logging.

    pcl::CropBox<Point> crop_filter_; ///< PCL crop filter instance.
};