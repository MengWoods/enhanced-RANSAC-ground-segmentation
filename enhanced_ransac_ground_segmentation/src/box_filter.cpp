/*
 * Enhanced RANSAC Ground Segmentation
 * Copyright (c) 2025 Menghao Woods
 *
 * Licensed under the MIT License. See LICENSE file in the project root for details.
 */
#include "box_filter.h"

BoxFilter::BoxFilter(const YAML::Node& config)
{
    // Load configuration parameters from YAML
    enable_ = config["filter_box"]["enable"].as<bool>(false);
    xmin_ = config["filter_box"]["xmin"].as<float>(-20.0);
    xmax_ = config["filter_box"]["xmax"].as<float>(20.0);
    ymin_ = config["filter_box"]["ymin"].as<float>(-20.0);
    ymax_ = config["filter_box"]["ymax"].as<float>(20.0);
    zmin_ = config["filter_box"]["zmin"].as<float>(-5.0);
    zmax_ = config["filter_box"]["zmax"].as<float>(5.0);
    verbose_ = config["verbose"].as<bool>(false);

    if (enable_)
    {
        std::cout << "BoxFilter initialized with:" << std::endl;
        std::cout << "  xmin: " << xmin_ << ", xmax: " << xmax_ << std::endl;
        std::cout << "  ymin: " << ymin_ << ", ymax: " << ymax_ << std::endl;
        std::cout << "  zmin: " << zmin_ << ", zmax: " << zmax_ << std::endl;
    }

    // Set up the PCL crop box filter with the bounding box limits
    crop_filter_.setMin(Eigen::Vector4f(xmin_, ymin_, zmin_, 1.0));
    crop_filter_.setMax(Eigen::Vector4f(xmax_, ymax_, zmax_, 1.0));
}

void BoxFilter::applyFilter(PointCloudPtr& cloud)
{
    if (!enable_)
    {
        return;
    }

    PointCloudPtr filtered_cloud(new PointCloud);
    crop_filter_.setInputCloud(cloud);
    crop_filter_.filter(*filtered_cloud);

    cloud.swap(filtered_cloud); // In-place modification

    if (verbose_)
    {
        std::cout << "\t[BoxFilter] Filtered cloud size: " << cloud->size() << std::endl;
    }
}
