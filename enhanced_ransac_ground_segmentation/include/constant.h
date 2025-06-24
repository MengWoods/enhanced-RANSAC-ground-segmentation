/*
 * Enhanced RANSAC Ground Segmentation
 * Copyright (c) 2025 Menghao Woods
 *
 * Licensed under the MIT License. See LICENSE file in the project root for details.
 */

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <filesystem>

namespace fs = std::filesystem;
using Point = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<pcl::PointXYZI>;
using PointCloudPtr = pcl::PointCloud<pcl::PointXYZI>::Ptr;
