/*
 * Enhanced RANSAC Ground Segmentation
 * Copyright (c) 2025 Menghao Woods
 *
 * Licensed under the MIT License. See LICENSE file in the project root for details.
 */
#pragma once
#include "constant.h"

class PointCloudLoader
{
public:
    /**
     * @brief Constructs a PointCloudLoader instance.
     * Initializes the loader with a YAML configuration node.
     * Use `explicit` for single-argument constructors to avoid unintended implicit conversions.
     * @param config YAML configuration node containing "point_cloud_path".
     */
    explicit PointCloudLoader(const YAML::Node &config);

    /**
     * @brief Loads the next point cloud from the file list.
     * 
     * Reads a .bin point cloud file, parses its contents, and stores the result
     * in the provided PointCloud pointer. If all files have been loaded, it returns false.
     * 
     * @param cloud Pointer to the PointCloud object where the data will be stored.
     * @return True if a point cloud was successfully loaded, false if no more files remain.
     */
    bool loadNextPointCloud(PointCloudPtr &cloud);

private:
    bool verbose_;                          ///< Verbose mode or not
    std::string cloud_type_;                ///< Point cloud data type, xyz, xyzi or xyziv
    std::string cloud_path_;                ///< Path to the cloud data directory.
    std::vector<std::string> file_list_;    ///< List of filenames containing point cloud data.
    std::vector<std::string>::iterator file_iter_;  ///< Iterator for traversing through the file list.

    /**
     * @brief Loads the list of .bin files from the specified directory.
     * 
     * Scans the directory provided in the YAML configuration for .bin files
     * and stores their paths in a vector for sequential access.
     */
    void loadFileList();

    /**
     * @brief Loads a single .bin file into a PointCloud object.
     * 
     * Reads a binary file containing LiDAR point cloud data, parses it,
     * and populates the provided PointCloud pointer with the points.
     * 
     * @param file_path Path to the .bin file to be loaded.
     * @param cloud Pointer to the PointCloud object where data will be stored.
     * @return True if loading was successful, false otherwise.
     */
    bool loadBinFile(const std::string &file_path, PointCloudPtr &cloud);
};
