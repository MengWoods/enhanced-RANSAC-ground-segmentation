/*
 * Enhanced RANSAC Ground Segmentation
 * Copyright (c) 2025 Menghao Woods
 *
 * Licensed under the MIT License. See LICENSE file in the project root for details.
 */

#include "point_cloud_loader.h"

PointCloudLoader::PointCloudLoader(const YAML::Node &config)
{
    if (config["point_cloud_loader"]["point_cloud_paths"])
    {
        for (const auto& node : config["point_cloud_loader"]["point_cloud_paths"])
        {
            cloud_paths_.emplace_back(node.as<std::string>());
        }
    }
    verbose_ = config["point_cloud_loader"]["verbose"].as<bool>(false);
    cloud_type_ = config["point_cloud_loader"]["point_cloud_type"].as<std::string>("xyzi");

    // Log the initialization parameters
    std::cout << "[PointCloudLoader] Initialized with " << cloud_paths_.size()
              << " path(s), type: " << cloud_type_ << ", verbose: "
              << (verbose_ ? "true" : "false") << std::endl;

    loadFileList();
    file_iter_ = file_list_.begin();
}

bool PointCloudLoader::loadNextPointCloud(PointCloudPtr &cloud)
{
    if (file_iter_ == file_list_.end())
    {
        std::cout << "No more point clouds to load." << std::endl;
        return false;
    }

    std::string file_path = *file_iter_;
    cloud->clear();
    if (!loadBinFile(file_path, cloud))
    {
        std::cerr << "Failed to load: " << file_path << std::endl;
        return false;
    }

    if (verbose_)
    {
        std::cout << "Loaded point cloud from: " << file_path << " with " << cloud->size() << " points." << std::endl;
    }
    ++file_iter_;
    return true;
}

void PointCloudLoader::loadFileList()
{
    for (const auto& path : cloud_paths_)
    {
        for (const auto& entry : fs::directory_iterator(path))
        {
            if (entry.path().extension() == ".bin")
            {
                file_list_.push_back(entry.path().string());
            }
        }
    }

    std::sort(file_list_.begin(), file_list_.end());
}


bool PointCloudLoader::loadBinFile(const std::string &file_path, PointCloudPtr &cloud)
{
    std::ifstream input(file_path, std::ios::binary);
    if (!input.is_open())
        return false;

    // Iterate through the file and read points
    while (input.good())
    {
        float data[4];         // Array to hold the point data (x, y, z, intensity)
        size_t num_fields = 3; // Default for 'xyz'

        if (cloud_type_ == "xyzi")
        {
            num_fields = 4; // 'xyzi' contains x, y, z, intensity
        }
        else if (cloud_type_ == "xyziv")
        {
            num_fields = 4; // 'xyziv' contains x, y, z, intensity, and an extra field (ignore extra)
        }

        // Read the point data based on the type
        input.read(reinterpret_cast<char *>(data), sizeof(float) * num_fields);
        if (input.gcount() < sizeof(float) * num_fields)
            break; // End of file or incomplete point

        Point point;
        point.x = data[0];
        point.y = data[1];
        point.z = data[2];

        // Handle intensity based on the point cloud type
        if (num_fields == 3 || num_fields == 4)
        {
            // For 'xyz' or 'xyzi', use the intensity field (or dummy value if not present)
            point.intensity = (num_fields == 3) ? -10.0f : data[3]; // Dummy intensity if 'xyz' type
        }

        cloud->push_back(point);
    }

    input.close();
    return true;
}
