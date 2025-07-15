/*
 * Enhanced RANSAC Ground Segmentation
 * Copyright (c) 2025 Menghao Woods
 *
 * Licensed under the MIT License. See LICENSE file in the project root for details.
 */
#include "ground_estimation.h"
#include <iostream>
#include <Eigen/Dense>
#include <cmath>

GroundEstimation::GroundEstimation(const YAML::Node &config)
    : ransac_(std::make_unique<Ransac>(config)),
      wall_filter_(std::make_unique<WallFilter>(config["ground_estimation"]["wall_filter"]["threshold"].as<float>(0.2f)))
{
    // Load parameters from config
    enable_ = config["ground_estimation"]["enable"].as<bool>(true);
    buffer_size_ = config["ground_estimation"]["buffer_size"].as<int>(10);
    max_angle_ = config["ground_estimation"]["max_angle"].as<float>(10.0f);
    max_height_ = config["ground_estimation"]["max_height"].as<float>(0.2f);
    min_points_ = config["ground_estimation"]["min_points"].as<int>(50);
    z_offset_ = config["ground_estimation"]["z_offset"].as<float>(0.1f);
    wall_filter_enabled_ = config["ground_estimation"]["wall_filter"]["enable"].as<bool>(true);
    max_rerun_times_ = config["ground_estimation"]["wall_filter"]["max_rerun_times"].as<int>(3);
    wall_threshold_ = config["ground_estimation"]["wall_filter"]["threshold"].as<float>(0.2f);

    ransac_ = std::make_unique<Ransac>(config);
    wall_filter_ = std::make_unique<WallFilter>(wall_threshold_);

    // Log the initialization parameters
    if (enable_)
    {
        std::cout << "[GroundEstimation] Loaded parameters:" << std::endl;
        std::cout << "  - Enable: " << (enable_ ? "true" : "false") << std::endl;
        std::cout << "  - Buffer Size: " << buffer_size_ << std::endl;
        std::cout << "  - Max Angle: " << max_angle_ << " degrees" << std::endl;
        std::cout << "  - Max Height: " << max_height_ << " meters" << std::endl;
        std::cout << "  - Min Points: " << min_points_ << std::endl;
        std::cout << "  - Z Offset: " << z_offset_ << " meters" << std::endl;
        std::cout << "  - Wall Filter Enabled: " << (wall_filter_enabled_ ? "true" : "false") << std::endl;
        std::cout << "  - Max Rerun Times: " << max_rerun_times_ << std::endl;
        std::cout << "  - Wall Threshold: " << wall_threshold_ << std::endl;
    }
    else
    {
        std::cout << "[GroundEstimation] Ground estimation is disabled." << std::endl;
    }
}

bool GroundEstimation::estimateGround(PointCloudPtr &cloud, std::vector<float> &plane_coeffs)
{
    std::cout << "[GroundEstimation] Buffer size: " << buffer_.size() << std::endl;
    if (!enable_)
    {
        return false;
    }
    if (cloud->size() < min_points_)
    {
        if (!getAverageFromBuffer(plane_coeffs))
        {
            std::cerr << "[GroundEstimation] Warning: Not enough points and no history available." << std::endl;
            return false;
        }
        return true;
    }

    int rerun_count = 0;
    bool valid_ground = false;

    while (!valid_ground && rerun_count <= max_rerun_times_)
    {
        ransac_->estimatePlane(cloud, plane_coeffs);

        flipPlaneIfNecessary(plane_coeffs);

        if (wall_filter_enabled_ && isWallLike(plane_coeffs))
        {
            wall_filter_->applyFilter(cloud, plane_coeffs);
            rerun_count++;
        }
        else
        {
            valid_ground = true;
        }
    }

    if (!valid_ground)
    {
        if (!getAverageFromBuffer(plane_coeffs))
        {
            std::cerr << "\t[GroundEstimation] Warning: Failed to estimate ground plane after "
                      << rerun_count << " attempts and no history available." << std::endl;
            return false;
        }
    }

    if (isGroundValid(plane_coeffs))
    {
        plane_coeffs[3] -= plane_coeffs[2] * z_offset_;
        saveToBuffer(plane_coeffs);
    }
    else
    {
        if (!getAverageFromBuffer(plane_coeffs))
        {
            std::cerr << "\t[GroundEstimation] Warning: Estimated ground plane is invalid and no history available." << std::endl;
            return false;
        }
    }
    return true;
}

void GroundEstimation::flipPlaneIfNecessary(std::vector<float> &plane_coeffs)
{
    if (plane_coeffs[2] < 0)
    {
        for (auto &coeff : plane_coeffs)
        {
            coeff *= -1;
        }
    }
}

bool GroundEstimation::isWallLike(const std::vector<float> &plane_coeffs) const
{
    return std::abs(plane_coeffs[2]) < wall_threshold_;
}

bool GroundEstimation::isGroundValid(const std::vector<float> &plane_coeffs) const
{
    float cos_thresh = std::cos(max_angle_ * M_PI / 180.0);
    if (std::abs(plane_coeffs[2]) < cos_thresh) {
        std::cout << "\t[GroundEstimation] Rejected plane: normal z (" << plane_coeffs[2]
                  << ") < cos(max_angle) (" << cos_thresh << ")" << std::endl;
        return false;
    }
    if (std::abs(plane_coeffs[3]) > max_height_) {
        std::cout << "\t[GroundEstimation] Rejected plane: offset d (" << plane_coeffs[3]
                  << ") > max_height (" << max_height_ << ")" << std::endl;
        return false;
    }
    return true;
}



void GroundEstimation::saveToBuffer(const std::vector<float> &plane_coeffs)
{
    buffer_.push_back(plane_coeffs);
    if (buffer_.size() > buffer_size_)
    {
        buffer_.pop_front();
    }
}

bool GroundEstimation::getAverageFromBuffer(std::vector<float> &plane_coeffs)
{
    if (buffer_.empty())
    {
        return false;
    }
    std::vector<float> avg(4, 0.0f);
    for (const auto &p : buffer_)
    {
        for (size_t i = 0; i < 4; ++i)
        {
            avg[i] += p[i];
        }
    }
    for (auto &v : avg)
    {
        v /= buffer_.size();
    }
    plane_coeffs = avg;
    return true;
}
