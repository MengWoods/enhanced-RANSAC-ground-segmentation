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
    buffer_size_ = config["ground_estimation"]["buffer_size"].as<int>(10);
    max_angle_ = config["ground_estimation"]["max_angle"].as<float>(10.0f);
    max_height_ = config["ground_estimation"]["max_height"].as<float>(0.2f);
    min_points_ = config["ground_estimation"]["min_points"].as<int>(50);
    wall_filter_enabled_ = config["ground_estimation"]["wall_filter"]["enable"].as<bool>(true);
    max_rerun_times_ = config["ground_estimation"]["wall_filter"]["max_rerun_times"].as<int>(3);
    wall_threshold_ = config["ground_estimation"]["wall_filter"]["threshold"].as<float>(0.2f);

    ransac_ = std::make_unique<Ransac>(config);
    wall_filter_ = std::make_unique<WallFilter>(wall_threshold_);

    std::cout << "[GroundEstimation] Initialized with buffer_size: " << buffer_size_ << ", max_angle: " << max_angle_ << ", max_height: " << max_height_ << ", min_points: " << min_points_ << "\n";

    std::cout << "[WallFilter] enabled: " << (wall_filter_enabled_ ? "true" : "false")
              << ", max_rerun_times: " << max_rerun_times_ << ", threshold: " << wall_threshold_ << "\n";
}

bool GroundEstimation::estimateGround(PointCloudPtr &cloud, std::vector<float> &plane_coeffs)
{
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
            return false;
        }
    }

    if (isGroundValid(plane_coeffs))
    {
        saveToBuffer(plane_coeffs);
    }
    else
    {
        if (!getAverageFromBuffer(plane_coeffs))
        {
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
    return std::abs(plane_coeffs[2]) < std::cos(max_angle_ * M_PI / 180.0) && std::abs(plane_coeffs[3]) < max_height_;
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
