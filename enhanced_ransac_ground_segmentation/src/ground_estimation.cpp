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
    verbose_ = config["ground_estimation"]["verbose"].as<bool>(false);
    max_angle_ = config["ground_estimation"]["max_angle"].as<float>(10.0f);
    max_height_ = config["ground_estimation"]["max_height"].as<float>(0.2f);
    min_points_ = config["ground_estimation"]["min_points"].as<int>(50);
    z_offset_ = config["ground_estimation"]["z_offset"].as<float>(0.1f);
    temporal_filter_enabled_ = config["ground_estimation"]["temporal_filter"]["enable"].as<bool>(true);
    wall_filter_enabled_ = config["ground_estimation"]["wall_filter"]["enable"].as<bool>(true);
    max_rerun_times_ = config["ground_estimation"]["wall_filter"]["max_rerun_times"].as<int>(3);
    wall_threshold_ = config["ground_estimation"]["wall_filter"]["threshold"].as<float>(0.2f);

    if (temporal_filter_enabled_)
    {
        temporal_filter_method_ = config["ground_estimation"]["temporal_filter"]["method"].as<std::string>("kalman_filter");
        if (temporal_filter_method_ == "moving_average")
        {
            buffer_size_ = config["ground_estimation"]["temporal_filter"]["moving_average"]["buffer_size"].as<int>(10);
        }
        else if (temporal_filter_method_ == "kalman_filter")
        {
            kalman_filter_ = std::make_unique<KalmanFilter>();
            kalman_process_noise_ = config["ground_estimation"]["temporal_filter"]["kalman_filter"]["process_noise"].as<double>(0.01);
            kalman_measurement_noise_ = config["ground_estimation"]["temporal_filter"]["kalman_filter"]["measurement_noise"].as<double>(0.1);
            kalman_initial_covariance_ = config["ground_estimation"]["temporal_filter"]["kalman_filter"]["initial_covariance"].as<double>(1.0);
            kalman_filter_->init({0.0f, 0.0f, 1.0f, 0.0f}, kalman_initial_covariance_, kalman_process_noise_, kalman_measurement_noise_);
        }
        else
        {
            throw std::runtime_error("Unknown temporal filter method: " + temporal_filter_method_);
        }
    }

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
        std::cout << "  - Temporal Filter Enabled: " << (temporal_filter_enabled_ ? "true" : "false") << std::endl;
        std::cout << "  - Temporal Filter Method: " << temporal_filter_method_ << std::endl;
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
    if (!enable_) { return false; }

    // --- Initial Check: Not enough points ---
    if (cloud->size() < min_points_)
    {
        // Fallback logic: Try to get a filtered estimate if the cloud is too small.
        if (temporal_filter_method_ == "kalman_filter")
        {
            if (kalman_filter_->isInitialized())
            {
                plane_coeffs = kalman_filter_->getState();
                return true;
            }
        }
        else // Default to moving_average
        {
            if (getAverageFromBuffer(plane_coeffs)) {
                return true;
            }
        }

        // If no filter is available, fail.
        std::cerr << "[GroundEstimation] Warning: Not enough points and no history available." << std::endl;
        return false;
    }

    // --- RANSAC Estimation Loop ---
    int rerun_count = 0;
    bool valid_ransac_result = false;
    std::vector<float> instant_coeffs; // Use a temporary vector for the raw RANSAC result

    while (!valid_ransac_result && rerun_count <= max_rerun_times_)
    {
        ransac_->estimatePlane(cloud, instant_coeffs);
        flipPlaneIfNecessary(instant_coeffs);

        if (wall_filter_enabled_ && isWallLike(instant_coeffs))
        {
            wall_filter_->applyFilter(cloud, instant_coeffs);
            rerun_count++;
        }
        else
        {
            valid_ransac_result = true;
        }
    }

    // --- Process the RANSAC Result ---
    if (valid_ransac_result && isGroundValid(instant_coeffs))
    {
        // --- Case 1: RANSAC succeeded and the plane is valid ---
        instant_coeffs[3] -= instant_coeffs[2] * z_offset_;

        if (temporal_filter_method_ == "kalman_filter")
        {
            if (!kalman_filter_->isInitialized())
            {
                // Initialize the filter with the first valid measurement
                kalman_filter_->init(instant_coeffs, kalman_initial_covariance_, kalman_process_noise_, kalman_measurement_noise_);
            }
            else
            {
                // Predict the next state, then update with the new measurement
                kalman_filter_->predict();
                kalman_filter_->update(instant_coeffs);
            }
            plane_coeffs = kalman_filter_->getState();
        }
        else // Moving Average
        {
            saveToBuffer(instant_coeffs);
            // plane_coeffs = instant_coeffs;
            getAverageFromBuffer(plane_coeffs);
        }
    }
    else
    {
        // --- Case 2: RANSAC failed or produced an invalid plane ---
        // Fallback to the last known good estimate from the chosen filter.
        if (temporal_filter_method_ == "kalman_filter")
        {
            if (kalman_filter_->isInitialized())
            {
                // Predict the next state without an update to keep it smooth
                kalman_filter_->predict();
                plane_coeffs = kalman_filter_->getState();
            }
            else
            {
                std::cerr << "\t[GroundEstimation] Warning: RANSAC failed and Kalman filter is not initialized." << std::endl;
                return false;
            }
        }
        else // Moving Average
        {
            if (!getAverageFromBuffer(plane_coeffs))
            {
                std::cerr << "\t[GroundEstimation] Warning: RANSAC failed and buffer is empty." << std::endl;
                return false;
            }
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

    if (verbose_ && buffer_.size() < buffer_size_)
    {
        std::cout << "\t[GroundEstimation] Buffer is not full ("
                  << buffer_.size() << "/" << buffer_size_
                  << "). Averaged result may be less stable." << std::endl;
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
