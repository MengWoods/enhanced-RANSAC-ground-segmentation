/*
 * Enhanced RANSAC Ground Segmentation
 * Copyright (c) 2025 Menghao Woods
 *
 * Licensed under the MIT License. See LICENSE file in the project root for details.
 */
#pragma once
#include "constant.h"

#include "ransac.h"
#include "wall_filter.h"
#include <memory>
#include <deque>
#include <Eigen/Dense>

/**
 * @class GroundEstimation
 * @brief Estimates the ground plane using RANSAC with optional wall filtering.
 */
class GroundEstimation
{
public:
    /**
     * @brief Constructor initializes parameters from YAML config.
     * @param config YAML node containing configuration parameters.
     */
    explicit GroundEstimation(const YAML::Node &config);
    
    /**
     * @brief Estimates the ground plane from the input cloud.
     * @param cloud Input point cloud.
     * @param plane_coeffs Output plane coefficients.
     * @return True if estimation is successful, false otherwise.
     */
    bool estimateGround(PointCloudPtr &cloud, std::vector<float> &plane_coeffs);

private:
    /**
     * @brief Retrieves the average plane from the history buffer.
     * @param plane_coeffs Output average plane coefficients.
     * @return True if buffer has data, false otherwise.
     */
    bool getAverageFromBuffer(std::vector<float> &plane_coeffs);
    
    /**
     * @brief Saves the estimated ground plane to the buffer.
     * @param plane_coeffs Plane coefficients to store.
     */
    void saveToBuffer(const std::vector<float> &plane_coeffs);
    
    /**
     * @brief Checks if the estimated ground plane meets validity conditions.
     * @param plane_coeffs Plane coefficients to check.
     * @return True if valid, false otherwise.
     */
    bool isGroundValid(const std::vector<float> &plane_coeffs) const;
    
    /**
     * @brief Determines if the detected plane resembles a wall.
     * @param plane_coeffs Plane coefficients to check.
     * @return True if it resembles a wall, false otherwise.
     */
    bool isWallLike(const std::vector<float> &plane_coeffs) const;
    
    /**
     * @brief Ensures the ground plane normal points upwards.
     * @param plane_coeffs Plane coefficients to adjust if necessary.
     */
    void flipPlaneIfNecessary(std::vector<float> &plane_coeffs);

    std::deque<std::vector<float>> buffer_; ///< Buffer storing recent ground plane estimates.
    int buffer_size_; ///< Maximum size of the history buffer.
    float max_angle_; ///< Maximum allowable ground plane angle.
    float max_height_; ///< Maximum allowable ground plane height.
    int min_points_; ///< Minimum number of points required for estimation.
    bool wall_filter_enabled_; ///< Flag indicating if wall filtering is enabled.
    int max_rerun_times_; ///< Maximum number of RANSAC retries for wall filtering.
    float wall_threshold_; ///< Threshold for determining if a plane resembles a wall.

    std::unique_ptr<Ransac> ransac_; ///< RANSAC-based plane estimation.
    std::unique_ptr<WallFilter> wall_filter_; ///< Wall filter for refining the ground estimate.
};