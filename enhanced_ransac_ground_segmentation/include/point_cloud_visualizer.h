/*
 * Enhanced RANSAC Ground Segmentation
 * Copyright (c) 2025 Menghao Woods
 *
 * Licensed under the MIT License. See LICENSE file in the project root for details.
 */
#pragma once
#include "constant.h"

#include <optional>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <pcl/visualization/pcl_visualizer.h>

struct VisualizerData
{
    PointCloud::Ptr cloud;
    std::vector<float> plane_coeffs;
};

class PointCloudVisualizer
{
public:
    /**
     * @brief Default constructor initializes the PCL visualizer.
     * @param config YAML configuration node containing visualizer settings.
     */
    PointCloudVisualizer(const YAML::Node &config);

    /**
     * @brief Destructor that ensures visualizer resources are released.
     */
    ~PointCloudVisualizer();

    /**
     * @brief Starts the viewer thread to handle visualization updates.
     */
    void pushFrame(PointCloud::Ptr cloud, std::vector<float> plane_coeffs);

    /**
     * @brief Pushes a new point cloud frame to the visualizer.
     * @param cloud The point cloud to visualize.
     */
    void pushFrame(PointCloud::Ptr cloud);

private:
    pcl::visualization::PCLVisualizer::Ptr viewer_; ///< PCL visualizer instance
    float camera_distance_;                         ///< Distance from the origin (camera to object)
    float angle_;                                   ///< Camera rotation angle in degrees around y-axis (horizontal rotation)
    int refresh_interval_;                          ///< Time interval in ms for refreshing the visualization

    /* ---------- threading primitives ---------- */
    std::thread thread_;                        ///< Thread for running the visualizer loop
    std::mutex mtx_;                            ///< Mutex for synchronizing access to shared data
    std::condition_variable cv_;                ///< Condition variable for signaling updates
    std::optional<VisualizerData> latest_data_; ///< Latest data to visualize
    bool shutdown_{false};                      ///< Flag to indicate if the viewer thread should shut down


    /**
     * @brief Initializes the visualizer, adds coordinate system and sets camera parameters.
     */
    void initViewer();

    /**
     * @brief Main loop for the viewer thread, processes visualization updates.
     */
    void runViewerLoop();

    /**
     * @brief Starts the viewer thread.
     */
    void startViewerThread();

    /**
     * @brief Stops the viewer thread and cleans up resources.
     */
    void stopViewerThread();

    /**
     * @brief Splits the point cloud into two parts based on the ground plane coefficients.
     *        Points above the plane are considered as non-ground, and points below are considered as ground.
     * @param cloud The point cloud to split.
     * @param plane_coeffs Coefficients of the ground plane (a, b, c, d).
     * @param threshold Distance threshold to consider a point as above or below the plane.
     * @return A pair of point clouds: first is above the plane, second is below the plane.
     */
    std::pair<PointCloud::Ptr, PointCloud::Ptr> splitCloudByPlane(
        const PointCloud::Ptr& cloud,
        const std::vector<float>& plane_coeffs,
        float threshold = 0.1f);
};
