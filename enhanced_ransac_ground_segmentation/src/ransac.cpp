/*
 * Enhanced RANSAC Ground Segmentation
 * Copyright (c) 2025 Menghao Woods
 *
 * Licensed under the MIT License. See LICENSE file in the project root for details.
 */
#include "ransac.h"
#include <cuda_runtime.h>

// CUDA function declaration
void ransacFitPlaneGPU(const Point *d_points, int num_points, int max_iterations, 
                       float distance_threshold, float *d_plane_coeffs);

Ransac::Ransac(const YAML::Node &config)
{
    distance_threshold_ = config["ransac"]["distance_threshold"].as<float>(0.2f);
    max_iterations_ = config["ransac"]["max_iterations"].as<int>(1000);
    min_points_ = config["ransac"]["min_points"].as<int>(50);
    verbose_ = config["verbose"].as<bool>(false);
    timer_ = config["ransac"]["timer"].as<bool>(false);

    std::cout << "[Ransac] Loaded parameters:" << std::endl;
    std::cout << "  - Distance Threshold: " << distance_threshold_ << std::endl;
    std::cout << "  - Max Iterations: " << max_iterations_ << std::endl;
    std::cout << "  - Min Points: " << min_points_ << std::endl;
    std::cout << "  - Verbose: " << (verbose_ ? "Enabled" : "Disabled") << std::endl;
    std::cout << "  - Timer: " << (timer_ ? "Enabled" : "Disabled") << std::endl;
}

void Ransac::estimatePlane(const PointCloudPtr &cloud, std::vector<float> &plane_coeffs)
{
    int num_points = cloud->size();

    if (num_points < min_points_)
    {
        if (verbose_)
            std::cout << "[Ransac] Not enough points for RANSAC (" << num_points << " < " << min_points_ << ").\n";
        return;
    }

    // Allocate device memory
    Point *d_points;
    cudaMalloc((void **)&d_points, num_points * sizeof(Point));
    cudaMemcpy(d_points, cloud->points.data(), num_points * sizeof(Point), cudaMemcpyHostToDevice);

    // Allocate memory for output coefficients
    float *d_plane_coeffs;
    cudaMalloc((void **)&d_plane_coeffs, 4 * sizeof(float));

    // Run RANSAC on GPU
    ransacFitPlaneGPU(d_points, num_points, max_iterations_, distance_threshold_, d_plane_coeffs);

    // Copy back results
    plane_coeffs.resize(4);
    cudaMemcpy(plane_coeffs.data(), d_plane_coeffs, 4 * sizeof(float), cudaMemcpyDeviceToHost);

    // Free memory
    cudaFree(d_points);
    cudaFree(d_plane_coeffs);
}
