/*
 * Enhanced RANSAC Ground Segmentation
 * Copyright (c) 2025 Menghao Woods
 *
 * Licensed under the MIT License. See LICENSE file in the project root for details.
 */
#include "ransac.h"
#include <cuda_runtime.h>
#include <chrono>

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
    // Declare timer variables outside the 'if' to be accessible throughout
    std::chrono::steady_clock::time_point start_total, start_step;
    if (timer_)
    {
        start_total = std::chrono::steady_clock::now();
        start_step = start_total;
    }

    int num_points = cloud->size();
    if (num_points < min_points_)
    {
        if (verbose_)
            std::cout << "[Ransac] Not enough points for RANSAC (" << num_points << " < " << min_points_ << ").\n";
        return;
    }

    // Allocate device memory
    Point *d_points;
    float *d_plane_coeffs; // Corrected type from Point* to float*
    cudaMalloc((void **)&d_points, num_points * sizeof(Point));
    cudaMalloc((void **)&d_plane_coeffs, 4 * sizeof(float));

    if (timer_)
    {
        cudaDeviceSynchronize(); // Wait for mallocs to complete
        auto t1 = std::chrono::steady_clock::now();
        std::cout << "[Ransac]   - Malloc duration: "
                  << std::chrono::duration_cast<std::chrono::microseconds>(t1 - start_step).count() << " us\n";
        start_step = t1; // Reset step timer
    }

    // Copy data to device
    cudaMemcpy(d_points, cloud->points.data(), num_points * sizeof(Point), cudaMemcpyHostToDevice);

    if (timer_)
    {
        cudaDeviceSynchronize(); // Wait for Host-to-Device copy
        auto t1 = std::chrono::steady_clock::now();
        std::cout << "[Ransac]   - H2D Copy duration: "
                  << std::chrono::duration_cast<std::chrono::microseconds>(t1 - start_step).count() << " us\n";
        start_step = t1;
    }

    // Run RANSAC on GPU
    ransacFitPlaneGPU(d_points, num_points, max_iterations_, distance_threshold_, d_plane_coeffs);

    if (timer_)
    {
        cudaDeviceSynchronize(); // Wait for GPU kernel to finish
        auto t1 = std::chrono::steady_clock::now();
        std::cout << "[Ransac]   - GPU Kernel duration: "
                  << std::chrono::duration_cast<std::chrono::microseconds>(t1 - start_step).count() << " us\n";
        start_step = t1;
    }

    // Copy back results
    plane_coeffs.resize(4);
    cudaMemcpy(plane_coeffs.data(), d_plane_coeffs, 4 * sizeof(float), cudaMemcpyDeviceToHost);

    if (timer_)
    {
        cudaDeviceSynchronize(); // Wait for Device-to-Host copy
        auto t1 = std::chrono::steady_clock::now();
        std::cout << "[Ransac]   - D2H Copy duration: "
                  << std::chrono::duration_cast<std::chrono::microseconds>(t1 - start_step).count() << " us\n";
    }

    // Free memory
    cudaFree(d_points);
    cudaFree(d_plane_coeffs);

    if (timer_)
    {
        cudaDeviceSynchronize(); // Wait for memory free to complete
        auto end_total = std::chrono::steady_clock::now();
        std::cout << "[Ransac] Total function duration: "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(end_total - start_total).count() << " ms\n";
    }
}
