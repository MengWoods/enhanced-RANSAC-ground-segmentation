/*
 * Enhanced RANSAC Ground Segmentation
 * Copyright (c) 2025 Menghao Woods
 *
 * Licensed under the MIT License. See LICENSE file in the project root for details.
 */
#include <cuda_runtime.h>
#include <curand_kernel.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <cmath>

#define THREADS_PER_BLOCK 256

using Point = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<pcl::PointXYZI>;
using PointCloudPtr = pcl::PointCloud<pcl::PointXYZI>::Ptr;

/**
 * @brief Kernel to compute RANSAC plane fitting.
 */
__global__ void ransacKernel(const Point *points, int num_points, int max_iterations,
                             float distance_threshold, float *best_plane)
{
    __shared__ float best_model[4];
    int tid = threadIdx.x + blockIdx.x * blockDim.x;

    // Random seed
    curandState state;
    curand_init(clock64(), tid, 0, &state);

    float best_inlier_count = 0;

    for (int i = 0; i < max_iterations; ++i)
    {
        // Randomly select 3 points
        int idx1 = curand(&state) % num_points;
        int idx2 = curand(&state) % num_points;
        int idx3 = curand(&state) % num_points;

        // Avoid duplicate points
        if (idx1 == idx2 || idx2 == idx3 || idx1 == idx3)
            continue;

        Point p1 = points[idx1];
        Point p2 = points[idx2];
        Point p3 = points[idx3];

        // Compute plane normal
        float ux = p2.x - p1.x, uy = p2.y - p1.y, uz = p2.z - p1.z;
        float vx = p3.x - p1.x, vy = p3.y - p1.y, vz = p3.z - p1.z;

        float a = uy * vz - uz * vy;
        float b = uz * vx - ux * vz;
        float c = ux * vy - uy * vx;
        float d = -(a * p1.x + b * p1.y + c * p1.z);

        // Normalize
        float norm = sqrt(a * a + b * b + c * c);
        a /= norm;
        b /= norm;
        c /= norm;
        d /= norm;

        // Count inliers
        int inlier_count = 0;
        for (int j = 0; j < num_points; ++j)
        {
            float dist = fabs(a * points[j].x + b * points[j].y + c * points[j].z + d);
            if (dist < distance_threshold)
                inlier_count++;
        }

        // Update best model
        if (inlier_count > best_inlier_count)
        {
            best_inlier_count = inlier_count;
            best_model[0] = a;
            best_model[1] = b;
            best_model[2] = c;
            best_model[3] = d;
        }
    }

    // Save best model
    if (tid == 0)
    {
        for (int i = 0; i < 4; ++i)
            best_plane[i] = best_model[i];
    }
}

/**
 * @brief Wrapper function to launch CUDA kernel.
 */
void ransacFitPlaneGPU(const Point *d_points, int num_points, int max_iterations,
                       float distance_threshold, float *d_plane_coeffs)
{
    int num_blocks = (num_points + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;

    ransacKernel<<<num_blocks, THREADS_PER_BLOCK>>>(d_points, num_points, max_iterations, distance_threshold, d_plane_coeffs);

    cudaDeviceSynchronize();
}
