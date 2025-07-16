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
 * @brief Kernel to compute RANSAC plane fitting in parallel.
 * It uses a single block where threads collaborate.
 */
__global__ void ransacParallelKernel(const Point *points, int num_points, int max_iterations,
                                     float distance_threshold, float *best_plane_out)
{
    // Best model found so far, stored in shared memory, visible to all threads in the block.
    __shared__ float best_model[4];
    __shared__ int best_inlier_count;

    // Shared memory for the current candidate plane.
    __shared__ float current_model[4];

    // Shared memory for the parallel reduction (summing inlier counts).
    __shared__ int inlier_counts_shared[THREADS_PER_BLOCK];

    // Initialize shared variables with the first thread.
    if (threadIdx.x == 0) {
        best_inlier_count = 0;
    }
    __syncthreads(); // Ensure initialization is complete before proceeding.

    // Each thread needs its own random state.
    curandState state;
    curand_init(clock64() + threadIdx.x, 0, 0, &state);

    // --- Main RANSAC Loop ---
    for (int iter = 0; iter < max_iterations; ++iter)
    {
        // Step 1: Let thread 0 pick 3 random points and compute a candidate plane.
        if (threadIdx.x == 0)
        {
            int idx1 = curand(&state) % num_points;
            int idx2 = curand(&state) % num_points;
            int idx3 = curand(&state) % num_points;

            // A simple check to avoid degenerate planes. A more robust check might be needed.
            if (idx1 == idx2 || idx2 == idx3 || idx1 == idx3) {
                // Set a flag or a dummy plane to skip this iteration
                current_model[0] = 2.0f; // Use an invalid norm to signal a skip
            }
            else
            {
                Point p1 = points[idx1];
                Point p2 = points[idx2];
                Point p3 = points[idx3];

                float ux = p2.x - p1.x, uy = p2.y - p1.y, uz = p2.z - p1.z;
                float vx = p3.x - p1.x, vy = p3.y - p1.y, vz = p3.z - p1.z;

                float a = uy * vz - uz * vy;
                float b = uz * vx - ux * vz;
                float c = ux * vy - uy * vx;
                float d = -(a * p1.x + b * p1.y + c * p1.z);

                float norm = sqrtf(a * a + b * b + c * c);
                if (norm > 1e-6)
                { // Avoid division by zero
                    current_model[0] = a / norm;
                    current_model[1] = b / norm;
                    current_model[2] = c / norm;
                    current_model[3] = d / norm;
                }
                else
                {
                    current_model[0] = 2.0f; // Invalid norm
                }
            }
        }
        __syncthreads(); // Wait for thread 0 to finish and broadcast the plane to all threads.

        // If the plane was invalid, all threads skip to the next iteration.
        if (current_model[0] > 1.0f)
        {
            continue;
        }

        // Step 2: Parallel Inlier Counting
        // Each thread counts inliers for its subset of the data.
        int local_inlier_count = 0;
        for (int j = threadIdx.x; j < num_points; j += blockDim.x)
        {
            float dist = fabsf(current_model[0] * points[j].x +
                               current_model[1] * points[j].y +
                               current_model[2] * points[j].z +
                               current_model[3]);
            if (dist < distance_threshold)
            {
                local_inlier_count++;
            }
        }
        inlier_counts_shared[threadIdx.x] = local_inlier_count;
        __syncthreads(); // Ensure all threads have finished counting.

        // Step 3: Parallel Reduction (Sum)
        // Combine the local counts into a single total.
        for (int s = blockDim.x / 2; s > 0; s >>= 1)
        {
            if (threadIdx.x < s)
            {
                inlier_counts_shared[threadIdx.x] += inlier_counts_shared[threadIdx.x + s];
            }
            __syncthreads();
        }
        // The final total is now in inlier_counts_shared[0].

        // Step 4: Update Best Model
        // Let thread 0 check if this model is the new best.
        if (threadIdx.x == 0)
        {
            if (inlier_counts_shared[0] > best_inlier_count)
            {
                best_inlier_count = inlier_counts_shared[0];
                best_model[0] = current_model[0];
                best_model[1] = current_model[1];
                best_model[2] = current_model[2];
                best_model[3] = current_model[3];
            }
        }
        __syncthreads(); // Ensure the best model is updated before the next iteration.
    }

    // Step 5: Write Final Result
    // Thread 0 writes the best model found to global memory.
    if (threadIdx.x == 0)
    {
        best_plane_out[0] = best_model[0];
        best_plane_out[1] = best_model[1];
        best_plane_out[2] = best_model[2];
        best_plane_out[3] = best_model[3];
    }
}

/**
 * @brief Wrapper function to launch the parallel CUDA kernel.
 */
void ransacFitPlaneGPU(const Point *d_points, int num_points, int max_iterations,
                       float distance_threshold, float *d_plane_coeffs)
{
    // Launch a SINGLE block of threads to work collaboratively.
    ransacParallelKernel<<<1, THREADS_PER_BLOCK>>>(d_points, num_points, max_iterations,
                                                  distance_threshold, d_plane_coeffs);

    // Ensure the kernel is finished before the CPU proceeds, especially for timing.
    cudaDeviceSynchronize();
}
