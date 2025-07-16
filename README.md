# Enhanced RANSAC Ground Segmentation

This repository contains an enhanced, real-time ground segmentation algorithm for LiDAR point clouds.

The core of the project is a highly optimized RANSAC-based plane estimator that leverages CUDA for significant acceleration. It is designed to be a robust and performant solution for robotics and autonomous vehicle applications.

## Demo

<p align="center">
  <img src="./resource/Peek%202025-07-16%2011-24.gif" width="60%">
</p>

The full video below showcases the algorithm's performance on three distinct scenes from the KITTI dataset, demonstrating its stability and accuracy in highway, city, and residential environments.

[![RANSAC Ground Segmentation V2 Demo](https://img.youtube.com/vi/Z5uDPQNM0xA/1.jpg)](https://www.youtube.com/watch?v=Z5uDPQNM0xA)
*(Click the image to watch on YouTube)*

## Key Features

- **CUDA-Accelerated RANSAC:** The RANSAC plane estimation is implemented in CUDA, using a parallel-collaborative kernel design for massive speedups. Processing time per frame is consistently under 10ms.
- **Configurable Pre-processing:** Includes filters like VoxelGrid, Box, and Statistical Outlier Removal to clean the point cloud before processing.
- **Corner Case Handling:** Includes logic to mitigate common real-world issues, such as the influence of vertical walls on plane estimation, based on empirical testing.
- **Real-time Visualization:** Uses the Point Cloud Library (PCL) for live 3D visualization of the segmented ground (green) and non-ground (white) points.
- **Stable Frequency Control:** The main processing loop is rate-limited, ensuring a consistent output frequency suitable for real-time systems.
- **Detailed Performance Logging:** Optional timers provide a frame-by-frame breakdown of the duration of each processing step for easy profiling.


## Requirements

### Hardware (Tested On)
- **OS:** Ubuntu 24.04
- **GPU:** NVIDIA GPU compatible with CUDA 12.0+
- **NVIDIA Driver:** 570.133.07

### Dependencies
- **GCC:** 12.x (or any compiler with C++17 support)
- **CUDA Toolkit:** 12.0 or newer
- **CMake:** 3.16 or newer
- **Point Cloud Library (PCL):** 1.10 or newer
- **YAML-CPP:** For parsing the configuration file.


## Performance

Here is a sample performance log from a typical run:

```bash
------------------- Frame -------------------
[Timer] Box Filter: 5 ms
[Timer] Voxel Filter: 12 ms
[Timer] Noise Filter: 0 ms
[Timer] Ground Estimation: 12 ms
[Timer] Visualization: 0 ms
[Timer] Total Frame Time: 66 ms
```
*Note: The `Total Frame Time` is controlled by a frequency setting in the configuration. The application will pause to maintain a consistent rate (e.g., 15 Hz or 66ms/frame), which is why the total time may be longer than the sum of the individual processing steps.*


## Getting Started

### 0. Prepare Data

The project was tested using the raw datasets from KITTI. You can download them from the [official website](https://www.cvlibs.net/datasets/kitti/raw_data.php).

The specific sequences used in the demo video are:
- Road Scene: 2011_09_26_drive_0101
- City Scene: 2011_09_29_drive_0071
- Residential Scene: 2011_09_30_drive_0033

Unzip the data and place the sequence folders in a known directory.

### 1. Clone the Repository

```bash
git clone git@github.com:MengWoods/enhanced-RANSAC-ground-segmentation.git
```

### 2. Configure the Program

Edit the [`enhanced_ransac_ground_segmentation/config/config.yaml`](./enhanced_ransac_ground_segmentation/config/enhanced_ransac.yaml) file to set the `point_cloud_paths` to the correct directory where your KITTI sequence is located. You can also adjust filter parameters, RANSAC iterations, and other settings here.


### 3. Run the Project

```bash
# Go to the folder
cd enhanced-RANSAC-ground-segmentation/enhanced_ransac_ground_segmentation/
# Run the script
bash compile.sh
```

## Project Roadmap (TODO)

This project is under active development. Future enhancements include:
- [ ] Performance: Accelerate noise filters using OpenMP.
- [ ] Segmentation Logic:
  - [ ] Add a third category for "negative" points (below the ground plane).
  - [ ] Improve handling of corner cases: empty point clouds, fully occluded views, and steep slopes.
  - [ ] Mitigate the effect of vertical walls on plane estimation.
- [ ] Estimation Algorithm:
  - [ ] Implement a predictive filter (e.g., Kalman Filter) for plane coefficients to improve temporal stability.
- [ ] Advanced Features:
  - [ ] Introduce a probability model where points closer to the sensor have a higher weight in the estimation.
  - [ ] Visualize this probability.
