# enhanced-RANSAC-ground-segmentation
An enhanced version ground segmentation program based on RANSAC algorithm


- key features:
- cuda acceleration, OpenMP speed up
- config: offset, iterration, ground, higher and lower points.
- preprocessing: multi threads filters.
  - voxel, box filter, octree?,
- Processing:
  - empty detect, moving average, kalman filter,
- Postprocessing:
  - wall filter, normal verification
- Record gif, pcl visualization.


gcc 12,
cuda 12.0
nvidia-driver 570.133.07


## data

KITTI data point cloud visualization, 3 scneoria: road + city + residential

https://www.cvlibs.net/datasets/kitti/raw_data.php

2011_09_26_drive_0101 (3.6 GB)
road
Length: 941 frames (01:34 minutes)
Image resolution: 1392 x 512 pixels

2011_09_29_drive_0071 (4.1 GB)
city
Length: 1065 frames (01:46 minutes)
Image resolution: 1392 x 512 pixels


2011_09_30_drive_0033 (6.2 GB)
residential
Length: 1600 frames (02:40 minutes)
Image resolution: 1392 x 512 pixels
