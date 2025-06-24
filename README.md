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
