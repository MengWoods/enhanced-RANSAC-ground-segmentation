rm -rf build
mkdir build && cd build
cmake ..
make -j$(nproc)
./enhanced_ransac_ground_segmentation /home/m/Documents/github/enhanced-RANSAC-ground-segmentation/enhanced_ransac_ground_segmentation/config/enhanced_ransac.yaml
cd ..