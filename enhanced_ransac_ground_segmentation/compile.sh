#!/bin/bash

# Stop script on any error
set -e

# Go to project root (optional if you always run from root)
cd "$(dirname "$0")"

# Clean build directory
rm -rf build
mkdir build && cd build

# Configure with CMake
cmake ..

# Compile using all available cores
make -j$(nproc)

# Run the program with the config
./enhanced_ransac_ground_segmentation ../config/enhanced_ransac.yaml

# Go back to root
cd ..
