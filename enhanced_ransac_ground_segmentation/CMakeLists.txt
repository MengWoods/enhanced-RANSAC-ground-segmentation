# Minimum CMake version
cmake_minimum_required(VERSION 3.10)

# Project name and version
project(enhanced_ransac_ground_segmentation)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add paths for headers and source files
include_directories(${CMAKE_SOURCE_DIR}/include)
file(GLOB SOURCES ${CMAKE_SOURCE_DIR}/src/*.cpp ${CMAKE_SOURCE_DIR}/src/*.cu)

# Find required libraries
find_package(OpenCV REQUIRED)           # For visualization
find_package(PCL REQUIRED)              # For point cloud processing
find_package(yaml-cpp REQUIRED)         # For YAML parsing
find_package(CUDA REQUIRED)             # For CUDA runtime
find_package(OpenMP REQUIRED)           # For parallel processing

# Include CUDA-specific CMake macros
include_directories(${CUDA_INCLUDE_DIRS})  # Add CUDA include directories

# Add the .cu file to the target's source files
cuda_add_executable(${PROJECT_NAME} ${SOURCES})

# Link required libraries
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    yaml-cpp
    ${CUDA_LIBRARIES}
    OpenMP::OpenMP_CXX
)
