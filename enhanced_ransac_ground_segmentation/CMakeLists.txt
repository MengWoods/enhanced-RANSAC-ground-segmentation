# Minimum CMake version
cmake_minimum_required(VERSION 3.14)

# Project name and version
# project(enhanced_ransac_ground_segmentation)
project(enhanced_ransac_ground_segmentation LANGUAGES CXX CUDA)
set(CMAKE_CUDA_ARCHITECTURES 75)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set CUDA standard
# set(CMAKE_CUDA_ARCHITECTURES 75)
# cmake_policy(SET CMP0104 NEW)

# Required packages
find_package(MPI REQUIRED)
find_package(OpenCV REQUIRED)           # For visualization
find_package(PCL REQUIRED)              # For point cloud processing
find_package(yaml-cpp REQUIRED)         # For YAML parsing
find_package(CUDA REQUIRED)             # For CUDA runtime

# set(PCL_COMPONENTS common io filters visualization)
# find_package(PCL REQUIRED COMPONENTS ${PCL_COMPONENTS})

# Add include directories
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${CUDA_INCLUDE_DIRS})  # For CUDA headers
include_directories(${PCL_INCLUDE_DIRS})

# Add subdirectories (modular structure)
add_subdirectory(src/io)
add_subdirectory(src/filter)
add_subdirectory(src/segmentation)
add_subdirectory(src/visualization)
add_subdirectory(src/app)

# Create the executable from main.cpp
add_executable(${PROJECT_NAME}
    src/app/main.cpp
)

# Link internal modules
target_link_libraries(${PROJECT_NAME}
    PRIVATE
        io
        filter
        segmentation
        visualization
)

# Link external dependencies
target_link_libraries(${PROJECT_NAME}
    PRIVATE
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        yaml-cpp
        ${CUDA_LIBRARIES}
)


# Enable C++17 for the main target
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
