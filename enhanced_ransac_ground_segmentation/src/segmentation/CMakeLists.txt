add_library(segmentation
    ransac.cpp
    ransac.cu
    ground_estimation.cpp
)

# Include headers from your project and PCL
target_include_directories(segmentation
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${PCL_INCLUDE_DIRS}
        ${CUDA_INCLUDE_DIRS}
)

# Link PCL and CUDA
target_link_libraries(segmentation
    PUBLIC
        ${PCL_LIBRARIES}
        ${CUDA_LIBRARIES}
)

# Add paths for headers and source files
include_directories(${CMAKE_SOURCE_DIR}/include)
file(GLOB SOURCES ${CMAKE_SOURCE_DIR}/src/*.cpp ${CMAKE_SOURCE_DIR}/src/*.cu)

# PCL-related preprocessor defines (if any)
target_compile_definitions(segmentation PUBLIC ${PCL_DEFINITIONS})

# Enable CUDA support and C++17
target_compile_features(segmentation PUBLIC cxx_std_17)
