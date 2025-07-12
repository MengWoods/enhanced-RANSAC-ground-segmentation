add_library(io
    point_cloud_loader.cpp
)

target_include_directories(io
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

target_compile_features(io PUBLIC cxx_std_17)
