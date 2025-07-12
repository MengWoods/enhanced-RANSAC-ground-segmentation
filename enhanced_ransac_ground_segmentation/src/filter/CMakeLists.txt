add_library(filter
    box_filter.cpp
    voxel_filter.cpp
    noise_filter.cpp
    wall_filter.cpp
)

target_include_directories(filter
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${PCL_INCLUDE_DIRS}
)

target_link_libraries(filter
    PUBLIC ${PCL_LIBRARIES}
)

target_compile_definitions(filter PUBLIC ${PCL_DEFINITIONS})
target_compile_features(filter PUBLIC cxx_std_17)
