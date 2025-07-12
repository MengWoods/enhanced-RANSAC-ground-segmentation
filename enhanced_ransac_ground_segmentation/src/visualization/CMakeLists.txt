add_library(visualization
    point_cloud_visualizer.cpp
)

target_include_directories(visualization
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${PCL_INCLUDE_DIRS}
)

target_link_libraries(visualization
    PUBLIC ${PCL_LIBRARIES}
)

target_compile_definitions(visualization PUBLIC ${PCL_DEFINITIONS})
target_compile_features(visualization PUBLIC cxx_std_17)
