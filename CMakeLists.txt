# CMake Main File

cmake_minimum_required(VERSION 3.10)
project(PointCloudTreeClassifier)

set(CMAKE_CXX_STANDARD 11)

# PCL
find_package(PCL 1.3 REQUIRED COMPONENTS common io visualization search filters segmentation keypoints features gpu)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Eigen
find_package (Eigen3 3.3 REQUIRED)
include_directories($ENV{EIGEN3_INCLUDE_DIR})

# Main exec sources
list(APPEND PROJECT_SOURCES
        src/main.cpp
        src/visualisation/point_cloud_renderer.cpp
        src/segmentation/segmenter.cpp
        src/classification/classifier.cpp
        src/pre_processing/preprocessor.cpp
        src/io/lasreader.cpp
)

# dependency - libLAS
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_subdirectory(libs/libLAS)
set(LIBLAS_LIB_PATH_MAC "${LIBLAS_BUILD_OUTPUT_DIRECTORY}/liblas.dylib" "${LIBLAS_BUILD_OUTPUT_DIRECTORY}/liblas_c.dylib")
set(LIBLAS_LIB_PATH_LINUX "${LIBLAS_BUILD_OUTPUT_DIRECTORY}/liblas.so" "${LIBLAS_BUILD_OUTPUT_DIRECTORY}/liblas_c.so")

# tools - downsampler
add_executable(downsample_las_cloud "src/tools/downsample_las_cloud.cpp" "src/pre_processing/preprocessor.cpp" "src/io/lasreader.cpp")

# main executable
add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})
if(APPLE)
    target_link_libraries(downsample_las_cloud ${PCL_LIBRARIES} ${LIBLAS_LIB_PATH_MAC})
    target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} Eigen3::Eigen ${LIBLAS_LIB_PATH_MAC})
else()
    target_link_libraries(downsample_las_cloud ${PCL_LIBRARIES} ${LIBLAS_LIB_PATH_LINUX})
    target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} Eigen3::Eigen ${LIBLAS_LIB_PATH_LINUX})
endif()
