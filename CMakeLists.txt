# CMake Main File

cmake_minimum_required(VERSION 3.10)
project(PointCloudTreeClassifier)

set(CMAKE_CXX_STANDARD 11)

# PCL
find_package(PCL 1.3 REQUIRED COMPONENTS common io visualization search filters segmentation keypoints features)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Eigen
find_package (Eigen3 3.3 REQUIRED)
include_directories($ENV{EIGEN3_INCLUDE_DIR})

# Sources
list(APPEND PROJECT_SOURCES
        src/main.cpp
        src/visualisation/point_cloud_renderer.cpp
        src/segmentation/segmenter.cpp
        src/classification/classifier.cpp
        src/pre_processing/preprocessor.cpp
)

add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} Eigen3::Eigen)
