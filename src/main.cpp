// main.cpp
// Point Cloud Tree Classifier

#include <iostream>
#include <string>
#include <chrono>

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/common/io.h"
#include "pcl/search/search.h"
#include "pcl/search/kdtree.h"
#include "pcl/filters/passthrough.h"
#include "pcl/keypoints/sift_keypoint.h"
#include "pcl/keypoints/impl/sift_keypoint.hpp"
#include "pcl/features/normal_3d.h"

#include "Eigen/Core"

#include "type_defs.h"
#include "io/lasreader.h"
#include "visualisation/point_cloud_renderer.h"
#include "classification/classifier.h"
#include "pre_processing/preprocessor.h"
#include "segmentation/segmenter.h"

const std::string FILE_PATH { "../../data/jacobs_campus.pcd" };
const std::string SEGMENT_CLOUD_FILE_PATH { "../../data/segmented_cloud.pcd" };

int main(int argc, char** argv)
{
    // begin recording execution time
    std::chrono::high_resolution_clock::time_point startTime = std::chrono::high_resolution_clock::now();

    // check for params
    if (argc < 2) {
        std::cerr << "\nUsage: PointCloudTreeClassifier [input point cloud pcd file] [optional: number of threads for reading las file]\n";
        std::cerr.flush();
        return 0;
    }

    // setup file paths
    const std::string inputPointCloudFilePath { argv[1] };

    // optional param: number of threads to read in the las file
    int numThreadsForLasReader = 4;
    if (argc == 3) {
        numThreadsForLasReader = atoi(argv[2]);
    }

    // read in point cloud data from file
    pcl::PointCloud<PointDefaultType>::Ptr cloud(new pcl::PointCloud<PointDefaultType>);
    /*
    if (pcl::io::loadPCDFile<PointDefaultType>(inputPointCloudFilePath, *cloud) == -1) {
        PCL_ERROR("Could not read point cloud file");
        return -1;
    }
    */

    // read in las point cloud
    std::cout << "\nReading LAS point cloud file..." << std::endl;

    LASReader lasReader(numThreadsForLasReader, cloud);
    lasReader.ReadLASPointCloud(inputPointCloudFilePath);

    unsigned int originalPointCount = cloud->width * cloud->height;

    // print basic stats on point cloud file
    std::cout << "\n\nPoint Cloud Data Loaded";
    std::cout << "\nWidth: " << cloud->width;
    std::cout << "\nHeight: " << cloud->height;
    std::cout << "\nTotal Number of Points: " << originalPointCount;
    std::cout << std::endl;

    // create new point cloud from the RGB vegetation indices
    /*
    std::cout << "Calculating Vegetation Indices" << std::endl;;
    //pcl::PointCloud<PointDefaultType>::Ptr result(new pcl::PointCloud<PointDefaultType>);
    Preprocessor preprocessor;
    preprocessor.ProcessPointCloud(cloud, VegetationIndex::TGI, cloud);
    */

    // down-sample point cloud
    std::cout << "\nDownsampling point cloud... \n";
    std::cout.flush();

    Preprocessor::DownsampleCloud(cloud);

    unsigned int downsampledCloudPointCount = cloud->width * cloud->height;

    std::cout << "Point cloud downsampled successfully";
    std::cout << "\nNumber of points after downsample: " << downsampledCloudPointCount;
    std::cout << " (" << (static_cast<double>(downsampledCloudPointCount) / static_cast<double>(originalPointCount)) * 100 << "% size)";
    std::cout << std::endl;


    // segment the cloud
    Segmenter segmenter;
    pcl::PointCloud<PointDefaultType>::Ptr segmentedCloud(new pcl::PointCloud<PointDefaultType>);
    std::cout << "Segmenting Point Cloud" << std::endl;
    segmenter.SegementPointCloudByColor(cloud, cloud);

    // Write the segmented cloud to disk
    std::cout << "\nWriting segmented cloud to disk...";
    pcl::io::savePCDFile(SEGMENT_CLOUD_FILE_PATH, *cloud);
    std::cout << "\nSegmented cloud written to disk";
    std::cout.flush();

    // visualize the resultant cloud
    /*
    std::cout << "\n\nVisualising Result Point Cloud" << std::endl;
    PointCloudRenderer renderer(cloud);
    renderer.Render();
    */

    // record end execution time and elapsed time
    std::chrono::high_resolution_clock::time_point endTime = std::chrono::high_resolution_clock::now();

    std::cout << "\n\nProcessing " << inputPointCloudFilePath << " complete" << std::endl;
    std::cout << "Final point cloud written to: " << SEGMENT_CLOUD_FILE_PATH << std::endl;
    std::cout << "\n\nTotal Processing Time: " << std::chrono::duration_cast<std::chrono::minutes>(endTime - startTime).count() << " minutes" << std::endl;
    std::cout << std::endl;

    return 0;
}
