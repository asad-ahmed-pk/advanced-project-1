// main.cpp
// Point Cloud Tree Classifier

#include <iostream>
#include <string>

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

    // print basic stats on point cloud file
    std::cout << "\n\nPoint Cloud Data Loaded";
    std::cout << "\nWidth: " << cloud->width;
    std::cout << "\nHeight: " << cloud->height;
    std::cout << "\nTotal Number of Points: " << cloud->width * cloud->height;
    std::cout << std::endl;

    // create new point cloud from the RGB vegetation indices
    std::cout << "Calculating Vegetation Indices" << std::endl;;
    //pcl::PointCloud<PointDefaultType>::Ptr result(new pcl::PointCloud<PointDefaultType>);
    Preprocessor preprocessor;
    preprocessor.ProcessPointCloud(cloud, VegetationIndex::TGI, cloud);

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
    std::cout << "\n\nVisualising Result Point Cloud" << std::endl;
    PointCloudRenderer renderer(cloud);
    renderer.Render();

    return 0;
}
