// downsample_las_cloud.cpp
// Downsampling tool for downsampling and writing cloud to disk as PCD

#include <iostream>
#include <string>
#include <chrono>

#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"

#include "../type_defs.h"
#include "../pre_processing/preprocessor.h"
#include "../io/lasreader.h"

int main(int argc, char** argv)
{
    // command line args downsample_las_cloud [input las file] [output las file]
    if (argc < 3) {
        std::cout << "\nUsage: downsample_las_cloud [input las file] [output las file]";
        std::cout << std::endl;
        return 0;
    }

    // optional arg: thread number for las reader
    int numThreadsForReading = 4;
    if (argc == 4) {
        numThreadsForReading = atoi(argv[3]);
    }

    std::string inputFile { argv[1] };
    std::string outputFile { argv[2] };

    pcl::PointCloud<PointDefaultType>::Ptr cloud(new pcl::PointCloud<PointDefaultType>);

    // read in las point cloud
    std::cout << "\n\nReading LAS point cloud file..." << std::endl;

    LASReader lasReader(numThreadsForReading, cloud);
    lasReader.ReadLASPointCloud(inputFile);

    unsigned int originalPointCount = cloud->width * cloud->height;

    // print basic stats on point cloud file
    std::cout << "\n\nPoint Cloud Data Loaded";
    std::cout << "\nWidth: " << cloud->width;
    std::cout << "\nHeight: " << cloud->height;
    std::cout << "\nTotal Number of Points: " << originalPointCount;
    std::cout << std::endl;

    // down-sample point cloud
    std::cout << "\nDownsampling point cloud... \n";
    std::cout.flush();

    Preprocessor::DownsampleCloud(cloud);

    unsigned int downsampledCloudPointCount = cloud->width * cloud->height;

    std::cout << "Point cloud downsampled successfully";
    std::cout << "\nNumber of points after downsample: " << downsampledCloudPointCount;
    std::cout << " (" << (static_cast<double>(downsampledCloudPointCount) / static_cast<double>(originalPointCount)) * 100 << "% size)";
    std::cout << std::endl;

    // write downsampled cloud to disk
    std::cout << "\n\nSaving cloud to disk: " << outputFile << std::endl;
    pcl::io::savePCDFile(outputFile, *cloud, true);
    std::cout << "\nSuccess. File Saved" << std::endl;

    return 0;
}
