// lasreader.h

// IO class for reading in LAS files

#ifndef LASREADER_H
#define LASREADER_H

#include <string>
#include <mutex>

#include "liblas/liblas.hpp"
#include "pcl/point_cloud.h"
#include "../type_defs.h"

class LASReader
{
public:
    // Constructor
    LASReader(int numThreads, pcl::PointCloud<PointDefaultType>::Ptr cloud);

    // Read in las file into point cloud concurrently
    bool ReadLASPointCloud(const std::string& lasFilePath);

private:
    void ConvertToPCDPoint(const liblas::Point& lasPoint, PointDefaultType& pclPoint);
    void ProcessFileTask(unsigned long int start, unsigned long int end, const char* filePath);

private:
    int m_NumberOfThreads;
    std::mutex m_Mutex;
    pcl::PointCloud<PointDefaultType>::Ptr m_PointCloud;
};

#endif // LASREADER_H
