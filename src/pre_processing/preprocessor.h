// preprocessor.h
// Class responsible for preprocessing the point cloud for tree classification

#ifndef PREPROCESSOR_H
#define PREPROCESSOR_H

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/PointIndices.h"

#include "../type_defs.h"

// The types of vegetation indices this preprocessor can use
enum VegetationIndex {
    TGI, VARI
};

class Preprocessor
{
public:
    Preprocessor();

    // Process the point cloud with the given vegetation index type
    void ProcessPointCloud(const pcl::PointCloud<PointDefaultType>::Ptr& cloud, VegetationIndex indexType, pcl::PointCloud<PointDefaultType>::Ptr& result) const;

    // Returns true if the point is a vegetation point based on the TGI index.
    static bool IsPointVegetationPoint(const PointDefaultType& point);

private:
    void FilterOutNonVegetation(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, VegetationIndex indexType, std::vector<uint32_t>& removedIndices) const;
    void RemoveNoise(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) const;

private:
    static float TGI_THRESHOLD_VALUE_MIN;
    static float TGI_THRESHOLD_VALUE_MAX;
};

#endif // PREPROCESSOR_H
