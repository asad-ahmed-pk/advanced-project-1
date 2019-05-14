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
    void ProcessPointCloud(const pcl::PointCloud<PointDefaultType>::Ptr& cloud, VegetationIndex indexType, pcl::PointCloud<PointDefaultType>::Ptr& result) const;

private:
    void FilterOutNonVegetation(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, VegetationIndex indexType, std::vector<uint32_t>& removedIndices) const;
    void RemoveNoise(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) const;
};

#endif // PREPROCESSOR_H
