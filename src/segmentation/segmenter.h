// segmenter.h
// Segments scenes into different clusters

#ifndef SEGMENTER_H
#define SEGMENTER_H

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "../type_defs.h"

class Segmenter
{
public:
    Segmenter();

    // Segment the given (filtered) cloud. It is assumed the cloud is filtered on some vegetation index.
    void SegmentVegetationIndexedCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& result) const;

    // Segment the given cloud using colour
    void SegementPointCloudByColor(const pcl::PointCloud<PointDefaultType>::Ptr cloud, pcl::PointCloud<PointDefaultType>::Ptr& result) const;

private:
    void ComputeNormals(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr& normals) const;
};

#endif // SEGMENTER_H
