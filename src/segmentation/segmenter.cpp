// segmenter.cpp

#include "segmenter.h"

#include "pcl/features/normal_3d.h"
#include "pcl/search/search.h"
#include "pcl/search/kdtree.h"
#include "pcl/segmentation/region_growing.h"
#include "pcl/segmentation/region_growing_rgb.h"
//#include "pcl/gpu/segmentation/gpu_seeded_hue_segmentation.h"

#include <vector>

const int MIN_CLUSTER_SIZE { 200 };

Segmenter::Segmenter()
{


}

// Segmentation (assuming intensities filtered by some vegetation index filter)
void Segmenter::SegmentVegetationIndexedCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& result) const
{
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::Search<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);

    ComputeNormals(cloud, normals);

    pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> reg;

    reg.setMinClusterSize (MIN_CLUSTER_SIZE);
    //reg.setMaxClusterSize (1000000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (30);
    reg.setInputCloud (cloud);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (1.0);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

    std::cout << "\nNumber of clusters: " << clusters.size() << std::endl;

    result = reg.getColoredCloud();
}

// Segment based on colour
void Segmenter::SegementPointCloudByColor(const pcl::PointCloud<PointDefaultType>::Ptr cloud, pcl::PointCloud<PointDefaultType>::Ptr& result) const
{
    pcl::search::Search<PointDefaultType>::Ptr tree(new pcl::search::KdTree<PointDefaultType>);

    pcl::RegionGrowingRGB<PointDefaultType> reg;
    reg.setInputCloud(cloud);
    reg.setSearchMethod(tree);
    reg.setDistanceThreshold(0.1);
    reg.setPointColorThreshold(15);
    reg.setRegionColorThreshold(5);
    reg.setMinClusterSize(300);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

    std::cout << "\nNumber of clusters detected: " << clusters.size() << std::endl;

    //pcl::io::savePCDFile(outputFilePath, *coloredCloud, false);

    result = reg.getColoredCloud();
}

// GPU segmentation
void Segmenter::SegmentWithGPU(const pcl::PointCloud<PointDefaultType>::Ptr& cloud, pcl::PointCloud<PointDefaultType>::Ptr& result) const
{
    
}

// Compute normals
void Segmenter::ComputeNormals(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr& normals) const
{
    pcl::search::Search<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normalEstimator;

    normalEstimator.setSearchMethod(tree);
    normalEstimator.setInputCloud(cloud);
    normalEstimator.setKSearch(50);
    normalEstimator.compute(*normals);
}
