// classifier.cpp

#include <iostream>

#include "pcl/keypoints/sift_keypoint.h"
#include "pcl/keypoints/impl/sift_keypoint.hpp"
#include "pcl/features/normal_3d.h"
#include "pcl/search/search.h"
#include "pcl/search/kdtree.h"

#include "classifier.h"

Classifier::Classifier(const pcl::PointCloud<PointDefaultType>::Ptr& scene) : m_Scene(scene), m_SiftKeyPoints(new pcl::PointCloud<PointDefaultType>)
{
    ComputeSIFT();
}

void Classifier::ComputeSIFT()
{
    std::cout << "\nComputing SIFT keypoints...";
    std::cout.flush();

    // sift params
    const float minScale = 0.1f;
    const int octaves = 6;
    const int scalesPerOctave = 10;
    const float minContrast = 0.5f;

    pcl::SIFTKeypoint<PointDefaultType, pcl::PointWithScale> sift;
    pcl::PointCloud<pcl::PointWithScale> result;
    pcl::search::KdTree<PointDefaultType>::Ptr tree(new pcl::search::KdTree<PointDefaultType>());

    sift.setSearchMethod(tree);
    sift.setScales(minScale, octaves, scalesPerOctave);
    sift.setMinimumContrast(minContrast);
    sift.setInputCloud(m_Scene);
    sift.compute(result);

    pcl::copyPointCloud(result, *m_SiftKeyPoints);
}

// Return computed keypoints
pcl::PointCloud<PointDefaultType>::Ptr Classifier::GetComputedKeypoints() const {
    return m_SiftKeyPoints;
}
