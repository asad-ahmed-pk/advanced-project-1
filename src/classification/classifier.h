// classifier.h

// Classifies the trees in the given scene

#ifndef CLASSIFIER_H
#define CLASSIFIER_H

#include "pcl/point_cloud.h"
#include "../type_defs.h"

class Classifier
{
public:
    // Constructor
    Classifier(const pcl::PointCloud<PointDefaultType>::Ptr& scene);

    // The classified scene
    void ClassifedScene(pcl::PointCloud<PointDefaultType>::Ptr& result);

    // Get the point cloud with SIFT keypoints
    pcl::PointCloud<PointDefaultType>::Ptr GetComputedKeypoints() const;

private:
    void ComputeSIFT();
    void GroupBySIFT();

private:
    pcl::PointCloud<PointDefaultType>::Ptr m_Scene;
    pcl::PointCloud<PointDefaultType>::Ptr m_SiftKeyPoints;
    pcl::PointCloud<PointDefaultType>::Ptr m_SiftComputedScene;
};

#endif // CLASSIFIER_H
