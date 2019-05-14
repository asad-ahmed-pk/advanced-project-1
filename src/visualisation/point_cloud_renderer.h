//
// Renders the point clouds using VTK
//

#ifndef POINTCLOUDTREECLASSIFIER_POINTCLOUDRENDERER_H
#define POINTCLOUDTREECLASSIFIER_POINTCLOUDRENDERER_H

#include <string>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "pcl/visualization/pcl_visualizer.h"

#include "../type_defs.h"

class PointCloudRenderer
{
public:
    // Constructors
    PointCloudRenderer(pcl::PointCloud<PointDefaultType>::Ptr ptr);
    PointCloudRenderer(pcl::PointCloud<pcl::PointXYZI>::Ptr ptr);

    // Render the pointcloud
    void Render() const;

    // Add another point cloud to the set of point clouds
    void AddPointCloud(const pcl::PointCloud<PointDefaultType>::ConstPtr& cloud, const std::string& name, double pointSize = 1.0);

    ~PointCloudRenderer(){}

private:
    void InitVisualizer();
    void PointClicked(const pcl::visualization::PointPickingEvent& event);

private:
    pcl::PointCloud<PointDefaultType>::Ptr m_PointCloudRGBPtr { nullptr };
    pcl::PointCloud<pcl::PointXYZI>::Ptr m_PointCloudIPtr { nullptr };
    pcl::visualization::PCLVisualizer::Ptr m_VisualizerPtr { nullptr };
};


#endif //POINTCLOUDTREECLASSIFIER_POINTCLOUDRENDERER_H
