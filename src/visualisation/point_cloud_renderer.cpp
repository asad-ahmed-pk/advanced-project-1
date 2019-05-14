//
// PointCloudRenderer.cpp
//

#include "point_cloud_renderer.h"

#include "pcl/point_types.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "pcl/common/centroid.h"

#include "Eigen/Core"

#include <thread>
#include <chrono>
#include <iostream>
#include <string>

const std::string DEFAULT_CLOUD_NAME { "cloud" };

void PointClicked(const pcl::visualization::PointPickingEvent& event, void* viewerVoid);


// Constructor with the default RGB cloud
PointCloudRenderer::PointCloudRenderer(pcl::PointCloud<PointDefaultType>::Ptr ptr) : m_PointCloudRGBPtr(ptr) {
    InitVisualizer();
}

// Constructor with XYZI point cloud
PointCloudRenderer::PointCloudRenderer(pcl::PointCloud<pcl::PointXYZI>::Ptr ptr) : m_PointCloudIPtr(ptr) {
    InitVisualizer();
}

// Visualiser Init
void PointCloudRenderer::InitVisualizer()
{
    m_VisualizerPtr = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));

    m_VisualizerPtr->setBackgroundColor (0, 0, 0);
    m_VisualizerPtr->setShowFPS(true);

    if (m_PointCloudIPtr != nullptr) {
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> fieldHandler(m_PointCloudIPtr, "intensity");
        m_VisualizerPtr->addPointCloud<pcl::PointXYZI>(m_PointCloudIPtr, fieldHandler, DEFAULT_CLOUD_NAME);
    }
    else {
        pcl::visualization::PointCloudColorHandlerRGBField<PointDefaultType> rgb(m_PointCloudRGBPtr);
        m_VisualizerPtr->addPointCloud<PointDefaultType> (m_PointCloudRGBPtr, rgb, DEFAULT_CLOUD_NAME);
    }

    m_VisualizerPtr->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, DEFAULT_CLOUD_NAME);
    m_VisualizerPtr->addCoordinateSystem(1.0);
    m_VisualizerPtr->initCameraParameters();

    // setup point click callback
    m_VisualizerPtr->registerPointPickingCallback(boost::bind(&PointCloudRenderer::PointClicked, this, _1));
}

// Render
void PointCloudRenderer::Render() const
{
    while (!m_VisualizerPtr->wasStopped()) {
        m_VisualizerPtr->spinOnce (100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    m_VisualizerPtr->close();
}

// Add point cloud
void PointCloudRenderer::AddPointCloud(const pcl::PointCloud<PointDefaultType>::ConstPtr& cloud, const std::string& name, double pointSize)
{
    pcl::visualization::PointCloudColorHandlerCustom<PointDefaultType> rgb(cloud, 255, 255, 255);
    m_VisualizerPtr->addPointCloud(cloud, rgb, name);
    m_VisualizerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, name);
}

// Mouse click event
void PointCloudRenderer::PointClicked(const pcl::visualization::PointPickingEvent& event)
{
    float x, y, z;
    event.getPoint(x, y, z);

    std::cout << "\nPoint #" << event.getPointIndex();
    std::cout << "\nClicked point at: (" << x << ", " << y << ", " << z << ")";

    // print out more information about the point
    if (m_PointCloudIPtr != nullptr) {
        std::cout << "\nIntensity: " << m_PointCloudIPtr->points[event.getPointIndex()].intensity << std::endl;
    }
}
