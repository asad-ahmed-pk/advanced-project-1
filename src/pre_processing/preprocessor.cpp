// preprocessor.cpp

#include "preprocessor.h"

#include "pcl/filters/passthrough.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/filters/extract_indices.h"

#include "Eigen/Eigen"

#include <algorithm>
#include <vector>
#include <map>
#include <numeric>

// TGI threshold values to filter out vegetation
float Preprocessor::TGI_THRESHOLD_VALUE_MIN = 30.0f;
float Preprocessor::TGI_THRESHOLD_VALUE_MAX = 100.0f;

Preprocessor::Preprocessor()
{

}

// Pre-process
void Preprocessor::ProcessPointCloud(const pcl::PointCloud<PointDefaultType>::Ptr& cloud, VegetationIndex indexType, pcl::PointCloud<PointDefaultType>::Ptr& result) const
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr vegetationIndexedCloud(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<float> intensities;

    // apply one of the RGB vegetation indices and create a new point cloud
    float intensity = 0.0f;
    uint32_t filteredIndex = 0;

    for (uint32_t i = 0; i < cloud->width; i++)
    {
        Eigen::Vector3f rgb(cloud->points[i].getRGBVector3i().x(), cloud->points[i].getRGBVector3i().y(), cloud->points[i].getRGBVector3i().z());

        if (indexType == VegetationIndex::TGI) {
            intensity = rgb[1] - 0.39f * rgb[0] - 0.61f * rgb[2];
        }
        else
        {
            float denom = (rgb[1] + rgb[0] - rgb[2]);
            if (denom > 0.0f) {
                intensity = (rgb[1] - rgb[0]) / denom;
            }
            else {
                intensity = 0.0f;
            }
        }

        intensities.push_back(intensity);

        pcl::PointXYZI point(intensity);

        point.x = cloud->points[i].x;
        point.y = cloud->points[i].y;
        point.z = cloud->points[i].z;

        vegetationIndexedCloud->push_back(point);
    }

    std::cout << "\nMax Intesity: " << *std::max_element(intensities.begin(), intensities.end());
    std::cout << "\nMin Intensity: " << *std::min_element(intensities.begin(), intensities.end());
    std::cout << "\nAverage Intensity: " << std::accumulate(intensities.begin(), intensities.end(), 0.0f) / intensities.size();
    std::cout << std::endl;

    // filter out non-vegetation and remove noise
    std::vector<uint32_t> removedIndices;
    FilterOutNonVegetation(vegetationIndexedCloud, indexType, removedIndices);
    RemoveNoise(vegetationIndexedCloud);

    std::cout << "\nFiltered Indices After TGI: " << removedIndices.size();
    std::cout << "\nOriginal Cloud Indices: " << cloud->points.size();
    std::cout << std::endl;

    // filter out the original RGB cloud with the indices that were used in the vegetation and noise filter
    pcl::PointIndicesPtr removedIndicesPtr(new pcl::PointIndices);
    removedIndicesPtr->indices.insert(removedIndicesPtr->indices.end(), removedIndices.begin(), removedIndices.end());

    pcl::ExtractIndices<PointDefaultType> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(removedIndicesPtr);
    extract.setNegative(true);
    extract.filter(*result);
}

// Filter out non vegetation. This will be based on the intensity values and the vegetation index used.
void Preprocessor::FilterOutNonVegetation(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, VegetationIndex indexType, std::vector<uint32_t>& removedIndices) const
{
    // height filter
    const float HEIGHT_FILTER_VALUE_MIN { 10.0f };
    const float HEIGHT_FILTER_VALUE_MAX { 100.0f };

    // intensity filter
    pcl::PassThrough<pcl::PointXYZI> pass(true);
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("intensity");

    if (indexType == VegetationIndex::TGI) {
        pass.setFilterLimits(Preprocessor::TGI_THRESHOLD_VALUE_MIN, Preprocessor::TGI_THRESHOLD_VALUE_MAX);
    }
    else {
        std::cerr << "\nError: Only TGI filtering supported!" << std::endl;
        return;
    }

    pass.filter(*cloud);
    pass.getRemovedIndices();

    // track indices that were removed
    removedIndices.insert(removedIndices.end(), pass.getRemovedIndices()->begin(), pass.getRemovedIndices()->end());

    // second pass - height filter
    pcl::PassThrough<pcl::PointXYZI> heightPass;

    heightPass.setInputCloud(cloud);
    heightPass.setFilterFieldName("z");
    heightPass.setFilterLimits(HEIGHT_FILTER_VALUE_MIN, HEIGHT_FILTER_VALUE_MAX);
    heightPass.filter(*cloud);
}

// Is vegetation point
bool Preprocessor::IsPointVegetationPoint(const PointDefaultType& point)
{
    // calculate TGI
    Eigen::Vector3i rgb = point.getRGBVector3i();
    float intensity = rgb[1] - 0.39f * rgb[0] - 0.61f * rgb[2];
    return (intensity >= Preprocessor::TGI_THRESHOLD_VALUE_MIN && intensity <= Preprocessor::TGI_THRESHOLD_VALUE_MAX);
}

// Remove noise from the cloud
void Preprocessor::RemoveNoise(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) const
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;

    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(2.0);

    sor.filter(*cloud);
}

