#pragma once
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "parameters.hpp"

class RadiusSegmentation {
public:
    RadiusSegmentation() = delete;
    RadiusSegmentation(const SegmentParams& p) :
    params(p),
    tree(new pcl::search::KdTree<pcl::PointXYZ>())
    {}
    void init(const pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud);
    std::vector<pcl::PointIndices> findRacks();
private:
    const SegmentParams params;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> extractor;
};