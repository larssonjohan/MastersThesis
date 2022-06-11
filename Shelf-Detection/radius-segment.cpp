#include "include/radius-segment.hpp"

void RadiusSegmentation::init(const pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud) {
    cloud = inCloud;
    extractor.setInputCloud(cloud);
    tree->setInputCloud(cloud);
    extractor.setSearchMethod(tree);
}

std::vector<pcl::PointIndices> RadiusSegmentation::findRacks() {
    std::cout << ">>> Segmenting with radius: " << params.radius << ".\n";
    
    std::vector<pcl::PointIndices> extractedIndices;
    
    extractor.setClusterTolerance(params.radius);
    extractor.extract(extractedIndices);
    
    return extractedIndices;
}