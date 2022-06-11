#include "include/shelf-detection.hpp"
/**
 * @brief Generates a coloured cloud for visualization
 * 
 * @return pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
 * Pointer to generated cloud
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr ShelfDetection::generateColouredCloud() {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointXYZRGB color(255, 255, 255);
    for(const auto& point : pointCloud->points) {
        pcl::PointXYZRGB newPoint;
        newPoint.x = point.x; newPoint.y = point.y; newPoint.z = point.z;
        newPoint.rgb = color.rgb;
        tempCloud->points.push_back(newPoint);
    }
    return tempCloud;
}


void ShelfDetection::visualize(std::vector<pcl::PointIndices>& indices, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    auto cCloud = generateColouredCloud();
    std::vector colors = {
        pcl::PointXYZRGB(255, 0, 0),
        pcl::PointXYZRGB(0, 255, 0),
        pcl::PointXYZRGB(0, 0, 255),
        pcl::PointXYZRGB(255, 255, 0),
        pcl::PointXYZRGB(255, 0, 255),
        pcl::PointXYZRGB(0, 255, 255)
    };
    int color = 0;
    for(const auto& shelf : indices) {
        for(const auto& point : shelf.indices) {
            pcl::PointXYZRGB pp;
            pp.x = cloud->points[point].x;
            pp.y = cloud->points[point].y;
            pp.rgb = colors[color % colors.size()].rgb;
            cCloud->push_back(pp);
        }
        color++;
    }
    visualize(cCloud);
}


void ShelfDetection::visualize(std::vector<RCBB>& boxes) {
    auto cloud = generateColouredCloud();
    pcl::visualization::PCLVisualizer visualizer("Visualizer");
    visualizer.setBackgroundColor(0, 0, 0);
    visualizer.initCameraParameters();
    for(const auto& box : boxes) {
        for(const auto& slot : box.pickSlots) {
            for(float x = slot.x - 0.05; x <= slot.x + 0.05; x += 0.05) {
                for(float y = slot.y - 0.05; y <= slot.y + 0.05; y += 0.05) {
                    for(float z = 0.0; z <= 4.5; z += 0.05) {
                        pcl::PointXYZRGB pp;
                        pp.x = x; pp.y = y; pp.z = z;
                        pp.r = 255; pp.g = 255; pp.b = 0;
                        cloud->points.push_back(pp);
                    }
                }
            }
            
        }
    }
    visualizer.addPointCloud(cloud);
    int i = 1;
    for(const auto& box : boxes) {
        std::string id = "Cube" + std::to_string(i);
        for(size_t c = 0; c < box.corners.size(); c++) {
            std::string lowerId = id + "L" + std::to_string(c);
            std::string upperId = id + "U" + std::to_string(c);
            std::string horizontalId = id + "H" + std::to_string(c);
            pcl::PointXYZ p1 = box.corners[c]; p1.z = 2.0;
            pcl::PointXYZ p2 = box.corners[(c + 1) % box.corners.size()]; p2.z = 2.0;
            visualizer.addLine<pcl::PointXYZ, pcl::PointXYZ>(box.corners[c], box.corners[(c + 1) % box.corners.size()], 0.0, 0.0, 1.0, lowerId);
            visualizer.addLine<pcl::PointXYZ, pcl::PointXYZ>(p1, p2, 0.0, 0.0, 1.0, upperId);
            visualizer.addLine<pcl::PointXYZ, pcl::PointXYZ>(box.corners[c], p1, 0.0, 0.0, 1.0, horizontalId);
        }
        i++;
    }
    while(!visualizer.wasStopped()) {
        visualizer.spinOnce(100);
    }
}


/**
 * @brief Visualizes the cloud created from poles, clusters of poles (shelves) 
 * and clusters of shelves (racks)
 * 
 * @param cloud The loaded point cloud
 */
void ShelfDetection::visualize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
    pcl::visualization::PCLVisualizer visualizer("Visualizer");
    visualizer.setBackgroundColor(0, 0, 0);
    visualizer.initCameraParameters();
    visualizer.addPointCloud(cloud);
    while(!visualizer.wasStopped()) {
        visualizer.spinOnce(100);
    }
}



void ShelfDetection::visualizeCandidates(std::vector<PoleCandidate>& candidates) {
    auto tempCloud = generateColouredCloud();
    for(const auto& c : candidates) {
        pcl::PointXYZRGB newPoint(255, 255, 0);
        newPoint.x = c.center.x;
        newPoint.y = c.center.y;
        for(float z = 0.0; z <= 5.0; z += 0.05) {
            newPoint.z = z;
            tempCloud->push_back(newPoint);
        }
    }
    visualize(tempCloud);
}
