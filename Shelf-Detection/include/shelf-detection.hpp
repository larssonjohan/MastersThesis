#pragma once
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <fstream>

#include <omp.h>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <limits>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <pcl/common/pca.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/transforms.h>
#include <pcl/Vertices.h>
#include <pcl/filters/extract_indices.h>

#include "parameters.hpp"
#include "file.hpp"
#include "radius-segment.hpp"
#include "boundingbox.hpp"
#include "pole.hpp"
#include "json.hpp"

class ShelfDetection {
public:
    ShelfDetection() = delete;
    ShelfDetection(const std::string& filename, ShelfParams params) :
    file(File(filename)),
    params(params),
    pointCloud(new pcl::PointCloud<pcl::PointXYZ>()),
    segmenter(RadiusSegmentation(params.segment))
    {
        if(loadPointCloud()) {
            omp_set_num_threads(params.threads.nrThreads);
            segmenter.init(pointCloud);
            preparePointCloud();
            run();
        }
    }
private:
    // Visualization
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr generateColouredCloud();
    void visualizeCandidates(std::vector<PoleCandidate>& candidates);
    void visualize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void visualize(std::vector<RCBB>& boxes);
    void visualize(std::vector<pcl::PointIndices>& shelves, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    // File handling
    bool loadPLY();
    bool loadPCD();
    bool loadPointCloud();

    // Preparations to reduce computation time
    void preparePointCloud();
    void downsample();
    void filter();
    
    void run();

    // Characterization
    std::vector<std::vector<int>> generateSegments();
    void mergeCandidates(std::vector<PoleCandidate>& candidates);
    std::vector<PoleCandidate> characterizeSegments(std::vector<std::vector<int>>& segments);
    std::vector<PoleCandidate> scorePoleCandidates(std::vector<PoleCandidate>& candidates);
    bool segmentIsVertical(const Eigen::Matrix3f& vectors);
    bool segmentIsALine(const Eigen::Vector3f& values);

    // Scoring
    float calculateAngles(PoleCandidate& point, std::vector<std::vector<int>>& widths, std::vector<std::vector<int>>& depths, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    float checkParallelity(PoleCandidate& current, std::vector<int>& widths, const float& paraThresh, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    float checkPerpendicular(PoleCandidate& current, std::vector<int>& widths, std::vector<int>& depths, const float& perpThresh, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    bool isParallel(Eigen::Vector3f& u, Eigen::Vector3f& v, const float& thresh);
    bool isPerpendicular(Eigen::Vector3f& u, Eigen::Vector3f& v, const float& thresh);
    float visit(PoleCandidate& c, std::vector<PoleCandidate>& candidates, std::vector<int>& current, std::vector<int>& history);
    void walk(std::vector<PoleCandidate>& candidates);    
    std::vector<PoleCandidate> computeScores(std::vector<PoleCandidate>& candidates);
    

    // Bounding boxes
    pcl::ConvexHull<pcl::PointXYZ> initConvexHull(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    pcl::PointCloud<pcl::PointXYZ> rotate(pcl::PointCloud<pcl::PointXYZ>& cloud, double angle, const pcl::PointXYZ& center);
    pcl::PointXYZ calculateCenterOfMass(pcl::PointCloud<pcl::PointXYZ>& hull);
    RCBB findMinimalBoundingBox(std::vector<RCBB>& candidates);
    std::vector<RCBB> generateBoundingBoxes(std::vector<pcl::PointIndices>& shelves, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void visualizeBestBox(const RCBB& box, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>& hull);

    // Pick slots distribution
    void distributePickSlots(RCBB& box);
    void distributeXAxis(RCBB& box, const float& startX, const float& endX, const float& startY, const float& endY);
    void distributeYAxis(RCBB& box, const float& startY, const float& endY, const float& startX, const float& endX);


    // Save result to file
    void saveToFile(const std::vector<RCBB>& boxes);

    File file;
    ShelfParams params;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud;
    pcl::IndicesPtr indices;
    RadiusSegmentation segmenter;
};