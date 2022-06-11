#pragma once

#include <pcl/point_types.h>

struct Downsampling {
    Downsampling() {}
    pcl::PointXYZ leafSize;
    bool enabled;
};

struct Visualization {
    bool visualizeSegmentedPoles;
    bool visualizeIndividualBoxes;
    bool visualizeScoredSegments;
    bool visualizeSegmentedRacks;
    bool visualizePickSlots;
};

struct Threading {
    int nrThreads;
};

struct Merging {
    bool enabled;
    float radius;
};

struct Shelf {
    Shelf(std::vector<float> d, std::vector<float> w) {
        for(const auto& depth : d) {
            depths.push_back(depth);
        }
        for(const auto& width : w) {
            widths.push_back(width);
        }
    }
    std::vector<float> depths;
    std::vector<float> widths;
};

struct ShelfDimensions {
    std::vector<Shelf> shelves;
    float angleDiffDeg;
    float distanceThreshold;
    float shelfScore;
    float searchRadius;
};

struct Filtering {
    Filtering() {}

    std::string fieldName;
    float minHeight;
    float maxHeight;
    bool enabled;
};

struct SegmentParams {
    float radius;
    int minPoints;
};

struct PoleParams {
    float radius;
    float linearThreshold;
    int minPolePoints;
    float xRatio;
    float yRatio;
};

struct ShelfParams {
    ShelfParams() {}
    Threading threads;
    Filtering filtering;
    Downsampling downsampling;
    SegmentParams segment;
    Visualization visualization;
    PoleParams pole;
    ShelfDimensions shelves;
    Merging merging;
};