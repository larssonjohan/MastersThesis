#pragma once
#include <fstream>

#include "parameters.hpp"
#include "json.hpp"

ShelfParams getParameters(std::string filename) {
    ShelfParams params;
    std::ifstream file(filename);
    nlohmann::json j = nlohmann::json::parse(file);

    // Thread settings
    params.threads.nrThreads = j["OpenMP"]["nrThreads"];
    
    // Downsample settings
    params.downsampling.enabled = j["Downsampling"]["enabled"];
    params.downsampling.leafSize.x = j["Downsampling"]["leafSize.x"];
    params.downsampling.leafSize.y = j["Downsampling"]["leafSize.y"];
    params.downsampling.leafSize.z = j["Downsampling"]["leafSize.z"];

    // Filter settings
    params.filtering.enabled = j["Filtering"]["enabled"];
    params.filtering.fieldName = j["Filtering"]["fieldName"];
    params.filtering.minHeight = j["Filtering"]["minHeight"];
    params.filtering.maxHeight = j["Filtering"]["maxHeight"];
    
    // Segmentation (Radially bound nearest neighbor) settings
    params.segment.radius = j["RadiusSegmenter"]["radius"];
    params.segment.minPoints = j["RadiusSegmenter"]["minPoints"];

    // Initial pole settings
    params.pole.radius = j["Poles"]["radius"];
    params.pole.linearThreshold = j["Poles"]["linearThreshold"];
    params.pole.minPolePoints = j["Poles"]["minPolePoints"];
    params.pole.xRatio = j["Poles"]["xRatio"];
    params.pole.yRatio = j["Poles"]["yRatio"];

    // Visualization settings
    params.visualization.visualizeSegmentedPoles = j["Visualization"]["visualizeSegmentedPoles"];
    params.visualization.visualizeIndividualBoxes = j["Visualization"]["visualizeIndividualBoxes"];
    params.visualization.visualizeScoredSegments = j["Visualization"]["visualizeScoredSegments"];
    params.visualization.visualizeSegmentedRacks = j["Visualization"]["visualizeSegmentedRacks"];
    params.visualization.visualizePickSlots = j["Visualization"]["visualizePickSlots"];

    // Shelf dimensions
    params.shelves.angleDiffDeg = j["ShelfDimensions"]["angleDiffDeg"];
    params.shelves.distanceThreshold = j["ShelfDimensions"]["distanceThreshold"];
    for(const auto& shelf : j["ShelfDimensions"]["dims"]) {
        std::vector<float> depths = shelf["depths"];
        std::vector<float> widths = shelf["widths"];
        Shelf s(depths, widths);
        params.shelves.shelves.push_back(s);
    }
    params.shelves.shelfScore = j["ShelfDimensions"]["shelfScore"];
    params.shelves.searchRadius = j["ShelfDimensions"]["searchRadius"];

    // Merging means
    params.merging.enabled = j["Merging"]["enabled"];
    params.merging.radius = j["Merging"]["radius"];
    return params;
}