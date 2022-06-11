#include "include/shelf-detection.hpp"

/**
 * @brief Loads a PLY file
 * 
 * @return true 
 * Returns true on success
 * @return false 
 * Returns false on failure
 */
bool ShelfDetection::loadPLY() {
    pcl::PLYReader reader;
    if(reader.read(file.name(), *pointCloud) < 0) {
        return false;
    }

    return true;
}

/**
 * @brief Loads a PCD file
 * 
 * @return true 
 * Returns true on succcess
 * @return false 
 * Returns false on failure
 */
bool ShelfDetection::loadPCD() {
    pcl::PCDReader reader;
    if(reader.read(file.name(), *pointCloud) < 0) {
        return false;
    }

    return true;
}


/**
 * @brief Downsamples input cloud to voxels with user adjustable leaf size(s)
 */
void ShelfDetection::downsample() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> downsampler;
    
    downsampler.setLeafSize(params.downsampling.leafSize.x,
                            params.downsampling.leafSize.y,
                            params.downsampling.leafSize.z);
    
    downsampler.setInputCloud(pointCloud);
    downsampler.filter(*tempCloud);
    
    std::cout << ">>> Downsampling input [" << pointCloud->size() << " -> " << tempCloud->size() << "]\n";
    pointCloud->clear();
    pcl::copyPointCloud(*tempCloud, *pointCloud);
}


/**
 * @brief Filters out points that corresponds to user adjustable 
 * min and max values from point cloud along a user adjustable axis
 */
void ShelfDetection::filter() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::PassThrough<pcl::PointXYZ> filter;

    filter.setInputCloud(pointCloud);
    filter.setFilterFieldName(params.filtering.fieldName);
    filter.setFilterLimits(params.filtering.minHeight,
                        params.filtering.maxHeight);

    filter.filter(*tempCloud);
    std::cout << ">>> Filtering input (" << params.filtering.minHeight 
    << " < " << params.filtering.fieldName 
    << " < " << params.filtering.maxHeight << ")"
    << "  [" << pointCloud->size() << " -> " << tempCloud->size() << "]\n";

    pointCloud->clear();
    pcl::copyPointCloud(*tempCloud, *pointCloud);
}


/**
 * @brief Downsamples and filters the input cloud if wanted
 */
void ShelfDetection::preparePointCloud() {
    if(params.downsampling.enabled) {
        downsample();
    }

    if(params.filtering.enabled) {
        filter();
    }
}


/**
 * @brief Generates the initial batch segments, all points within r metres
 * from all points in the cloud
 * 
 * @return std::vector<std::vector<int>> 
 * Set of initial clusters to characterize
 */
std::vector<std::vector<int>> ShelfDetection::generateSegments() {
    std::vector<std::vector<int>> segments;
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setSortedResults(false);
    tree.setInputCloud(pointCloud);
    
    for(const auto& point : pointCloud->points) {
        std::vector<int> elements;
        std::vector<float> distances;
        if(tree.radiusSearch(point, params.pole.radius, elements, distances) > params.pole.minPolePoints) {
            segments.push_back(std::move(elements));
        }
    }
    std::cout << ">>> Initial batch of " << segments.size() << " segments.\n";

    return segments;
}


/**
 * @brief Checks the alignment, or, the variance is larger along the Z axis than M times
 * both X and Y axis
 * 
 * @param vectors 
 * @return true 
 * Returns true if the cluster is dominant in Z direction
 * @return false 
 * Returns false if it is not
 */
bool ShelfDetection::segmentIsVertical(const Eigen::Matrix3f& vectors) {
    return (std::abs(vectors.col(0)[2]) > std::abs(params.pole.xRatio * vectors.col(0)[0]) &&
            std::abs(vectors.col(0)[2]) > std::abs(params.pole.yRatio * vectors.col(0)[1]));
}


/**
 * @brief Checks if the segment is distributed along a line by comparing eigenvalues of PCA
 * 
 * @param values Eigenvalues
 * @return true 
 * Returns true if the segment is distributed along a line
 * @return false 
 * Returns false if not
 */
bool ShelfDetection::segmentIsALine(const Eigen::Vector3f& values) {
    return (values[1] < params.pole.linearThreshold * values[0]);
}


/**
 * @brief Characterizes the generated set of clusters by checking if they are a line and if the
 * line is vertical enough
 * 
 * @param segments Segments to examine
 * @return std::vector<int> 
 * List of points that were part of segments that were vertical lines (contains duplicate points)
 */
std::vector<PoleCandidate> ShelfDetection::characterizeSegments(std::vector<std::vector<int>>& segments) {
    std::vector<std::vector<PoleCandidate>> threadSegments(params.threads.nrThreads);
    std::vector<PoleCandidate> validSegments;
    std::atomic<int> nrValid = 0;
    
    std::cout << ">>> Characterizing segments with settings: Linear threshold - " << params.pole.linearThreshold << "\n"
    << "\t\t\t\t\t   X ratio - " << params.pole.xRatio << "\n"
    << "\t\t\t\t\t   Y ratio - " << params.pole.yRatio << "\n";

    #pragma omp parallel for shared(segments)
    for(const auto& segment : segments) {
        pcl::PCA<pcl::PointXYZ> pca;
        pca.setInputCloud(pointCloud);

        int threadId = omp_get_thread_num();
        
        pcl::IndicesPtr indices = boost::make_shared<std::vector<int>>(segment); 
        pca.setIndices(indices);
        
        auto values = pca.getEigenValues();
        
        if(segmentIsALine(values) 
        && segmentIsVertical(pca.getEigenVectors())) {
                threadSegments[threadId].push_back(PoleCandidate(pca.getMean()));
                std::atomic_fetch_add(&nrValid, 1);
        }

    }


    for(auto& threadSegment : threadSegments) {
        for(auto& segment : threadSegment) {
            validSegments.push_back(segment);
        }
    }
    std::cout << ">>> Found " << nrValid << " valid segments.\n";
    return validSegments;
}


/**
 * @brief Merges candidates with an adjustable radius
 * 
 * @param candidates Candidates to merge
 */
void ShelfDetection::mergeCandidates(std::vector<PoleCandidate>& candidates) {
    std::vector<PoleCandidate> newCandidates;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr otherTemp(new pcl::PointCloud<pcl::PointXYZ>());
    for(const auto& candidate : candidates) {
        tempCloud->push_back(candidate.center);
    }

    pcl::search::KdTree<pcl::PointXYZ> tree;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    while(tempCloud->points.size()) {
        extract.setInputCloud(tempCloud);
        tree.setInputCloud(tempCloud);
        
        pcl::PointXYZ point = tempCloud->points[0];

        std::vector<int> ind; std::vector<float> dist;
        if(tree.radiusSearch(point, params.merging.radius, ind, dist)) {
            PoleCandidate p;
            float nrTimes = 0.0;
            for(const auto& index : ind) {
                p.center.x += tempCloud->points[index].x;
                p.center.y += tempCloud->points[index].y;
                nrTimes += 1.0;
            }
            p.center.x /= nrTimes;
            p.center.y /= nrTimes;
            p.nrTimes = nrTimes;
            extract.setIndices(boost::make_shared<std::vector<int>>(ind));
            extract.setNegative(true);
            extract.filter(*otherTemp);
            tempCloud.swap(otherTemp);
            newCandidates.push_back(p);
        }
    }
    candidates.swap(newCandidates);
    std::cout << ">>> Merged pole candidates size: " << candidates.size() << std::endl;
}


/**
 * @brief Computes the perpendicularity between two vectors
 * 
 * @param u First vector
 * @param v Second vector
 * @param thresh Angular threshold
 * @return true Returns true if perpendicular
 * @return false Returns false if not perpendicular
 */
bool ShelfDetection::isPerpendicular(Eigen::Vector3f& u, Eigen::Vector3f& v, const float& thresh) {
    return std::abs(u.dot(v)) <= thresh;
}


/**
 * @brief Computes the parallellity between two vectors
 * 
 * @param u First vector
 * @param v Second vector
 * @param thresh Angular threshold
 * @return true Returns true is parallel
 * @return false Returns false if not 
 */
bool ShelfDetection::isParallel(Eigen::Vector3f& u, Eigen::Vector3f& v, const float& thresh) {
    return u.dot(v) <= thresh;
}


/**
 * @brief Checks all neighbors of a point if any of them constructs perpendicular vectors
 * 
 * @param current Examined pole candidate
 * @param widths List of valid points that could be widths
 * @param depths List of valid points that could be depths
 * @param perpThresh Angular threshold
 * @param cloud Input point cloud
 * @return float Returns the score for examined pole
 */
float ShelfDetection::checkPerpendicular(PoleCandidate& current, std::vector<int>& widths, std::vector<int>& depths, const float& perpThresh, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    float score = 0.0;
    for(size_t w = 0; w < widths.size(); w++) {
        for(size_t d = 0; d < depths.size(); d++) {
            Eigen::Vector3f u(cloud->points[widths[w]].x - current.center.x, cloud->points[widths[w]].y - current.center.y, 0.0);
            u.normalize();
            Eigen::Vector3f v(cloud->points[depths[d]].x - current.center.x, cloud->points[depths[d]].y - current.center.y, 0.0);
            v.normalize();
            if(isPerpendicular(u, v, perpThresh)) {
                score += 1.0;
                current.neighbors.push_back(depths[d]);
                current.neighbors.push_back(widths[w]);
            }
        }
    }
    return score;
}

/**
 * @brief Checks all neighbors of a point if any of them construct parallel vectors
 * 
 * @param current Examined point
 * @param widths List of valid points that could be widths
 * @param paraThresh Angular threshold
 * @param cloud Input cloud
 * @return float Returns score for examined pole
 */
float ShelfDetection::checkParallelity(PoleCandidate& current, std::vector<int>& widths, const float& paraThresh, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    float score = 0.0;
    if(widths.size() >= 2) {
        for(size_t i = 0; i < widths.size() - 1; i++) {
            for(size_t j = i + 1; j < widths.size(); j++) {
                Eigen::Vector3f u(cloud->points[widths[i]].x - current.center.x, cloud->points[widths[i]].y - current.center.y, 0.0);
                u.normalize();
                Eigen::Vector3f v(cloud->points[widths[j]].x - current.center.x, cloud->points[widths[j]].y - current.center.y, 0.0);
                v.normalize();
                if(isParallel(u, v, paraThresh)) {
                    current.neighbors.push_back(widths[i]);
                    current.neighbors.push_back(widths[j]);
                    score += 1.0;
                }
            }
        }
    }
    return score;
}


/**
 * @brief Scores pole candidate based on the angles between constructed vectors
 * 
 * @param current Examined pole
 * @param widths List of points that could be widths
 * @param depths List of points that could be depths
 * @param cloud Input cloud
 * @return float Returns score of examined pole
 */
float ShelfDetection::calculateAngles(PoleCandidate& current, std::vector<std::vector<int>>& widths, std::vector<std::vector<int>>& depths, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    float score = 0.0;
    float radDiff = (params.shelves.angleDiffDeg * M_PI) / 180.0;
    float perpThresh = std::cos(M_PI_2 - radDiff);
    float paraThresh = std::cos(M_PI - radDiff);
    for(size_t shelf = 0; shelf < widths.size(); shelf++) {
        score += checkParallelity(current, widths[shelf], paraThresh, cloud);
        score += checkPerpendicular(current, widths[shelf], depths[shelf], perpThresh, cloud);
    }
    return score;
}

/**
 * @brief Recursively used to visit all reachable nodes from examined pole
 * 
 * @param candidate Examined pole
 * @param candidates All candidates
 * @param current Visited from examined
 * @param history All visited regardless
 * @return float Score
 */
float ShelfDetection::visit(PoleCandidate& candidate, std::vector<PoleCandidate>& candidates, std::vector<int>& current, std::vector<int>& history) {
    float score = candidate.score;
    for(int i = 0; i < (int) candidate.neighbors.size(); i++) {
        if(std::find(history.begin(), history.end(), candidate.neighbors[i]) == history.end()) {
            history.push_back(candidate.neighbors[i]);
            current.push_back(candidate.neighbors[i]);
            score += visit(candidates[candidate.neighbors[i]], candidates, current, history);    
        }
    }
    return score;
}


/**
 * @brief Constructs a graph and walks it
 * 
 * @param candidates List of pole candidates
 */
void ShelfDetection::walk(std::vector<PoleCandidate>& candidates) {
    std::vector<int> historyVisited;
    for(int i = 0; i < (int) candidates.size(); i++) {
        if(std::find(historyVisited.begin(), historyVisited.end(), i) == historyVisited.end()) {
            std::vector<int> currentlyVisited;
            currentlyVisited.push_back(i);
            historyVisited.push_back(i);
            candidates[i].score = visit(candidates[i], candidates, historyVisited, currentlyVisited);
            for(const auto& c : currentlyVisited) {
                if(candidates[c].score < candidates[i].score) {
                    candidates[c].score = candidates[i].score;
                }
            }
        }
    }
}


/**
 * @brief Computes scores for vertically aligned neighborhoods
 * 
 * @param candidates Neighborhoods
 * @return std::vector<PoleCandidate> Scored candidates
 */
std::vector<PoleCandidate> ShelfDetection::computeScores(std::vector<PoleCandidate>& candidates) {
    std::vector<PoleCandidate> newCandidates;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for(const auto& candidate : candidates) {
        cloud->push_back(candidate.center);
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);
    int c = 0;
    for(const auto& candidate : candidates) {
        std::vector<int> ind;
        std::vector<float> dist;
        PoleCandidate newCandidate = candidate;
        if(tree.radiusSearch(newCandidate.center, params.shelves.searchRadius, ind, dist)) {
            std::vector<std::vector<int>> depths(params.shelves.shelves.size());
            std::vector<std::vector<int>> widths(params.shelves.shelves.size());
            for(size_t index = 0; index < ind.size(); index++) {
                float d = std::sqrt(dist[index]);
                if(d > 0.5) {
                    size_t i = 0;
                    for(const auto& p : params.shelves.shelves) {
                        for(const auto& depth : p.depths) {
                            if(std::abs(d - depth) <= params.shelves.distanceThreshold) {
                                depths[i].push_back(ind[index]);
                            }
                        }
                        for(const auto& width : p.widths) {
                            if(std::abs(d - width) <= params.shelves.distanceThreshold) {
                                widths[i].push_back(ind[index]);
                            }
                        }
                        i++;
                    }
                }
            }
            newCandidate.score = calculateAngles(newCandidate, widths, depths, cloud);
            for(auto& neighbors : newCandidate.neighbors) {
                candidates[neighbors].neighbors.push_back(c);
            }
            newCandidates.push_back(newCandidate);
        }
        c++;
    }
    walk(newCandidates);

    std::vector<PoleCandidate> valids;
    for(const auto& c : newCandidates) {
        if(c.score >= params.shelves.shelfScore) {
            valids.push_back(c);
        }
    }

    return valids;
}


void ShelfDetection::saveToFile(const std::vector<RCBB>& boxes) {
    std::string fname = "../result/shelves.json";
    std::ofstream o(fname, std::ofstream::out | std::ios::trunc);
    if(o) {
        size_t index = 1;
        o << "{" << std::endl;
        o << "\t\"information\": {" << std::endl;
        o << "\t\t\"found_shelves\": " << boxes.size() << "," << std::endl;
        o << "\t\t\"total_pick_slots\": {" << std::endl;
        size_t slotIndex = 1;
        for(const auto& box : boxes) {
            if(slotIndex == boxes.size()) {
            o << "\t\t\t" << "\"shelf_" + std::to_string(slotIndex) << "\": " << box.pickSlots.size() << std::endl;
            } else {
                o << "\t\t\t" << "\"shelf_" + std::to_string(slotIndex) << "\": " << box.pickSlots.size() << "," << std::endl;
            }
            slotIndex++;
        }
        o << "\t\t}" << std::endl;
        o << "\t}," << std::endl;
        o << "\t\"extracted_pick_slots\": {" << std::endl;
        for(const auto& box : boxes) {
            std::string s = "\t\t\"shelf_" + std::to_string(index) + "\": {";
            o << s << std::endl;
            slotIndex = 1;
            for(const auto& slot : box.pickSlots) {
                if(slotIndex == box.pickSlots.size()) {                
                    o << "\t\t\t\"" << "pickslot_" + std::to_string(slotIndex) << "\": " << "[" << slot.x << ", " << slot.y << "]" << std::endl; 
                } else {
                    o << "\t\t\t\"" << "pickslot_" + std::to_string(slotIndex) << "\": " << "[" << slot.x << ", " << slot.y << "]," << std::endl; 
                }
                slotIndex++;
            }
            
            if(index == boxes.size()) {
                o << "\t\t}" << std::endl;
            } else {
                o << "\t\t}," << std::endl;
            }
            index++;
        }
        
        o << "\t}" << std::endl << "}" << std::endl;

    } else {
        std::cout << "Could not open file " << fname << std::endl;
    }
}

/**
 * @brief The entirety
 * 
 */
void ShelfDetection::run() {
    std::vector<std::vector<int>> initialSegments = generateSegments();
    
    // Finding (possibly) valid poles
    std::vector<PoleCandidate> initialSetOfPoles = characterizeSegments(initialSegments);
    if(params.merging.enabled) {
        mergeCandidates(initialSetOfPoles);
    }
    if(params.visualization.visualizeSegmentedPoles) {
        visualizeCandidates(initialSetOfPoles);
    }
    std::vector<PoleCandidate> couldBeValid = computeScores(initialSetOfPoles);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for(const auto& pole : couldBeValid) {
        cloud->push_back(pole.center);
    }
    if(params.visualization.visualizeScoredSegments) {
        visualizeCandidates(couldBeValid);
    }
    segmenter.init(cloud);
    std::cout << ">>> Poles remaining: " << cloud->size() << ".\n";
    std::vector<pcl::PointIndices> shelves = segmenter.findRacks();
    if(params.visualization.visualizeSegmentedRacks) {
        visualize(shelves, cloud);
    }
    std::vector<RCBB> boxes = generateBoundingBoxes(shelves, cloud);
    if(params.visualization.visualizePickSlots) {
        visualize(boxes);
    }

    saveToFile(boxes);
}


/**
 * @brief Load point cloud from file
 * 
 * @return true 
 * Returns true on success
 * @return false 
 * Returns false on failure
 */
bool ShelfDetection::loadPointCloud() {
    if(file.isPCD()) {
        return loadPCD();
    } else if(file.isPLY()) {
        return loadPLY();
    } else {
        std::cout << "Unsupported format. PCD/PLY only.\n";
        return false;
    }
}