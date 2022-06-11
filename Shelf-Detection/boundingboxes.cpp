#include "include/shelf-detection.hpp"

pcl::PointCloud<pcl::PointXYZ> ShelfDetection::rotate(pcl::PointCloud<pcl::PointXYZ>& cloud, double angle, const pcl::PointXYZ& center) {
    pcl::PointCloud<pcl::PointXYZ> rotated;
    pcl::PointCloud<pcl::PointXYZ> translated;
    Eigen::Affine3f tf = Eigen::Affine3f::Identity();
    Eigen::Affine3f tf2 = Eigen::Affine3f::Identity();
    tf.translation() << -center.x, -center.y, 0.0;
    pcl::transformPointCloud(cloud, translated, tf);
    tf2.rotate(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(translated, rotated, tf2);
    
    return rotated;
}


RCBB ShelfDetection::findMinimalBoundingBox(std::vector<RCBB>& candidates) {
    float area = std::numeric_limits<float>::max();
    size_t bestBox = 0;
    for(size_t i = 0; i < candidates.size(); i++) {
        if(candidates[i].area() < area) {
            bestBox = i;
            area = candidates[i].area();
        }
    }
    return candidates[bestBox];
}

pcl::ConvexHull<pcl::PointXYZ> ShelfDetection::initConvexHull(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::ConvexHull<pcl::PointXYZ> cHull;
    cHull.setDimension(2);
    cHull.setInputCloud(cloud);

    return cHull;
}

pcl::PointXYZ ShelfDetection::calculateCenterOfMass(pcl::PointCloud<pcl::PointXYZ>& cloud) {
    pcl::PointXYZ min, max, center;
    pcl::getMinMax3D(cloud, min, max);
    center.x = (max.x + min.x) / 2.0;
    center.y = (max.y + min.y) / 2.0;
    return center;
}

void ShelfDetection::distributeYAxis(RCBB& box, const float& startX, const float& endX, const float& startY, const float& endY) {
    float x = (endX + startX) / 2.0;
    int i = 0;
    for(float y = startY + 0.5; y < endY; y += 0.9) {
        if(i % 3 == 0) {
            y += 0.1;
        }
        pcl::PointXYZ p;
        p.x = x; p.y = y; p.z = 0.0;
        box.pickSlots.push_back(p);
        i++;
    }
}

void ShelfDetection::distributeXAxis(RCBB& box, const float& startY, const float& endY, const float& startX, const float& endX) {
    float y = (endY + startY) / 2.0;
    int i = 0;
    for(float x = startX + 0.4; x < endX; x += 0.9) {
        if(i % 3 == 0) {
            x += 0.1;
        }
        pcl::PointXYZ p;
        p.x = x; p.y = y; p.z = 0.0;
        box.pickSlots.push_back(p);
        i++;
    }
}

void ShelfDetection::distributePickSlots(RCBB& box) {
    if(box.width < box.height) {
        if(box.width > 2.3) {
            distributeYAxis(box, box.min.x, box.min.x + 1.2, box.min.y, box.max.y);
            distributeYAxis(box, box.max.x - 1.2, box.max.x, box.min.y, box.max.y);
        } else {
            distributeYAxis(box, box.max.x - 1.2, box.max.x, box.min.y, box.max.y);
        }
    } else {
        if(box.height > 2.3) {
            distributeXAxis(box, box.min.y, box.min.y + 1.2, box.min.x, box.max.x);
            distributeXAxis(box, box.max.y - 1.2, box.max.y, box.min.x, box.max.x);
        } else {
            distributeXAxis(box, box.min.y, box.min.y + 1.2, box.min.x, box.max.x);
        }
    }
}

void ShelfDetection::visualizeBestBox(const RCBB& box, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cc, pcl::PointCloud<pcl::PointXYZ>& hull) {
    pcl::visualization::PCLVisualizer vis;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr c(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::copyPointCloud<pcl::PointXYZRGB, pcl::PointXYZRGB>(*cc, *c);
    
    for(const auto& p : hull) {
        pcl::PointXYZRGB pp;
        pp.x = p.x; pp.y = p.y;
        pp.b = 255;
        for(float z = 0.0; z <= 5.0; z += 0.05) {
            pp.z = z;
            c->points.push_back(pp);
        }
    }

    for(const auto& p : box.corners) {
        pcl::PointXYZRGB pp;
        pp.x = p.x; pp.y = p.y; pp.g = 255;
        for(float z = 0.0; z <= 7.0; z+= 0.05) {
            pp.z = z;
            c->points.push_back(pp);
        }
    }
    vis.setBackgroundColor(0.0, 0.0, 0.0);
    vis.addPointCloud(c, "c");
    vis.addPointCloud(c, "cc");
    while(!vis.wasStopped()) {
        vis.spinOnce(1000);
    }
}

std::vector<RCBB> ShelfDetection::generateBoundingBoxes(std::vector<pcl::PointIndices>& shelves, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::ConvexHull<pcl::PointXYZ> cHull = initConvexHull(cloud);
    std::vector<RCBB> boxes;
    size_t nrSlots = 0;
    Eigen::Vector3f yAxis = Eigen::Vector3f(0.0, 1.0, 0.0);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cc = nullptr;
    if(params.visualization.visualizeIndividualBoxes) {
        cc = generateColouredCloud();
    }

    for(auto& shelf : shelves) {
        if(shelf.indices.size() < 3) {
            continue;
        }
        pcl::IndicesPtr indices = boost::make_shared<std::vector<int>>(shelf.indices);
        cHull.setIndices(indices);

        std::vector<RCBB> candidates;
        
        pcl::PointCloud<pcl::PointXYZ> hull;
        std::vector<pcl::Vertices> verts;
        cHull.reconstruct(hull, verts);
        pcl::PointXYZ hullCenter = calculateCenterOfMass(hull);
        for(size_t i = 0; i < verts[0].vertices.size(); i++) {
           Eigen::Vector3f edge(
               hull[verts[0].vertices[(i + 1) % verts[0].vertices.size()]].x - hull[verts[0].vertices[i]].x,
               hull[verts[0].vertices[(i + 1) % verts[0].vertices.size()]].y - hull[verts[0].vertices[i]].y,
               0.0
           );
            edge.normalize();
            
            float angle = pcl::getAngle3D(yAxis, edge);
        
            pcl::PointCloud<pcl::PointXYZ> rotated = rotate(hull, angle, hullCenter);
            candidates.push_back(RCBB(rotated));
            candidates.at(i).angle = angle;
            candidates.at(i).hullCenter = hullCenter;
        }
        RCBB bestBox = findMinimalBoundingBox(candidates);
        if(bestBox.width > 3.5 && bestBox.height > 3.5) {
            continue;
        }
        distributePickSlots(bestBox);
        nrSlots += bestBox.pickSlots.size();
        std::cout << bestBox.pickSlots.size() << " pick slots distributed\n";
        bestBox.rotate(-bestBox.angle, bestBox.hullCenter);

        if(params.visualization.visualizeIndividualBoxes) {
            visualizeBestBox(bestBox, cc, hull);
        }

        boxes.push_back(bestBox);
    }
    std::cout << ">>> Found " << nrSlots << " pick slots.\n";
    return boxes;
}