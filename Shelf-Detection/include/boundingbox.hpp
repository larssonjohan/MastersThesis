#pragma once

#include <pcl/point_types.h>
#include <vector>
#include <math.h>
#include <pcl/Vertices.h>
#include <limits>
#include <Eigen/Eigen>

struct RCBB {
    RCBB(const pcl::PointCloud<pcl::PointXYZ>& hull) {
        pcl::getMinMax3D<pcl::PointXYZ>(hull, min, max);
        width = max.x - min.x;
        height = max.y - min.y;
        corners.push_back(pcl::PointXYZ(min.x, min.y, 0.0));
        corners.push_back(pcl::PointXYZ(max.x, min.y, 0.0));
        corners.push_back(pcl::PointXYZ(max.x, max.y, 0.0));
        corners.push_back(pcl::PointXYZ(min.x, max.y, 0.0));
    }

    void rotate(double angle, const pcl::PointXYZ& center) {
        Eigen::Affine3f tf = Eigen::Affine3f::Identity();
        Eigen::Affine3f tf2 = Eigen::Affine3f::Identity();
        tf.rotate(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ()));
        tf2.translation() << center.x, center.y, 0.0;

        for(auto& corner : corners) {
            pcl::PointXYZ r = pcl::transformPoint(corner, tf);
            pcl::PointXYZ t = pcl::transformPoint(r, tf2);
            corner.x = t.x; corner.y = t.y; corner.z = 0.0;
        }
        for(auto & slot : pickSlots) {
            pcl::PointXYZ r = pcl::transformPoint(slot, tf);
            pcl::PointXYZ t = pcl::transformPoint(r, tf2);
            slot.x = t.x; slot.y = t.y; slot.z = 0.0;
        }
    }

    void rotate() {
        rotate(-angle, hullCenter);
    }
    
    float area() {
        return width * height;
    }

    float width, height;
    std::vector<pcl::PointXYZ> pickSlots;
    pcl::PointXYZ hullCenter, min, max;
    float angle;
    std::vector<pcl::PointXYZ> corners;
};