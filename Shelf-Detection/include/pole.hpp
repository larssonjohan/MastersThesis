#include <pcl/point_types.h>
#include <Eigen/Dense>
struct PoleCandidate {
    PoleCandidate() : 
    center(0.0, 0.0, 0.0),
    score(0.0),
    nrTimes(0.0) {}

    PoleCandidate(Eigen::Vector4f m) :
    center(m[0], m[1], 0.0),
    score(0.0),
    nrTimes(0.0) {}

    PoleCandidate(const PoleCandidate& other) {
        this->center.x = other.center.x;
        this->center.y = other.center.y;
        this->center.z = other.center.z;
        this->score = other.score;
        this->nrTimes = other.nrTimes;
        for(const auto& n : other.neighbors) {
            this->neighbors.push_back(n);
        }
    }

    pcl::PointXYZ center;
    std::vector<int> neighbors;
    float score;
    float nrTimes;
};