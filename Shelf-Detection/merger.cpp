#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <omp.h>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <string>

#define NR_THREADS 12
struct Point3D {
    Point3D () {}
    Point3D(float x, float y, float z) :
    x(x),
    y(y),
    z(z) {}
    float x;
    float y;
    float z;
    float distance(const pcl::PointXYZ p) {
        return std::sqrt(std::pow(p.x - x, 2.0) + std::pow(p.y - y, 2.0) + std::pow(p.z - z, 2.0));
    }
};

std::vector<Point3D> loadPoses(std::string file) {
    std::vector<Point3D> poses;
    std::ifstream f(file);
    if(f.is_open()) {
        std::string line;
        while(std::getline(f, line)) {
            std::stringstream ss(line);
            Point3D p;
            ss >> p.x >> p.y >> p.z;
            poses.push_back(p);
        }
    }
    return poses;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>> createVectors(const int n) {
    std::vector<pcl::PointCloud<pcl::PointXYZ>> clouds;
    for(int i = 0; i < n; i++) {
        pcl::PointCloud<pcl::PointXYZ> c;
        clouds.push_back(std::move(c));
    }
    return clouds;
}

int main() {
    
    std::string folder = "../2021-06-30_0/data/";
    std::string pose = "sensor.txt";
    std::string c_prefix = "cloud_";
    omp_set_num_threads(NR_THREADS);
    
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr fullCloud(new pcl::PointCloud<pcl::PointXYZ>());

    auto clouds = createVectors(NR_THREADS);
    std::vector<Point3D> poses = loadPoses(folder + pose);
    std::cout << "Poses size: " << poses.size() << std::endl;
    
    #pragma omp parallel for
    for(int i = 0; i < (int) poses.size(); i++) {
        int index = omp_get_thread_num();
        pcl::PointCloud<pcl::PointXYZ> cloud;
        std::stringstream ss;
        ss << folder << c_prefix << std::setw(4) << std::setfill('0') << i << ".pcd";
        reader.read(ss.str(), cloud);
        Point3D p = poses[i];
        for(const auto& point : cloud) {
            if(p.distance(point) <= 5.0) {
                clouds[index].push_back(point);
            }
        }
    }
    
    for(const auto& cloud : clouds) {
        for(const auto& point : cloud) {
            fullCloud->push_back(point);
        }
    }

    std::cout << ">>> Unfiltered size: " << fullCloud->size() << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ>::Ptr sor(new pcl::VoxelGrid<pcl::PointXYZ>());
    sor->setLeafSize(0.05, 0.05, 0.05);
    sor->setInputCloud(fullCloud);
    sor->filter(*filtered);
    std::cout << ">>> Filtered size: " << filtered->size() << std::endl;
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>("06-30-5cm-ascii.pcd", *filtered);

    return 0;
}