#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>

int main() {
    std::string filename = "shelves.pcd";
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    reader.read(filename, cloud);
    pcl::PCDWriter writer;
    writer.write("ascii-" + filename, cloud);

    return 0;
}