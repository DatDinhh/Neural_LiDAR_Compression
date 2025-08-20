#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <cmath>

using PointT = pcl::PointXYZ;

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cout << "Usage: pcd_rmse original.pcd decoded.pcd\n";
        return 2;
    }
    pcl::PointCloud<PointT>::Ptr a(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr b(new pcl::PointCloud<PointT>);
    if (pcl::io::loadPCDFile<PointT>(argv[1], *a) != 0) { std::cerr<<"load "<<argv[1]<<" failed\n"; return 1; }
    if (pcl::io::loadPCDFile<PointT>(argv[2], *b) != 0) { std::cerr<<"load "<<argv[2]<<" failed\n"; return 1; }

    const size_t n = std::min(a->size(), b->size());
    if (n == 0) { std::cerr<<"empty clouds\n"; return 1; }

    long double acc = 0.0;
    for (size_t i=0;i<n;++i) {
        long double dx = a->points[i].x - b->points[i].x;
        long double dy = a->points[i].y - b->points[i].y;
        long double dz = a->points[i].z - b->points[i].z;
        acc += dx*dx + dy*dy + dz*dz;
    }
    double rmse = std::sqrt((double)(acc / (long double)n));
    std::cout << "RMSE over " << n << " points: " << rmse << "\n";
    return 0;
}
