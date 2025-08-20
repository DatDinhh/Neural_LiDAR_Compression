// tools/compare_clouds.cpp
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <iostream>
#include <cmath>
#include <string>

static double rmse_indexwise(const pcl::PointCloud<pcl::PointXYZ>& A,
                             const pcl::PointCloud<pcl::PointXYZ>& B) {
    if (A.size() != B.size())
        throw std::runtime_error("indexwise RMSE requires same point count");
    double sum_sq = 0.0;
    for (size_t i = 0; i < A.size(); ++i) {
        const auto dx = A[i].x - B[i].x;
        const auto dy = A[i].y - B[i].y;
        const auto dz = A[i].z - B[i].z;
        sum_sq += dx*dx + dy*dy + dz*dz;
    }
    return std::sqrt(sum_sq / static_cast<double>(A.size()));
}

static double rmse_c2c(const pcl::PointCloud<pcl::PointXYZ>& A,
                       const pcl::PointCloud<pcl::PointXYZ>& B) {
    pcl::KdTreeFLANN<pcl::PointXYZ> kdt;
    kdt.setInputCloud(B.makeShared());
    double sum_sq = 0.0;
    std::vector<int> idx(1);
    std::vector<float> dist(1);
    for (const auto& p : A) {
        if (kdt.nearestKSearch(p, 1, idx, dist) == 1) {
            sum_sq += dist[0]; 
        }
    }
    return std::sqrt(sum_sq / static_cast<double>(A.size()));
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: compare_clouds <a.pcd> <b.pcd> [--c2c]\n";
        return 1;
    }
    const std::string pathA = argv[1];
    const std::string pathB = argv[2];
    const bool use_c2c = (argc >= 4 && std::string(argv[3]) == "--c2c");

    pcl::PointCloud<pcl::PointXYZ> A, B;
    if (pcl::io::loadPCDFile(pathA, A) != 0) { std::cerr << "Load failed: " << pathA << "\n"; return 1; }
    if (pcl::io::loadPCDFile(pathB, B) != 0) { std::cerr << "Load failed: " << pathB << "\n"; return 1; }

    try {
        if (use_c2c) {
            const double rms = rmse_c2c(A, B);
            std::cout << "C2C RMSE: " << rms << " m\n";
        } else {
            const double rms = rmse_indexwise(A, B);
            std::cout << "Indexwise RMSE: " << rms << " m\n";
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 2;
    }
    return 0;
}
