#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <cmath>

using PC = pcl::PointCloud<pcl::PointXYZ>;

static double rmse_nn_one_way(const PC& A, const PC& B) {
    pcl::KdTreeFLANN<pcl::PointXYZ> kdt;
    kdt.setInputCloud(B.makeShared());
    long double acc = 0.0L;
    std::vector<int> idx(1); std::vector<float> dist2(1);
    for (const auto& p : A.points) {
        if (kdt.nearestKSearch(p, 1, idx, dist2) == 1) acc += dist2[0];
    }
    return std::sqrt((double)(acc / (long double)A.size()));
}

int main(int argc, char** argv) {
    if (argc < 3) { std::cout << "Usage: pcd_chamfer A.pcd B.pcd\n"; return 2; }
    PC A, B;
    if (pcl::io::loadPCDFile(argv[1], A) != 0) { std::cerr<<"load "<<argv[1]<<" failed\n"; return 1; }
    if (pcl::io::loadPCDFile(argv[2], B) != 0) { std::cerr<<"load "<<argv[2]<<" failed\n"; return 1; }
    if (A.empty() || B.empty()) { std::cerr<<"empty cloud\n"; return 1; }

    const double rAB = rmse_nn_one_way(A, B);
    const double rBA = rmse_nn_one_way(B, A);
    std::cout << "NN-RMSE A->B: " << rAB << "\n"
              << "NN-RMSE B->A: " << rBA << "\n"
              << "Chamfer RMSE: " << 0.5*(rAB + rBA) << "\n";
    return 0;
}
