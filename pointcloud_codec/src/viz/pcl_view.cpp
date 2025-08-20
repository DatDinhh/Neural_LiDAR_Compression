#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <thread>

using PointT = pcl::PointXYZ;

static pcl::PointCloud<PointT>::Ptr load_cloud(const std::string& path) {
    auto cloud = pcl::make_shared<pcl::PointCloud<PointT>>();
    if (pcl::io::loadPCDFile<PointT>(path, *cloud) != 0) {
        std::cerr << "Failed to read " << path << "\n";
        return nullptr;
    }
    std::cout << "Loaded " << cloud->size() << " points from " << path << "\n";
    return cloud;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "Usage:\n"
                  << "  pcc_viewer <A.pcd>\n"
                  << "  pcc_viewer <A_original.pcd> <B_decoded.pcd>\n";
        return 0;
    }

    const bool compare = (argc >= 3);
    auto cloudA = load_cloud(argv[1]);
    if (!cloudA) return 1;

    pcl::visualization::PCLVisualizer::Ptr vis(new pcl::visualization::PCLVisualizer(
        compare ? "PCL Viewer – Compare" : "PCL Viewer"));
    vis->setBackgroundColor(0, 0, 0);

    if (!compare) {
        pcl::visualization::PointCloudColorHandlerGenericField<PointT> colorZ(cloudA, "z");
        vis->addPointCloud<PointT>(cloudA, colorZ, "cloudA");
        vis->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloudA");
        vis->addCoordinateSystem(0.25);
    } else {
        auto cloudB = load_cloud(argv[2]);
        if (!cloudB) return 1;

        int vL = 0, vR = 1;
        vis->createViewPort(0.0, 0.0, 0.5, 1.0, vL);
        vis->createViewPort(0.5, 0.0, 1.0, 1.0, vR);
        vis->setBackgroundColor(0, 0, 0, vL);
        vis->setBackgroundColor(0, 0, 0, vR);

        pcl::visualization::PointCloudColorHandlerGenericField<PointT> colorL(cloudA, "z");
        pcl::visualization::PointCloudColorHandlerGenericField<PointT> colorR(cloudB, "z");

        vis->addPointCloud<PointT>(cloudA, colorL, "orig", vL);
        vis->addPointCloud<PointT>(cloudB, colorR, "decoded", vR);

        vis->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "orig");
        vis->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "decoded");

        vis->addText("Original", 10, 10, 16, 1, 1, 1, "tL", vL);
        vis->addText("Decoded",  10, 10, 16, 1, 1, 1, "tR", vR);

        vis->addCoordinateSystem(0.25, "axesL", vL);
        vis->addCoordinateSystem(0.25, "axesR", vR);
    }

    vis->initCameraParameters();
    while (!vis->wasStopped()) {
        vis->spinOnce(16);
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
    return 0;
}
