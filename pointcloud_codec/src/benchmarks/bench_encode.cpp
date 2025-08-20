// tests/benchmarks/bench_encode.cpp
#include <benchmark/benchmark.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <draco/point_cloud/point_cloud_builder.h>
#include <draco/compression/point_cloud/point_cloud_encoder.h>

#include <filesystem>
#include <memory>
#include <vector>

namespace fs = std::filesystem;

static std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> g_cloud;

static fs::path find_data_file(const std::string& name) {
    fs::path here = fs::path(__FILE__).parent_path().parent_path().parent_path(); // tests/
    std::vector<fs::path> candidates = {
        here / "data" / name,
        fs::current_path() / "data" / name
    };
    for (auto& p : candidates) if (fs::exists(p)) return p;
    return {};
}

static void load_once() {
    if (g_cloud) return;
    g_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto p = find_data_file("table_scene_lms400.pcd");
    if (p.empty()) throw std::runtime_error("test data not found");
    if (pcl::io::loadPCDFile(p.string(), *g_cloud) != 0)
        throw std::runtime_error("failed to load PCD");
}

static std::unique_ptr<draco::PointCloud> to_draco(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
    draco::PointCloudBuilder b;
    b.Start(cloud.size());
    const int pos_id = b.AddAttribute(draco::GeometryAttribute::POSITION, 3, draco::DT_FLOAT32);
#if __has_include(<draco/compression/encode.h>)
    for (size_t i = 0; i < cloud.size(); ++i)
        b.SetAttributeValuesForPoint(pos_id, draco::PointIndex(i), &cloud[i].x);
#else
    for (size_t i = 0; i < cloud.size(); ++i)
        b.SetAttributeValueForPoint(pos_id, draco::PointIndex(i), &cloud[i].x);
#endif
    return b.Finalize(false);
}

static void BenchEncodeQ(benchmark::State& st, int qbits) {
    load_once();
    auto dpc = to_draco(*g_cloud);

    for (auto _ : st) {
        draco::Encoder encoder;
        encoder.SetEncodingMethod(draco::POINT_CLOUD_SEQUENTIAL_ENCODING);
        encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, qbits);

        draco::EncoderBuffer enc;
        auto ok = encoder.EncodePointCloudToBuffer(*dpc, &enc).ok();
        if (!ok) throw std::runtime_error("encode failed");

        st.counters["bytes"] = static_cast<double>(enc.size());
    }
    st.SetItemsProcessed(static_cast<int64_t>(g_cloud->size()) * st.iterations());
}

static void B_Q10(benchmark::State& s) { BenchEncodeQ(s, 10); }
static void B_Q12(benchmark::State& s) { BenchEncodeQ(s, 12); }
static void B_Q14(benchmark::State& s) { BenchEncodeQ(s, 14); }
static void B_Q16(benchmark::State& s) { BenchEncodeQ(s, 16); }

BENCHMARK(B_Q10);
BENCHMARK(B_Q12);
BENCHMARK(B_Q14);
BENCHMARK(B_Q16);

BENCHMARK_MAIN();
