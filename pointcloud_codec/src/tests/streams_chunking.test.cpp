// tests/streams_chunking.test.cpp
#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <draco/point_cloud/point_cloud_builder.h>
#include <draco/point_cloud/point_cloud.h>
#include <draco/compression/point_cloud/point_cloud_encoder.h>
#include <draco/compression/point_cloud/point_cloud_decoder.h>

#if __has_include(<draco/compression/encode.h>)
  #include <draco/compression/encode.h>
  #include <draco/compression/decode.h>
  #define DRACO_NEW_API 1
#else
  #define DRACO_NEW_API 0
#endif

#include <filesystem>
#include <vector>
#include <cmath>

namespace fs = std::filesystem;

static fs::path find_data_file(const std::string& name) {
    fs::path here = fs::path(__FILE__).parent_path();
    std::vector<fs::path> candidates = {
        here / ".." / "data" / name,
        here / ".." / ".." / "data" / name,
        fs::current_path() / "data" / name
    };
    for (auto& p : candidates) if (fs::exists(p)) return fs::weakly_canonical(p);
    return {};
}

static std::unique_ptr<draco::PointCloud> slice_to_draco(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                                         size_t begin, size_t end) {
    end = std::min(end, cloud.size());
    size_t n = (end > begin) ? (end - begin) : 0;
    draco::PointCloudBuilder b;
    b.Start(n);
    const int pos_id = b.AddAttribute(draco::GeometryAttribute::POSITION, 3, draco::DT_FLOAT32);
#if DRACO_NEW_API
    for (size_t i = 0; i < n; ++i)
        b.SetAttributeValuesForPoint(pos_id, draco::PointIndex(i), &cloud[begin + i].x);
#else
    for (size_t i = 0; i < n; ++i)
        b.SetAttributeValueForPoint(pos_id, draco::PointIndex(i), &cloud[begin + i].x);
#endif
    return b.Finalize(false);
}

TEST(Streaming, ChunkingRoundtrip) {
    const auto pcd_path = find_data_file("table_scene_lms400.pcd");
    ASSERT_FALSE(pcd_path.empty());

    pcl::PointCloud<pcl::PointXYZ> cloud;
    ASSERT_EQ(pcl::io::loadPCDFile(pcd_path.string(), cloud), 0);

    // Encode in chunks and concatenate decoded result
    const size_t K = 65536;
    pcl::PointCloud<pcl::PointXYZ> recon;
    recon.reserve(cloud.size());

    size_t total_bytes = 0;

    for (size_t off = 0; off < cloud.size(); off += K) {
        auto dpc = slice_to_draco(cloud, off, off + K);

        draco::EncoderBuffer enc;
        draco::Encoder encoder;
        encoder.SetEncodingMethod(draco::POINT_CLOUD_SEQUENTIAL_ENCODING);
        encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, 14);
        ASSERT_TRUE(encoder.EncodePointCloudToBuffer(*dpc, &enc).ok());
        total_bytes += enc.size();

        draco::DecoderBuffer dec;
        dec.Init(enc.data(), enc.size());

#if DRACO_NEW_API
        auto res = draco::DecodePointCloudFromBuffer(&dec);
        ASSERT_TRUE(res.ok());
        std::unique_ptr<draco::PointCloud> outpc(res.value().release());
#else
        draco::Decoder decoder;
        auto res = decoder.DecodePointCloudFromBuffer(&dec);
        ASSERT_TRUE(res.ok());
        std::unique_ptr<draco::PointCloud> outpc(res.value().release());
#endif

        const auto* pos = outpc->GetNamedAttribute(draco::GeometryAttribute::POSITION);
        ASSERT_NE(pos, nullptr);
        size_t base = recon.size();
        recon.resize(base + outpc->num_points());
        for (draco::PointIndex p(0); p < outpc->num_points(); ++p) {
            pos->GetValue(draco::AttributeValueIndex(p.value()), &recon[base + p.value()].x);
        }
    }

    EXPECT_EQ(recon.size(), cloud.size());
    double sum_sq = 0.0;
    for (size_t i = 0; i < cloud.size(); ++i) {
        const auto dx = cloud[i].x - recon[i].x;
        const auto dy = cloud[i].y - recon[i].y;
        const auto dz = cloud[i].z - recon[i].z;
        sum_sq += dx*dx + dy*dy + dz*dz;
    }
    const double rms = std::sqrt(sum_sq / static_cast<double>(cloud.size()));
    EXPECT_LT(rms, 1e-2);

    EXPECT_LT(total_bytes, 2u * 1024u * 1024u);
}
