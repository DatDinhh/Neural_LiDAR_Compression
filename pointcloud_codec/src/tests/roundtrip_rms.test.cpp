// tests/roundtrip_rms.test.cpp
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
#include <fstream>
#include <vector>
#include <cmath>
#include <optional>

namespace fs = std::filesystem;

static fs::path find_data_file(const std::string& name) {
    fs::path here = fs::path(__FILE__).parent_path();
    std::vector<fs::path> candidates = {
        here / ".." / "data" / name,                
        here / ".." / ".." / "data" / name,                
        fs::current_path() / "data" / name                 
    };
    for (auto& p : candidates) {
        if (fs::exists(p)) return fs::weakly_canonical(p);
    }
    return {};
}

static double rmse_indexwise(const pcl::PointCloud<pcl::PointXYZ>& a,
                             const pcl::PointCloud<pcl::PointXYZ>& b) {
    ASSERT_EQ(a.size(), b.size());
    double sum_sq = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
        const auto dx = a[i].x - b[i].x;
        const auto dy = a[i].y - b[i].y;
        const auto dz = a[i].z - b[i].z;
        sum_sq += dx*dx + dy*dy + dz*dz;
    }
    return std::sqrt(sum_sq / static_cast<double>(a.size()));
}

static std::unique_ptr<draco::PointCloud> pcl_to_draco(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
    auto pc = std::make_unique<draco::PointCloud>();
    draco::PointCloudBuilder builder;
    builder.Start(cloud.size());
    const int pos_id = builder.AddAttribute(draco::GeometryAttribute::POSITION, 3, draco::DT_FLOAT32);

#if DRACO_NEW_API
    for (size_t i = 0; i < cloud.size(); ++i) {
        builder.SetAttributeValuesForPoint(pos_id, draco::PointIndex(i), &cloud[i].x);
    }
#else
    for (size_t i = 0; i < cloud.size(); ++i) {
        builder.SetAttributeValueForPoint(pos_id, draco::PointIndex(i), &cloud[i].x);
    }
#endif
    return builder.Finalize(false);
}

TEST(Roundtrip, RMSEAndSize) {
    const auto pcd_path = find_data_file("table_scene_lms400.pcd");
    ASSERT_FALSE(pcd_path.empty()) << "PCD sample not found. Place it under <repo>/data/";

    pcl::PointCloud<pcl::PointXYZ> in;
    ASSERT_EQ(pcl::io::loadPCDFile(pcd_path.string(), in), 0) << "Failed to load PCD";

    auto dpc = pcl_to_draco(in);

    draco::EncoderBuffer enc;
#if DRACO_NEW_API
    draco::Encoder encoder;
    encoder.SetEncodingMethod(draco::POINT_CLOUD_SEQUENTIAL_ENCODING);
    encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, 14);
    auto ok = encoder.EncodePointCloudToBuffer(*dpc, &enc).ok();
#else
    draco::Encoder encoder;
    encoder.SetEncodingMethod(draco::POINT_CLOUD_SEQUENTIAL_ENCODING);
    encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, 14);
    auto ok = encoder.EncodePointCloudToBuffer(*dpc, &enc).ok();
#endif
    ASSERT_TRUE(ok);
    ASSERT_GT(enc.size(), 0u);

    EXPECT_LT(enc.size(), 2u * 1024u * 1024u);

    draco::DecoderBuffer dec;
    dec.Init(enc.data(), enc.size());

#if DRACO_NEW_API
    auto dec_res = draco::DecodePointCloudFromBuffer(&dec);
    ASSERT_TRUE(dec_res.ok());
    std::unique_ptr<draco::PointCloud> outpc(dec_res.value().release());
#else
    draco::Decoder decoder;
    auto dec_res = decoder.DecodePointCloudFromBuffer(&dec);
    ASSERT_TRUE(dec_res.ok());
    std::unique_ptr<draco::PointCloud> outpc(dec_res.value().release());
#endif

    pcl::PointCloud<pcl::PointXYZ> out;
    out.resize(outpc->num_points());
    const auto* pos = outpc->GetNamedAttribute(draco::GeometryAttribute::POSITION);
    ASSERT_NE(pos, nullptr);
    for (draco::PointIndex p(0); p < outpc->num_points(); ++p) {
        pos->GetValue(draco::AttributeValueIndex(p.value()), &out[p.value()].x);
    }

    const double rms = rmse_indexwise(in, out);
    EXPECT_LT(rms, 1e-2) << "RMSE too high: " << rms;
}
