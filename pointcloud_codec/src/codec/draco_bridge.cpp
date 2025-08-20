// src/codec/draco_bridge.cpp
#include "draco_bridge.hpp"
#include <sstream>

#if PCC_WITH_DRACO
  #include <draco/point_cloud/point_cloud_builder.h>
  #include <draco/compression/point_cloud/point_cloud_encoder.h>
  #include <draco/compression/point_cloud/point_cloud_decoder.h>
  #include <draco/core/encoder_buffer.h>
  #include <draco/core/decoder_buffer.h>

#endif

namespace pcc::draco_bridge {

static std::runtime_error make_err(const char* msg) {
    return std::runtime_error(std::string("draco_bridge: ") + msg);
}

void encode_positions_to_draco(const pcc::float3* pts,
                               std::size_t count,
                               int qbits,
                               std::vector<std::uint8_t>& out_bytes,
                               bool sequential_encoding) {
#if PCC_WITH_DRACO
    if (!pts || count == 0) {
        out_bytes.clear();
        return;
    }

    draco::PointCloudBuilder b;
    b.Start(static_cast<int>(count));
    const int att_id = b.AddAttribute(draco::GeometryAttribute::POSITION,
                                      3, draco::DT_FLOAT32);

    for (int i = 0; i < static_cast<int>(count); ++i) {
        b.SetAttributeValueForPoint(att_id, draco::PointIndex(i), &pts[i].x);
    }

    std::unique_ptr<draco::PointCloud> pc = b.Finalize(/*use_metadata=*/false);
    if (!pc) throw make_err("builder Finalize() failed");

    draco::Encoder enc;
    if (sequential_encoding) {
        enc.SetEncodingMethod(draco::POINT_CLOUD_SEQUENTIAL_ENCODING);
    }
    if (qbits > 0) {
        enc.SetAttributeQuantization(draco::GeometryAttribute::POSITION, qbits);
    }

    draco::EncoderBuffer ebuf;
    const draco::Status st = enc.EncodePointCloudToBuffer(*pc, &ebuf);
    if (!st.ok()) throw make_err(st.error_msg());

    out_bytes.assign(reinterpret_cast<const std::uint8_t*>(ebuf.data()),
                     reinterpret_cast<const std::uint8_t*>(ebuf.data()) + ebuf.size());
#else
    (void)pts; (void)count; (void)qbits; (void)out_bytes; (void)sequential_encoding;
    throw make_err("PCC_WITH_DRACO=0 (Draco backend not available)");
#endif
}

void decode_positions_from_draco(const std::uint8_t* bytes,
                                 std::size_t byte_count,
                                 std::vector<pcc::float3>& out_pts) {
#if PCC_WITH_DRACO
    if (!bytes || byte_count == 0) {
        out_pts.clear();
        return;
    }

    draco::DecoderBuffer dbuf;
    dbuf.Init(reinterpret_cast<const char*>(bytes),
              static_cast<size_t>(byte_count));

    draco::Decoder dec;
    auto res = dec.DecodePointCloudFromBuffer(&dbuf);
    if (!res.ok()) throw make_err(res.status().error_msg());

    std::unique_ptr<draco::PointCloud> pc = std::move(res).value();
    const draco::PointAttribute* pos =
        pc->GetNamedAttribute(draco::GeometryAttribute::POSITION);
    if (!pos) throw make_err("POSITION attribute missing");

    const int n = static_cast<int>(pc->num_points());
    out_pts.resize(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        pos->GetValue(draco::AttributeValueIndex(i), &out_pts[static_cast<std::size_t>(i)].x);
    }
#else
    (void)bytes; (void)byte_count; (void)out_pts;
    throw make_err("PCC_WITH_DRACO=0 (Draco backend not available)");
#endif
}

} 
