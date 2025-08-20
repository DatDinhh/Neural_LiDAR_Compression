// ensure quant header and kill Windows min/max
#if __has_include(<pcc/codec/quantisation.hpp>)
  #include <pcc/codec/quantisation.hpp>
#elif __has_include("codec/quantisation.hpp")
  #include "codec/quantisation.hpp"
#elif __has_include("../codec/quantisation.hpp")
  #include "../codec/quantisation.hpp"
#else
  #error "quantisation.hpp not found"
#endif

#ifndef NOMINMAX
#define NOMINMAX 1
#endif
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif
#include <deque>
#include <stdexcept>
#include <cstring>
#include "pcc/stream_encoder.hpp"
#include "quantisation.hpp"
#include "draco_bridge.hpp"

namespace pcc {

static constexpr std::uint32_t kMagic = 0x524E4F31u; 

struct StreamEncoder::Impl {
    Config cfg{};
    std::uint32_t next_chunk_id{0};

    std::vector<float3> buf;       
    std::deque<EncodedChunk> ready;

    explicit Impl(const Config& c) : cfg(c) {
        if (!cfg.valid()) throw std::runtime_error("StreamEncoder: invalid config");
        buf.reserve(cfg.chunk_points);
    }

    void encode_current_chunk() {
        if (buf.empty()) return;

        Aabb aabb = quant::compute_aabb(buf);
        aabb = quant::clamp_aabb(aabb, cfg.qpos.clamp_max_range);

        std::vector<std::uint8_t> payload;
        draco_bridge::encode_positions_to_draco(
            buf.data(), buf.size(), cfg.qpos.bits, payload, /*sequential=*/true);

        EncodedChunk out{};
        out.header.magic       = kMagic;
        out.header.ver_major   = 0;
        out.header.ver_minor   = 1;
        out.header.chunk_id    = next_chunk_id++;
        out.header.point_count = static_cast<std::uint32_t>(buf.size());
        out.header.qpos_bits   = static_cast<std::uint8_t>(cfg.qpos.bits);
        out.header.attr_mask   = 0;
        out.header.reserved    = 0;
        out.header.aabb_min[0] = aabb.min.x; out.header.aabb_min[1] = aabb.min.y; out.header.aabb_min[2] = aabb.min.z;
        out.header.aabb_max[0] = aabb.max.x; out.header.aabb_max[1] = aabb.max.y; out.header.aabb_max[2] = aabb.max.z;
        out.header.payload_bytes = static_cast<std::uint32_t>(payload.size());

        out.payload = std::move(payload);
        out.crc32 = 0;

        ready.emplace_back(std::move(out));
        buf.clear();
    }
};

StreamEncoder::StreamEncoder(const Config& cfg)
: impl_(std::make_unique<Impl>(cfg)) {}

StreamEncoder::~StreamEncoder() = default;
StreamEncoder::StreamEncoder(StreamEncoder&&) noexcept = default;
StreamEncoder& StreamEncoder::operator=(StreamEncoder&&) noexcept = default;

void StreamEncoder::reset(std::uint32_t next_chunk_id) {
    impl_->buf.clear();
    impl_->ready.clear();
    impl_->next_chunk_id = next_chunk_id;
}

void StreamEncoder::set_config(const Config& cfg) {
    if (!cfg.valid()) throw std::runtime_error("StreamEncoder::set_config: invalid config");
    impl_->cfg = cfg;
    impl_->buf.reserve(cfg.chunk_points);
}

const Config& StreamEncoder::config() const noexcept { return impl_->cfg; }

std::size_t StreamEncoder::push(std::span<const float3> pts) {
    if (pts.empty()) return 0;
    const std::size_t cap = static_cast<std::size_t>(impl_->cfg.chunk_points) - impl_->buf.size();
    const std::size_t take = std::min(cap, pts.size());
    impl_->buf.insert(impl_->buf.end(), pts.begin(), pts.begin() + take);

    if (impl_->buf.size() == impl_->cfg.chunk_points) {
        impl_->encode_current_chunk();
    }
    return take;
}

bool StreamEncoder::pop(EncodedChunk& out) {
    if (impl_->ready.empty()) return false;
    out = std::move(impl_->ready.front());
    impl_->ready.pop_front();
    return true;
}

bool StreamEncoder::flush() {
    const bool had = !impl_->buf.empty();
    impl_->encode_current_chunk();
    return had;
}

} 

