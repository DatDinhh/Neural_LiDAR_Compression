// src/codec/stream_decoder.cpp
#include <stdexcept>
#include <cstring>
#include "pcc/stream_decoder.hpp"
#include "draco_bridge.hpp"

namespace pcc {

namespace {

constexpr std::size_t kHeaderFixedSize = 48; 
constexpr std::uint32_t kMagic = 0x524E4F31u; 

static std::uint16_t rd_u16_le(const std::uint8_t* p) {
    return static_cast<std::uint16_t>(p[0] | (static_cast<std::uint16_t>(p[1]) << 8));
}
static std::uint32_t rd_u32_le(const std::uint8_t* p) {
    return static_cast<std::uint32_t>(p[0] |
           (static_cast<std::uint32_t>(p[1]) << 8) |
           (static_cast<std::uint32_t>(p[2]) << 16) |
           (static_cast<std::uint32_t>(p[3]) << 24));
}
static float rd_f32_le(const std::uint8_t* p) {
    std::uint32_t u = rd_u32_le(p);
    float f;
    std::memcpy(&f, &u, sizeof(float));
    return f;
}

static void parse_header(const std::uint8_t* p, std::size_t n, ChunkHeader& h) {
    if (n < kHeaderFixedSize) throw std::runtime_error("stream_decoder: frame too small");
    const std::uint32_t magic = rd_u32_le(p + 0);
    if (magic != kMagic) throw std::runtime_error("stream_decoder: bad magic");

    h.magic        = magic;
    h.ver_major    = rd_u16_le(p + 4);
    h.ver_minor    = rd_u16_le(p + 6);
    h.chunk_id     = rd_u32_le(p + 8);
    h.point_count  = rd_u32_le(p + 12);
    h.qpos_bits    = p[16];
    h.attr_mask    = p[17];
    h.reserved     = rd_u16_le(p + 18);

    h.aabb_min[0]  = rd_f32_le(p + 20);
    h.aabb_min[1]  = rd_f32_le(p + 24);
    h.aabb_min[2]  = rd_f32_le(p + 28);
    h.aabb_max[0]  = rd_f32_le(p + 32);
    h.aabb_max[1]  = rd_f32_le(p + 36);
    h.aabb_max[2]  = rd_f32_le(p + 40);
    h.payload_bytes= rd_u32_le(p + 44);
}

} 

struct StreamDecoder::Impl {
    Impl() = default;
};

StreamDecoder::StreamDecoder() : impl_(std::make_unique<Impl>()) {}
StreamDecoder::~StreamDecoder() = default;
StreamDecoder::StreamDecoder(StreamDecoder&&) noexcept = default;
StreamDecoder& StreamDecoder::operator=(StreamDecoder&&) noexcept = default;

void StreamDecoder::decode_one(std::span<const std::uint8_t> frame, DecodedChunk& out) {
    if (frame.size() < kHeaderFixedSize) {
        throw std::runtime_error("stream_decoder: frame too small");
    }

    parse_header(frame.data(), frame.size(), out.header);

    const std::size_t need = kHeaderFixedSize + static_cast<std::size_t>(out.header.payload_bytes);
    if (frame.size() < need) {
        throw std::runtime_error("stream_decoder: incomplete frame payload");
    }

    const std::uint8_t* payload = frame.data() + kHeaderFixedSize;
    const std::size_t   pbytes  = static_cast<std::size_t>(out.header.payload_bytes);

    pcc::draco_bridge::decode_positions_from_draco(payload, pbytes, out.points);

    out.header.point_count = static_cast<std::uint32_t>(out.points.size());
}

std::size_t StreamDecoder::peek_frame_size(std::span<const std::uint8_t> bytes) {
    if (bytes.size() < kHeaderFixedSize) return 0;
    const std::uint32_t magic = rd_u32_le(bytes.data());
    if (magic != kMagic) return 0;

    const std::uint32_t payload = rd_u32_le(bytes.data() + 44);
    const std::size_t total = kHeaderFixedSize + static_cast<std::size_t>(payload);
    return (bytes.size() >= total) ? total : 0;
}

} 
