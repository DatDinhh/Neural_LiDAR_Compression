// include/pcc/stream_encoder.hpp
#pragma once
#include <cstdint>
#include <cstddef>
#include <memory>
#include <vector>
#include <span>

#include "pcc/config.hpp"
#include "pcc/attributes.hpp"

namespace pcc {

struct ChunkHeader {
    std::uint32_t magic;      
    std::uint16_t ver_major;   
    std::uint16_t ver_minor;   
    std::uint32_t chunk_id;    
    std::uint32_t point_count;
    std::uint8_t  qpos_bits;   
    std::uint8_t  attr_mask;   
    std::uint16_t reserved;    
    float aabb_min[3];
    float aabb_max[3];
    std::uint32_t payload_bytes; 
};

struct EncodedChunk {
    ChunkHeader header{};
    std::vector<std::uint8_t> payload;  
    std::uint32_t crc32{0};             
};

class StreamEncoder {
public:
    explicit StreamEncoder(const Config& cfg);
    ~StreamEncoder();

    StreamEncoder(StreamEncoder&&) noexcept;
    StreamEncoder& operator=(StreamEncoder&&) noexcept;

    StreamEncoder(const StreamEncoder&) = delete;
    StreamEncoder& operator=(const StreamEncoder&) = delete;

    void reset(std::uint32_t next_chunk_id = 0);

    void set_config(const Config& cfg);
    const Config& config() const noexcept;

    std::size_t push(std::span<const float3> pts);

    bool pop(EncodedChunk& out);

    bool flush();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} 
