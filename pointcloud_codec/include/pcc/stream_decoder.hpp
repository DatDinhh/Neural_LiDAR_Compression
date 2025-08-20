// include/pcc/stream_decoder.hpp
#pragma once
#include <cstdint>
#include <cstddef>
#include <memory>
#include <vector>
#include <span>

#include "pcc/config.hpp"
#include "pcc/attributes.hpp"
#include "pcc/stream_encoder.hpp"

namespace pcc {

struct DecodedChunk {
    ChunkHeader header{};
    std::vector<float3> points; 
};

class StreamDecoder {
public:
    StreamDecoder();
    ~StreamDecoder();

    StreamDecoder(StreamDecoder&&) noexcept;
    StreamDecoder& operator=(StreamDecoder&&) noexcept;

    StreamDecoder(const StreamDecoder&) = delete;
    StreamDecoder& operator=(const StreamDecoder&) = delete;

    void decode_one(std::span<const std::uint8_t> frame, DecodedChunk& out);

    static std::size_t peek_frame_size(std::span<const std::uint8_t> bytes);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} 
