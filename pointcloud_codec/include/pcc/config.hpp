// include/pcc/config.hpp
#pragma once
#include <cstdint>
#include <limits>

namespace pcc {

struct float3 {
    float x{}, y{}, z{};
};

struct Aabb {
    float3 min{};
    float3 max{};
};

// Quantization parameters for positions
struct QuantParams {
    int bits = 14;           
    Aabb aabb{};             
    float clamp_max_range = 0.0f; 
};

struct Config {
    QuantParams qpos{};              
    std::uint32_t chunk_points = 65536; 
    int threads = 0;               
    bool enable_crc32 = false;     
    bool allow_lossless = false;     

    static constexpr int kMinBits = 1;
    static constexpr int kMaxBits = 30;

    bool valid() const noexcept {
        return (qpos.bits >= kMinBits && qpos.bits <= kMaxBits) &&
               (chunk_points > 0);
    }
};

} 
