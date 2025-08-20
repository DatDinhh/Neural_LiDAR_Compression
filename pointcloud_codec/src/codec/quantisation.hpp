#pragma once

#include <array>
#include <cstdint>
#include <span>

#include "pcc/config.hpp" 

namespace pcc {

Aabb compute_aabb(std::span<const float3> pts);

Aabb clamp_aabb(const Aabb& box, float eps);

void quantise(std::span<const float3> in,
              std::span<std::array<std::uint32_t, 3>> out,
              const Aabb& aabb,
              std::uint32_t qpos_bits);

void dequantise(std::span<const std::array<std::uint32_t, 3>> in,
                std::span<float3> out,
                const Aabb& aabb,
                std::uint32_t qpos_bits);

double rmse(std::span<const float3> a,
            std::span<const float3> b);

} 

namespace quant {
using pcc::Aabb;
using pcc::float3;

Aabb compute_aabb(std::span<const float3> pts);
Aabb clamp_aabb(const Aabb& box, float eps);

using pcc::quantise;
using pcc::dequantise;
using pcc::rmse;
} 
