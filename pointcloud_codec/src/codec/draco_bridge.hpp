// src/codec/draco_bridge.hpp
#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>
#include <stdexcept>
#include <string>
#include "pcc/config.hpp"

namespace pcc::draco_bridge {

// Throws std::runtime_error when Draco support is disabled or on failure.
void encode_positions_to_draco(const pcc::float3* pts,
                               std::size_t count,
                               int qbits,
                               std::vector<std::uint8_t>& out_bytes,
                               bool sequential_encoding = true);

// Throws std::runtime_error when Draco support is disabled or on failure.
void decode_positions_from_draco(const std::uint8_t* bytes,
                                 std::size_t byte_count,
                                 std::vector<pcc::float3>& out_pts);

} 
