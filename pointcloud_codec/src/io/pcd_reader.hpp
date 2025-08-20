// io/pcd_reader.hpp
#pragma once
#include <string>
#include <vector>
#include <cstddef>
#include <span>
#include <cstdint>
#include <stdexcept>
#include "pcc/config.hpp"

namespace pcc::io {
bool read_pcd_positions(const std::string& path,
                        std::vector<float3>& out,
                        std::size_t max_points = 0);

bool write_pcd_ascii(const std::string& path,
                     std::span<const float3> pts);

} 
