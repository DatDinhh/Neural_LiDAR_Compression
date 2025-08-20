// io/las_reader.hpp
#pragma once
#include <string>
#include <vector>
#include <cstddef>
#include "pcc/config.hpp"

namespace pcc::io {

bool read_las_positions(const std::string& path,
                        std::vector<float3>& out,
                        std::size_t max_points = 0);

} 
