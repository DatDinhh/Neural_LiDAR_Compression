// cli/cli_options.hpp
#pragma once
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <optional>

#if __has_include(<cxxopts.hpp>)
  #include <cxxopts.hpp>
  #define PCC_HAVE_CXXOPTS 1
#else
  #define PCC_HAVE_CXXOPTS 0
#endif

namespace pcc::cli {

struct CmdEncode {
    std::string in_path;
    std::string out_path;
    int qpos_bits = 14;
    std::size_t chunk_points = 65536;
};

struct CmdDecode {
    std::string in_path;
    std::string out_path;
};

struct CmdBench {
    std::string in_path;
    std::vector<int> qpos_list;
    std::string report_csv;
    std::size_t chunk_points = 65536;
};

enum class Mode { Encode, Decode, Bench, Help, Invalid };

struct Parsed {
    Mode mode = Mode::Invalid;
    std::optional<CmdEncode> enc;
    std::optional<CmdDecode> dec;
    std::optional<CmdBench>  bench;
    std::string help;
};

Parsed parse(int argc, char** argv);

} 
