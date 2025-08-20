// include/pcc/version.hpp
#pragma once
#include <cstdint>
#include <string>

#define PCC_VERSION_MAJOR 0
#define PCC_VERSION_MINOR 1
#define PCC_VERSION_PATCH 0

#define PCC_VERSION_IS_PRERELEASE 1

namespace pcc {

inline constexpr std::uint32_t version_number() noexcept {
    return (static_cast<std::uint32_t>(PCC_VERSION_MAJOR) << 24) |
           (static_cast<std::uint32_t>(PCC_VERSION_MINOR) << 16) |
           (static_cast<std::uint32_t>(PCC_VERSION_PATCH) <<  8) |
           static_cast<std::uint32_t>(PCC_VERSION_IS_PRERELEASE);
}

inline std::string version_string() {
    std::string s = std::to_string(PCC_VERSION_MAJOR) + "." +
                    std::to_string(PCC_VERSION_MINOR) + "." +
                    std::to_string(PCC_VERSION_PATCH);
    if (PCC_VERSION_IS_PRERELEASE) s += "-pre";
    return s;
}

}
