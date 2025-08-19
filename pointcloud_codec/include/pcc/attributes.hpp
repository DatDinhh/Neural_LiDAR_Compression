// include/pcc/attributes.hpp
#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace pcc {

enum class AttrId : std::uint8_t {
    Position  = 0,
    Normal    = 1,
    ColorRGB  = 2,
    Intensity = 3,

    User0     = 16
};

enum class AttrType : std::uint8_t {
    F32  = 0,   // float32
    U8   = 1,   // uint8_t
    U16  = 2,   // uint16_t
    I16  = 3    // int16_t
};

struct AttrSpec {
    AttrId   id{AttrId::Position};
    AttrType type{AttrType::F32};
    int      dims{3};           
    int      quant_bits{0};    
    bool     present{true};


    const char* name{"position"};
};

struct AttributeSet {
    AttrSpec position{};               
    std::vector<AttrSpec> extras;     
};

} 
