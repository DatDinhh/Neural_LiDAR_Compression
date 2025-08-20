// tools/drc_info.cpp
#include <draco/compression/point_cloud/point_cloud_decoder.h>
#if __has_include(<draco/compression/decode.h>)
  #include <draco/compression/decode.h>
  #define DRACO_NEW_API 1
#else
  #define DRACO_NEW_API 0
#endif

#include <fstream>
#include <iostream>
#include <vector>
#include <cstdint>

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: drc_info <file.drc>\n";
        return 1;
    }
    const char* path = argv[1];

    std::ifstream ifs(path, std::ios::binary);
    if (!ifs) { std::cerr << "Cannot open: " << path << "\n"; return 1; }

    std::vector<char> buf((std::istreambuf_iterator<char>(ifs)), {});
    if (buf.size() < 4) { std::cerr << "File too small.\n"; return 1; }

    draco::DecoderBuffer dbuf;
    dbuf.Init(buf.data(), buf.size());

#if DRACO_NEW_API
    auto res = draco::DecodePointCloudFromBuffer(&dbuf);
    if (!res.ok()) {
        std::cerr << "Not a Draco point cloud or decode failed.\n";
        return 1;
    }
    std::unique_ptr<draco::PointCloud> pc(res.value().release());
#else
    draco::Decoder decoder;
    auto res = decoder.DecodePointCloudFromBuffer(&dbuf);
    if (!res.ok()) {
        std::cerr << "Not a Draco point cloud or decode failed.\n";
        return 1;
    }
    std::unique_ptr<draco::PointCloud> pc(res.value().release());
#endif

    std::cout << "Decoded OK\n";
    std::cout << "Points: " << pc->num_points() << "\n";
    std::cout << "Attributes:\n";
    for (int i = 0; i < pc->num_attributes(); ++i) {
        const auto* a = pc->attribute(i);
        std::cout << "  [" << i << "] semantic=" << a->attribute_type()
                  << " comp_count=" << a->num_components()
                  << " dt=" << a->data_type() << "\n";
    }
    return 0;
}
