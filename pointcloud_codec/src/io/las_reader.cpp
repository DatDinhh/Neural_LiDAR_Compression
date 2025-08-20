// io/las_reader.cpp
#include "las_reader.hpp"
#include <stdexcept>

#if __has_include(<pdal/pdal.hpp>)
  #define PCC_WITH_PDAL 1
  #include <pdal/pdal.hpp>
  #include <pdal/StageFactory.hpp>
  #include <pdal/PointTable.hpp>
  #include <pdal/PointView.hpp>
  #include <pdal/io/LasReader.hpp>
#else
  #define PCC_WITH_PDAL 0
#endif

namespace pcc::io {

bool read_las_positions(const std::string& path,
                        std::vector<float3>& out,
                        std::size_t max_points) {
#if PCC_WITH_PDAL
    out.clear();

    pdal::LasReader reader;
    reader.setFilename(path);

    pdal::Options opts;
    reader.setOptions(opts);

    pdal::PointTable table;
    reader.prepare(table);
    pdal::PointViewSet pvs = reader.execute(table);

    std::size_t total = 0;
    for (auto* view : pvs) {
        const std::size_t n = view->size();
        out.reserve(out.size() + n);
        for (std::size_t i = 0; i < n; ++i) {
            float3 p;
            p.x = static_cast<float>(view->getFieldAs<double>(pdal::Dimension::Id::X, i));
            p.y = static_cast<float>(view->getFieldAs<double>(pdal::Dimension::Id::Y, i));
            p.z = static_cast<float>(view->getFieldAs<double>(pdal::Dimension::Id::Z, i));
            out.push_back(p);
            ++total;
            if (max_points > 0 && total >= max_points) return true;
        }
    }
    return !out.empty();
#else
    (void)path; (void)out; (void)max_points;
    throw std::runtime_error("LAS/LAZ reading requires PDAL; rebuild with PDAL or use PCD.");
#endif
}

} 
