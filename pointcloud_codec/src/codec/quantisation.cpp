#ifndef NOMINMAX
#define NOMINMAX 1
#endif

#include <array>
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <cstddef>
#include <span>

#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

#include "pcc/config.hpp"
#include "codec/quantisation.hpp"

namespace pcc {

Aabb compute_aabb(std::span<const float3> pts)
{
    if (pts.empty()) {
        return Aabb{ float3{0.f,0.f,0.f}, float3{1.f,1.f,1.f} };
    }
    float3 mn = pts[0];
    float3 mx = pts[0];
    for (std::size_t i = 1; i < pts.size(); ++i) {
        const float3& p = pts[i];
        if (p.x < mn.x) mn.x = p.x; if (p.x > mx.x) mx.x = p.x;
        if (p.y < mn.y) mn.y = p.y; if (p.y > mx.y) mx.y = p.y;
        if (p.z < mn.z) mn.z = p.z; if (p.z > mx.z) mx.z = p.z;
    }
    return Aabb{ mn, mx };
}

Aabb clamp_aabb(const Aabb& box, float eps)
{
    Aabb r = box;
    if (eps <= 0.f) eps = 1e-6f;
    if (r.max.x - r.min.x < eps) r.max.x = r.min.x + eps;
    if (r.max.y - r.min.y < eps) r.max.y = r.min.y + eps;
    if (r.max.z - r.min.z < eps) r.max.z = r.min.z + eps;
    return r;
}

void quantise(std::span<const float3> in,
              std::span<std::array<std::uint32_t, 3>> out,
              const Aabb& aabb,
              std::uint32_t qpos_bits)
{
    const std::size_t in_sz  = in.size();
    const std::size_t out_sz = out.size();
    const std::size_t n = (in_sz < out_sz) ? in_sz : out_sz;

    const std::uint32_t Lu = (qpos_bits > 0u) ? ((1u << qpos_bits) - 1u) : 0u;
    const double L = static_cast<double>(Lu);

    const double rx = static_cast<double>(aabb.max.x) - static_cast<double>(aabb.min.x);
    const double ry = static_cast<double>(aabb.max.y) - static_cast<double>(aabb.min.y);
    const double rz = static_cast<double>(aabb.max.z) - static_cast<double>(aabb.min.z);

    auto scale = [&](double v, double mn, double r) -> std::uint32_t {
        if (Lu == 0u || r <= 0.0) return 0u;
        const double t  = (v - mn) / r;
        const double qd = std::round(std::clamp(t, 0.0, 1.0) * L);
        const double qdc = std::clamp(qd, 0.0, L);
        return static_cast<std::uint32_t>(qdc);
    };

    for (std::size_t i = 0; i < n; ++i) {
        const float3& p = in[i];
        std::array<std::uint32_t, 3>& q = out[i];
        q[0] = scale(static_cast<double>(p.x), static_cast<double>(aabb.min.x), rx);
        q[1] = scale(static_cast<double>(p.y), static_cast<double>(aabb.min.y), ry);
        q[2] = scale(static_cast<double>(p.z), static_cast<double>(aabb.min.z), rz);
    }
}

void dequantise(std::span<const std::array<std::uint32_t, 3>> in,
                std::span<float3> out,
                const Aabb& aabb,
                std::uint32_t qpos_bits)
{
    const std::size_t in_sz  = in.size();
    const std::size_t out_sz = out.size();
    const std::size_t n = (in_sz < out_sz) ? in_sz : out_sz;

    const std::uint32_t Lu = (qpos_bits > 0u) ? ((1u << qpos_bits) - 1u) : 0u;
    const double L = static_cast<double>(Lu == 0u ? 1u : Lu); // avoid divide-by-zero

    const double rx = static_cast<double>(aabb.max.x) - static_cast<double>(aabb.min.x);
    const double ry = static_cast<double>(aabb.max.y) - static_cast<double>(aabb.min.y);
    const double rz = static_cast<double>(aabb.max.z) - static_cast<double>(aabb.min.z);

    for (std::size_t i = 0; i < n; ++i) {
        const auto& q = in[i];

        const double nx = (Lu == 0u) ? 0.0 : (static_cast<double>((q[0] <= Lu) ? q[0] : Lu) / L);
        const double ny = (Lu == 0u) ? 0.0 : (static_cast<double>((q[1] <= Lu) ? q[1] : Lu) / L);
        const double nz = (Lu == 0u) ? 0.0 : (static_cast<double>((q[2] <= Lu) ? q[2] : Lu) / L);

        float3& p = out[i];
        p.x = static_cast<float>(static_cast<double>(aabb.min.x) + nx * rx);
        p.y = static_cast<float>(static_cast<double>(aabb.min.y) + ny * ry);
        p.z = static_cast<float>(static_cast<double>(aabb.min.z) + nz * rz);
    }
}

double rmse(std::span<const float3> a, std::span<const float3> b)
{
    const std::size_t n = a.size();
    if (n == 0 || n != b.size()) return 0.0;

    long double acc = 0.0L;
    for (std::size_t i = 0; i < n; ++i) {
        const long double dx = static_cast<long double>(a[i].x) - static_cast<long double>(b[i].x);
        const long double dy = static_cast<long double>(a[i].y) - static_cast<long double>(b[i].y);
        const long double dz = static_cast<long double>(a[i].z) - static_cast<long double>(b[i].z);
        acc += dx * dx + dy * dy + dz * dz;
    }
    return static_cast<double>(std::sqrt(acc / static_cast<long double>(n)));
}

} 

namespace quant {
using pcc::Aabb;
using pcc::float3;

Aabb compute_aabb(std::span<const float3> pts) { return pcc::compute_aabb(pts); }
Aabb clamp_aabb(const Aabb& box, float eps)    { return pcc::clamp_aabb(box, eps); }

} 
