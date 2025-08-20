// cuda/quantize.cu
#include <cuda_runtime.h>
#include <cstdint>
#include "pcc/config.hpp"

extern "C" {

__global__ void pcc_quantize_xyz_kernel(const pcc::float3* __restrict__ in,
                                        std::uint32_t* __restrict__ out, // 3*N
                                        int N,
                                        float3 aabb_min,
                                        float3 aabb_max,
                                        int bits)
{
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= N) return;

    const float L = (1u << bits) - 1u;
    const float rx = fmaxf(aabb_max.x - aabb_min.x, 1e-20f);
    const float ry = fmaxf(aabb_max.y - aabb_min.y, 1e-20f);
    const float rz = fmaxf(aabb_max.z - aabb_min.z, 1e-20f);

    const float3 p = make_float3(in[i].x, in[i].y, in[i].z);

    const float qx = roundf(fminf(fmaxf((p.x - aabb_min.x)/rx, 0.f), 1.f) * L);
    const float qy = roundf(fminf(fmaxf((p.y - aabb_min.y)/ry, 0.f), 1.f) * L);
    const float qz = roundf(fminf(fmaxf((p.z - aabb_min.z)/rz, 0.f), 1.f) * L);

    out[3*i + 0] = static_cast<std::uint32_t>(qx);
    out[3*i + 1] = static_cast<std::uint32_t>(qy);
    out[3*i + 2] = static_cast<std::uint32_t>(qz);
}

} 
