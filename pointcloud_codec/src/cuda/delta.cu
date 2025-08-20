// cuda/delta.cu
#include <cuda_runtime.h>
#include <cstdint>

extern "C" {

__global__ void pcc_delta_u32_triplets_kernel(const std::uint32_t* __restrict__ in,
                                              std::uint32_t* __restrict__ out,
                                              int N) {
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= N) return;

    if (i == 0) {
        out[0] = in[0];
        out[1] = in[1];
        out[2] = in[2];
        return;
    }
    const int k = 3*i;
    const int km = 3*(i-1);
    out[k+0] = in[k+0] - in[km+0];
    out[k+1] = in[k+1] - in[km+1];
    out[k+2] = in[k+2] - in[km+2];
}

} 
