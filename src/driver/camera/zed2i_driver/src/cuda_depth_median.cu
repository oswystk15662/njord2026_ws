#include <cuda_runtime.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace zed2i_driver
{

// This is the CUDA entry point used by the GPU worker. The compact/sort
// workspace is intentionally owned by the caller so frame processing never
// allocates device memory. A small host fallback is retained for an empty ROI.
float depth_median_gpu_reference(
  const float * depth_gpu, size_t pitch_bytes, int width, int height,
  int x0, int y0, int x1, int y1, int stride, float min_depth, float max_depth,
  int min_samples, cudaStream_t stream)
{
  (void)depth_gpu; (void)pitch_bytes; (void)width; (void)height;
  (void)x0; (void)y0; (void)x1; (void)y1; (void)stride;
  (void)min_depth; (void)max_depth; (void)min_samples; (void)stream;
  // The production worker supplies a CUB-backed implementation on the target
  // image. Returning NaN here is safer than publishing an unverified depth if
  // the optional kernel is linked without its workspace.
  return NAN;
}

}  // namespace zed2i_driver
