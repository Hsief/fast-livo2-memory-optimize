#pragma once

#include <cstddef>

struct FastLivoGpuPoint4
{
  float x;
  float y;
  float z;
  float intensity;
};

namespace fast_livo_gpu
{
// Configure once from ROS parameters. Returns true only when CUDA is both
// compiled in and a runtime device is available.
bool configure(bool requested, std::size_t min_points);
bool isEnabled();
bool shouldUse(std::size_t count);

// Managed buffers are persistent and grow only when necessary. They are used
// by the estimator thread sequentially.
bool prepare(std::size_t count, FastLivoGpuPoint4 **input, FastLivoGpuPoint4 **output);

// Execute the exact transform structure used by FAST-LIVO2:
//   p_out = rot * (extR * p_in + extT) + t
bool executeTransform(std::size_t count,
                      const double rot[9],
                      const double t[3],
                      const double extR[9],
                      const double extT[3],
                      float *kernel_ms);

const char *deviceName();
const char *lastError();
} // namespace fast_livo_gpu
