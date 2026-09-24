#include "gpu_accel_core.h"

namespace fast_livo_gpu
{
bool configure(bool, std::size_t) { return false; }
bool isEnabled() { return false; }
bool shouldUse(std::size_t) { return false; }
bool prepare(std::size_t, FastLivoGpuPoint4 **, FastLivoGpuPoint4 **) { return false; }
bool executeTransform(std::size_t, const double[9], const double[3],
                      const double[9], const double[3], float *) { return false; }
const char *deviceName() { return "CUDA not compiled"; }
const char *lastError() { return "FAST-LIVO2 built without CUDA"; }
} // namespace fast_livo_gpu
