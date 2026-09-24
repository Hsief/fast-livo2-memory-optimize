#include "gpu_accel_core.h"

#include <cuda_runtime.h>

#include <algorithm>
#include <cstdio>
#include <mutex>
#include <string>

namespace
{
std::mutex g_mutex;
bool g_requested = false;
bool g_enabled = false;
std::size_t g_min_points = 2048;
std::size_t g_capacity = 0;
FastLivoGpuPoint4 *g_input = nullptr;
FastLivoGpuPoint4 *g_output = nullptr;
std::string g_device_name = "CUDA unavailable";
std::string g_last_error;
cudaEvent_t g_start = nullptr;
cudaEvent_t g_stop = nullptr;

void setError(const char *where, cudaError_t err)
{
  g_last_error = std::string(where) + ": " + cudaGetErrorString(err);
}

bool ensureCapacity(std::size_t count)
{
  if (count <= g_capacity && g_input && g_output) return true;

  std::size_t new_capacity = std::max<std::size_t>(4096, g_capacity ? g_capacity : 4096);
  while (new_capacity < count) new_capacity *= 2;

  if (g_input) cudaFree(g_input);
  if (g_output) cudaFree(g_output);
  g_input = nullptr;
  g_output = nullptr;
  g_capacity = 0;

  cudaError_t err = cudaMallocManaged(reinterpret_cast<void **>(&g_input), new_capacity * sizeof(FastLivoGpuPoint4));
  if (err != cudaSuccess)
  {
    setError("cudaMallocManaged(input)", err);
    return false;
  }

  err = cudaMallocManaged(reinterpret_cast<void **>(&g_output), new_capacity * sizeof(FastLivoGpuPoint4));
  if (err != cudaSuccess)
  {
    setError("cudaMallocManaged(output)", err);
    cudaFree(g_input);
    g_input = nullptr;
    return false;
  }

  g_capacity = new_capacity;
  return true;
}

struct TransformParams
{
  double rot[9];
  double t[3];
  double extR[9];
  double extT[3];
};

__global__ void transformKernel(const FastLivoGpuPoint4 *input,
                                FastLivoGpuPoint4 *output,
                                std::size_t count,
                                TransformParams p)
{
  const std::size_t i =
      static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (i >= count) return;

  const double x = static_cast<double>(input[i].x);
  const double y = static_cast<double>(input[i].y);
  const double z = static_cast<double>(input[i].z);

  // Preserve the original expression order:
  //   p = rot * (extR * p + extT) + t
  const double ex = p.extR[0] * x + p.extR[1] * y + p.extR[2] * z + p.extT[0];
  const double ey = p.extR[3] * x + p.extR[4] * y + p.extR[5] * z + p.extT[1];
  const double ez = p.extR[6] * x + p.extR[7] * y + p.extR[8] * z + p.extT[2];

  FastLivoGpuPoint4 q;
  q.x = static_cast<float>(p.rot[0] * ex + p.rot[1] * ey + p.rot[2] * ez + p.t[0]);
  q.y = static_cast<float>(p.rot[3] * ex + p.rot[4] * ey + p.rot[5] * ez + p.t[1]);
  q.z = static_cast<float>(p.rot[6] * ex + p.rot[7] * ey + p.rot[8] * ez + p.t[2]);
  q.intensity = input[i].intensity;
  output[i] = q;
}
} // namespace

namespace fast_livo_gpu
{
bool configure(bool requested, std::size_t min_points)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  g_requested = requested;
  g_min_points = std::max<std::size_t>(1, min_points);
  g_enabled = false;
  g_last_error.clear();

  if (!requested)
  {
    g_device_name = "disabled by config";
    return false;
  }

  int count = 0;
  cudaError_t err = cudaGetDeviceCount(&count);
  if (err != cudaSuccess || count <= 0)
  {
    if (err != cudaSuccess) setError("cudaGetDeviceCount", err);
    else g_last_error = "no CUDA device";
    return false;
  }

  cudaDeviceProp prop;
  err = cudaGetDeviceProperties(&prop, 0);
  if (err != cudaSuccess)
  {
    setError("cudaGetDeviceProperties", err);
    return false;
  }

  err = cudaSetDevice(0);
  if (err != cudaSuccess)
  {
    setError("cudaSetDevice", err);
    return false;
  }

  if (!g_start) cudaEventCreate(&g_start);
  if (!g_stop) cudaEventCreate(&g_stop);

  g_device_name = prop.name;
  g_enabled = true;
  return true;
}

bool isEnabled()
{
  return g_enabled;
}

bool shouldUse(std::size_t count)
{
  return g_enabled && g_requested && count >= g_min_points;
}

bool prepare(std::size_t count, FastLivoGpuPoint4 **input, FastLivoGpuPoint4 **output)
{
  if (!shouldUse(count) || !input || !output) return false;
  std::lock_guard<std::mutex> lock(g_mutex);
  if (!ensureCapacity(count))
  {
    g_enabled = false;
    return false;
  }
  *input = g_input;
  *output = g_output;
  return true;
}

bool executeTransform(std::size_t count,
                      const double rot[9],
                      const double t[3],
                      const double extR[9],
                      const double extT[3],
                      float *kernel_ms)
{
  if (!shouldUse(count)) return false;

  TransformParams p;
  for (int i = 0; i < 9; ++i)
  {
    p.rot[i] = rot[i];
    p.extR[i] = extR[i];
  }
  for (int i = 0; i < 3; ++i)
  {
    p.t[i] = t[i];
    p.extT[i] = extT[i];
  }

  cudaError_t err = cudaEventRecord(g_start);
  if (err != cudaSuccess)
  {
    setError("cudaEventRecord(start)", err);
    g_enabled = false;
    return false;
  }

  constexpr int threads = 256;
  const int blocks = static_cast<int>((count + threads - 1) / threads);
  transformKernel<<<blocks, threads>>>(g_input, g_output, count, p);

  err = cudaGetLastError();
  if (err != cudaSuccess)
  {
    setError("transformKernel launch", err);
    g_enabled = false;
    return false;
  }

  err = cudaEventRecord(g_stop);
  if (err != cudaSuccess)
  {
    setError("cudaEventRecord(stop)", err);
    g_enabled = false;
    return false;
  }

  err = cudaEventSynchronize(g_stop);
  if (err != cudaSuccess)
  {
    setError("cudaEventSynchronize", err);
    g_enabled = false;
    return false;
  }

  if (kernel_ms)
  {
    float ms = 0.0f;
    cudaEventElapsedTime(&ms, g_start, g_stop);
    *kernel_ms = ms;
  }
  return true;
}

const char *deviceName()
{
  return g_device_name.c_str();
}

const char *lastError()
{
  return g_last_error.c_str();
}
} // namespace fast_livo_gpu
