#pragma once

#include "gpu_accel_core.h"
#include "utils/types.h"

#include <chrono>
#include <pcl/point_types.h>

namespace fast_livo_gpu
{
inline void copyMatrixRowMajor(const Eigen::Matrix3d &m, double out[9])
{
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      out[r * 3 + c] = m(r, c);
}

inline bool transformPointCloud(const PointCloudXYZI::Ptr &input,
                                PointCloudXYZI::Ptr &output,
                                const Eigen::Matrix3d &rot,
                                const Eigen::Vector3d &t,
                                const Eigen::Matrix3d &extR,
                                const Eigen::Vector3d &extT,
                                float *kernel_ms = nullptr,
                                double *total_ms = nullptr)
{
  if (!input) return false;
  const std::size_t n = input->size();
  if (!shouldUse(n)) return false;

  FastLivoGpuPoint4 *gpu_in = nullptr;
  FastLivoGpuPoint4 *gpu_out = nullptr;
  if (!prepare(n, &gpu_in, &gpu_out)) return false;

  const auto t0 = std::chrono::steady_clock::now();

#ifdef MP_EN
#pragma omp parallel for
#endif
  for (int i = 0; i < static_cast<int>(n); ++i)
  {
    const auto &p = input->points[i];
    gpu_in[i].x = p.x;
    gpu_in[i].y = p.y;
    gpu_in[i].z = p.z;
    gpu_in[i].intensity = p.intensity;
  }

  double R[9], ER[9], T[3] = {t.x(), t.y(), t.z()},
                         ET[3] = {extT.x(), extT.y(), extT.z()};
  copyMatrixRowMajor(rot, R);
  copyMatrixRowMajor(extR, ER);

  float k_ms = 0.0f;
  if (!executeTransform(n, R, T, ER, ET, &k_ms)) return false;

  output->points.resize(n);
  output->width = static_cast<uint32_t>(n);
  output->height = 1;
  output->is_dense = input->is_dense;

#ifdef MP_EN
#pragma omp parallel for
#endif
  for (int i = 0; i < static_cast<int>(n); ++i)
  {
    auto &p = output->points[i];
    p.x = gpu_out[i].x;
    p.y = gpu_out[i].y;
    p.z = gpu_out[i].z;
    p.intensity = gpu_out[i].intensity;
  }

  if (kernel_ms) *kernel_ms = k_ms;
  if (total_ms)
  {
    const auto t1 = std::chrono::steady_clock::now();
    *total_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  }
  return true;
}

inline bool transformPointCloudXYZI(const PointCloudXYZI::Ptr &input,
                                    pcl::PointCloud<pcl::PointXYZI>::Ptr &output,
                                    const Eigen::Matrix3d &rot,
                                    const Eigen::Vector3d &t,
                                    const Eigen::Matrix3d &extR,
                                    const Eigen::Vector3d &extT,
                                    float *kernel_ms = nullptr,
                                    double *total_ms = nullptr)
{
  if (!input) return false;
  const std::size_t n = input->size();
  if (!shouldUse(n)) return false;

  FastLivoGpuPoint4 *gpu_in = nullptr;
  FastLivoGpuPoint4 *gpu_out = nullptr;
  if (!prepare(n, &gpu_in, &gpu_out)) return false;

  const auto t0 = std::chrono::steady_clock::now();

#ifdef MP_EN
#pragma omp parallel for
#endif
  for (int i = 0; i < static_cast<int>(n); ++i)
  {
    const auto &p = input->points[i];
    gpu_in[i].x = p.x;
    gpu_in[i].y = p.y;
    gpu_in[i].z = p.z;
    gpu_in[i].intensity = p.intensity;
  }

  double R[9], ER[9], T[3] = {t.x(), t.y(), t.z()},
                         ET[3] = {extT.x(), extT.y(), extT.z()};
  copyMatrixRowMajor(rot, R);
  copyMatrixRowMajor(extR, ER);

  float k_ms = 0.0f;
  if (!executeTransform(n, R, T, ER, ET, &k_ms)) return false;

  output->points.resize(n);
  output->width = static_cast<uint32_t>(n);
  output->height = 1;
  output->is_dense = input->is_dense;

#ifdef MP_EN
#pragma omp parallel for
#endif
  for (int i = 0; i < static_cast<int>(n); ++i)
  {
    auto &p = output->points[i];
    p.x = gpu_out[i].x;
    p.y = gpu_out[i].y;
    p.z = gpu_out[i].z;
    p.intensity = gpu_out[i].intensity;
  }

  if (kernel_ms) *kernel_ms = k_ms;
  if (total_ms)
  {
    const auto t1 = std::chrono::steady_clock::now();
    *total_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  }
  return true;
}
} // namespace fast_livo_gpu
