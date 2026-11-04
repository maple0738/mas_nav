#pragma once

#include <array>
#include <cmath>
#include <rog_map/terrain_config.hpp>

namespace rog_map {

// Recognize repeated short waves, not a single bump or a monotonic grade.
// Heights must be measured support surfaces, sampled at a constant spacing.
template<std::size_t N>
inline bool isUndulatingProfile(std::array<double, N> heights, double spacing,
  const UndulatingRecognitionConfig & config = {})
{
  static_assert(N==13 || N==17);
  if (!std::isfinite(spacing) || spacing <= 0.0 || spacing > 0.08) return false;
  // Shape inference only: bridge at most two absent samples between measured
  // heights. No map occupancy or support mask is created by this interpolation.
  int known = 0, first = -1, last = -1;
  for (int i=0;i<static_cast<int>(N);++i) if (std::isfinite(heights[i])) {
    ++known; if(first<0) first=i; last=i;
  }
  if (known<(N==13?9:12) || first>2 || last<static_cast<int>(N)-3) return false;
  for (int i=0;i<first;++i) heights[i]=heights[first];
  for (int i=last+1;i<static_cast<int>(N);++i) heights[i]=heights[last];
  for (int left=first;left<last;) {
    int right=left+1;
    while(right<=last && !std::isfinite(heights[right])) ++right;
    if(right-left>3) return false;
    for(int i=left+1;i<right;++i)
      heights[i]=heights[left]+(heights[right]-heights[left])*(i-left)/(right-left);
    left=right;
  }
  double mean = 0.0, trend = 0.0;
  const double center=(N-1)*.5, denominator=N*(N*N-1)/12.;
  for (int i = 0; i < static_cast<int>(N); ++i) {
    if (!std::isfinite(heights[i])) return false;
    mean += heights[i] / N;
    trend += (i - center) * heights[i] / denominator;
  }
  if (std::abs(trend / spacing) > config.max_trend_gradient) return false;
  for (int i = 0; i < static_cast<int>(N); ++i) heights[i] -= mean + trend * (i - center);
  std::array<int, N> extrema{};
  int count = 0, previous_sign = 0;
  for (int i = 1; i < static_cast<int>(N); ++i) {
    const double delta = heights[i] - heights[i - 1];
    const int sign = delta > config.min_delta_m ? 1 : delta < -config.min_delta_m ? -1 : 0;
    if (!sign) continue;
    if (previous_sign && sign != previous_sign) extrema[count++] = i - 1;
    previous_sign = sign;
  }
  if (N==13 && count==2 && last>extrema[count-1] &&
      std::abs(heights[last]-heights[extrema[count-1]])>=config.min_peak_to_trough_m)
    extrema[count++]=last;
  if (count < (N==13 ? 3 : 5)) return false;
  for (int i = 1; i < count; ++i) {
    const double height = std::abs(heights[extrema[i]] - heights[extrema[i - 1]]);
    if (height < config.min_peak_to_trough_m || height > config.max_peak_to_trough_m) return false;
    if (i >= 2) {
      const double wavelength = (extrema[i] - extrema[i - 2]) * spacing;
      if (wavelength < config.min_wavelength_m || wavelength > config.max_wavelength_m) return false;
    }
  }
  return true;
}

}  // namespace rog_map
