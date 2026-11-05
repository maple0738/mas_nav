#pragma once

#include <cstdint>
#include <vector>

namespace rog_map {

struct MapSnapshot;

// Geometry from one immutable sensor snapshot. The caller owns the snapshot;
// this view must not outlive it. Temporal confirmation belongs to map history.
class GroundEvidenceView
{
public:
  explicit GroundEvidenceView(const MapSnapshot & source);

  bool measuredSupport(int x, int y) const;
  // Dense support is sufficient for ordinary updates. Sparse enclosing support
  // is considered only when correcting a previously remembered height edge.
  bool continuousSupport(int x, int y, bool allow_sparse) const;
  // Independent precise-height evidence for a suspected quantization edge.
  bool smoothSupport(int x, int y) const;

private:
  bool inside(int x, int y) const;
  const MapSnapshot & source_;
  bool valid_{false};
  std::vector<uint8_t> measured_support_;
};

}  // namespace rog_map
