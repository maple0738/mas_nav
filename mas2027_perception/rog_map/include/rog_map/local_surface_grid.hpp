#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace rog_map {

// Bounded XY cache of actual world-Z returns, independent of occupancy voxels.
// Height estimates require raw returns; a ray through empty space is not support.
class LocalSurfaceGrid {
public:
  struct Surface {
    bool valid{false};
    bool conflict{false};
    double height{0}, low{0}, high{0};
  };
  void begin(double x, double y, double stamp) {
    if (stamp < stamp_) cells_.assign(kSize * kSize, Cell{});
    x_ = x; y_ = y; stamp_ = stamp;
  }
  bool add(double x, double y, double z) {
    if (!inside(x, y) || !std::isfinite(z)) return false;
    const int ix = static_cast<int>(std::floor(x / kResolution));
    const int iy = static_cast<int>(std::floor(y / kResolution));
    auto & cell = cells_[hash(ix, iy)];
    if (cell.count == 0 || cell.x != ix || cell.y != iy || stamp_ - cell.first > 1.) {
      cell = Cell{}; cell.x = ix; cell.y = iy; cell.first = stamp_;
    }
    cell.samples[cell.next] = static_cast<float>(z);
    cell.xs[cell.next] = static_cast<float>(x); cell.ys[cell.next] = static_cast<float>(y);
    cell.next = (cell.next + 1) % 8;
    cell.count = std::min(8, cell.count + 1);
    // Keep extremes as a veto, even when the median rejects an outlier.
    cell.low = std::min(cell.low, z); cell.high = std::max(cell.high, z);
    cell.last = stamp_;
    return true;
  }
  Surface surface(double x, double y, double width, double stamp, bool complete = true,
                  bool enclosing = false) const {
    Surface result;
    if (!inside(x, y) || !std::isfinite(width) || width < kResolution || width > .10 ||
        !std::isfinite(stamp) || stamp < stamp_) return result;
    // ROGMap centers are floats. Snap aligned boundaries to tile indices so a
    // few ulps cannot accidentally demand a third tile beyond a 50 mm column.
    const double gx = (x - width * .5) / kResolution;
    const double gy = (y - width * .5) / kResolution;
    const int x0 = static_cast<int>(std::llround(gx));
    const int y0 = static_cast<int>(std::llround(gy));
    const int steps = static_cast<int>(std::llround(width / kResolution));
    if (std::abs(gx - x0) > .002 || std::abs(gy - y0) > .002 ||
        std::abs(width / kResolution - steps) > 1e-5) return result;
    const int x1 = x0 + steps - 1, y1 = y0 + steps - 1;
    result.low = std::numeric_limits<double>::infinity(); result.high = -result.low;
    int count = 0;
    std::array<std::pair<double,double>,128> nearest{};
    std::array<double,128> bearings{};
    double sx=0,sy=0,sz=0,sxx=0,sxy=0,syy=0,sxz=0,syz=0,szz=0;
    int fit_count=0;
    for (int iy = y0; iy <= y1; ++iy) for (int ix = x0; ix <= x1; ++ix) {
      const auto & cell = cells_[hash(ix, iy)];
      if (cell.x != ix || cell.y != iy || cell.count < 3 || stamp - cell.last > 1.) {
        if (complete) return {};
        continue;
      }
      auto sorted = cell.samples;
      std::sort(sorted.begin(), sorted.begin() + cell.count);
      result.height += .5 * (sorted[(cell.count - 1) / 2] + sorted[cell.count / 2]);
      const int trim=cell.count>=6?1:0;
      const double lo=sorted[trim], hi=sorted[cell.count-1-trim];
      const bool veto=cell.high-cell.low>.08;
      result.conflict = result.conflict || veto;
      result.low = std::min(result.low, veto?cell.low:lo);
      result.high = std::max(result.high, veto?cell.high:hi);
      for(int i=0;i<cell.count;++i) {
        const double pz=cell.samples[i];
        if(pz<lo || pz>hi) continue;
        const double px=cell.xs[i]-x, py=cell.ys[i]-y;
        sx+=px;sy+=py;sz+=pz;sxx+=px*px;sxy+=px*py;syy+=py*py;
        sxz+=px*pz;syz+=py*pz;szz+=pz*pz;++fit_count;
        nearest[fit_count-1]={px*px+py*py,pz};
        bearings[fit_count-1]=std::atan2(py,px);
      }
      ++count;
    }
    // Partial support estimates HEIGHT only for an already occupied column;
    // it never proves empty/unknown space traversable. Require distinct tiles.
    if (count < (complete ? 1 : 2)) return result;
    if (!complete) {
      if (fit_count<6) return result;
      const double xx=sxx-sx*sx/fit_count, xy=sxy-sx*sy/fit_count, yy=syy-sy*sy/fit_count;
      const double xz=sxz-sx*sz/fit_count,yz=syz-sy*sz/fit_count,zz=szz-sz*sz/fit_count;
      const double determinant=xx*yy-xy*xy;
      if (determinant<1e-12 || xx/fit_count<1e-6 || yy/fit_count<1e-6) return result;
      const double a=(xz*yy-yz*xy)/determinant,b=(yz*xx-xz*xy)/determinant;
      const double rms=std::sqrt(std::max(0.,zz-a*xz-b*yz)/fit_count);
      if (std::hypot(a,b)>1.25 || rms>.0125) { result.conflict=true; return result; }
      const double centered_height=(sz-a*sx-b*sy)/fit_count;
      // Heights from different columns must refer to their column CENTERS.
      // On a grade, a median of returns near one edge introduces a systematic
      // XY-dependent height bias even when all those returns fit the ground.
      if(centered_height<result.low-1e-6 || centered_height>result.high+1e-6) return result;
      result.height=std::clamp(centered_height,result.low,result.high);
      std::sort(nearest.begin(),nearest.begin()+fit_count);
      if (enclosing) {
        std::sort(bearings.begin(),bearings.begin()+fit_count);
        constexpr double pi=3.14159265358979323846;
        double gap=bearings[0]+2*pi-bearings[fit_count-1];
        for(int i=1;i<fit_count;++i) gap=std::max(gap,bearings[i]-bearings[i-1]);
        if(gap>=pi-1e-6 || nearest[5].first>.04*.04) return result;
      }
    }
    if (complete) result.height /= count;
    result.valid = true;
    return result;
  }
private:
  static constexpr double kResolution = .025;
  static constexpr int kSize = 201;
  struct Cell {
    int x{0}, y{0}, count{0}, next{0};
    double first{0}, last{0};
    double low{std::numeric_limits<double>::infinity()}, high{-std::numeric_limits<double>::infinity()};
    std::array<float, 8> samples{};
    std::array<float, 8> xs{}, ys{};
  };
  static std::size_t hash(int x, int y) {
    return static_cast<std::size_t>((y % kSize + kSize) % kSize) * kSize + (x % kSize + kSize) % kSize;
  }
  bool inside(double x, double y) const {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(stamp_) &&
      std::abs(x) < 1e6 && std::abs(y) < 1e6 && std::abs(x - x_) <= 2.49 && std::abs(y - y_) <= 2.49;
  }
  std::vector<Cell> cells_{static_cast<std::size_t>(kSize * kSize)};
  double x_{0}, y_{0}, stamp_{-std::numeric_limits<double>::infinity()};
};
}  // namespace rog_map
