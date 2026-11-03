#pragma once
#include <Eigen/Eigenvalues>
#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

namespace small_point_lio {
// The ordered, bit-exact support geometry is the cache key. No map pointers,
// voxel-only approximation, or query-dependent residuals are retained.
class PlaneCache {
public:
    struct Fit {
        Eigen::Vector3f normal = Eigen::Vector3f::Zero();
        float offset = 0, maximum_residual = 0;
        bool valid = false;
    };
    void clear() { for (auto &entry : entries_) entry.ready = false; hits = misses = 0; }
    Fit fit(const std::vector<Eigen::Vector3f> &points) {
        if (points.size() != 5) return {};
        std::array<std::uint32_t, 15> key;
        std::size_t hash = 1469598103934665603ULL;
        for (std::size_t i = 0; i < 5; ++i) {
            std::memcpy(key.data() + i * 3, points[i].data(), 3 * sizeof(float));
            for (int j = 0; j < 3; ++j) { hash ^= key[i * 3 + j]; hash *= 1099511628211ULL; }
        }
        auto &entry = entries_[hash % entries_.size()];
        if (entry.ready && entry.key == key) { ++hits; return entry.surface; }
        ++misses;
        Fit surface;
        Eigen::Vector3f center = Eigen::Vector3f::Zero();
        for (const auto &p : points) center.noalias() += p;
        center /= 5.f;
        Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
        for (const auto &p : points) {
            const Eigen::Vector3f delta = p - center;
            covariance.noalias() += delta * delta.transpose();
        }
        covariance /= 4.f;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
        surface.normal = solver.eigenvectors().col(0);
        surface.offset = -surface.normal.dot(center);
        surface.valid = solver.info() == Eigen::Success && surface.normal.allFinite();
        for (const auto &p : points)
            surface.maximum_residual = std::max(surface.maximum_residual,
                std::abs(surface.normal.dot(p) + surface.offset));
        entry = {key, surface, true};
        return surface;
    }
    std::size_t hits = 0, misses = 0;
private:
    struct Entry { std::array<std::uint32_t, 15> key{}; Fit surface; bool ready = false; };
    std::array<Entry, 1024> entries_{};
};
}
