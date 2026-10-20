#ifndef FAST_CALIB_LIDAR_HOLES_H
#define FAST_CALIB_LIDAR_HOLES_H

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>
#include <Eigen/Cholesky>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace lidar_holes {
using Cloud = pcl::PointCloud<pcl::PointXYZ>;

struct Options {
    int min_holes = 3;
    int min_inliers = 20;
    double fit_distance = 0.01;
    double radius_tolerance = 0.02;
    double geometry_tolerance = 0.03;
    double max_interior_density = 0.05;
    int max_interior_points = 2;
    int min_surround_sectors = 6;
    double max_angular_gap_deg = 180.0;
    double max_center_shift = 0.005;
};

struct SearchStats {
    std::size_t fits = 0;
    std::size_t rejected_radius = 0;
    std::size_t rejected_interior = 0;
    std::size_t rejected_duplicate = 0;
    std::size_t rejected_coverage = 0;
    std::size_t rejected_stability = 0;
    Cloud rejected_centers;
};

struct CircleCandidate {
    pcl::PointXYZ center;
    double error;
    std::size_t support;
    double max_gap_deg = 360.0;
    double center_shift = 0.0;
};

inline std::size_t angularSector(double x, double y)
{
    const double pi = std::acos(-1.0);
    return std::min<std::size_t>(7, (std::atan2(y, x) + pi) * 8.0 / (2.0 * pi));
}

inline double distance(const pcl::PointXYZ& a, const pcl::PointXYZ& b)
{
    return std::hypot(a.x - b.x, a.y - b.y);
}

// Compare spatial return densities, using the full board rather than edges.
// A fixed point-count cutoff rejects real holes as more scans accumulate.
// The inner disk must have at most 5% of the surrounding board's density;
// require board returns around the hole as well, to reject exterior arcs.
inline bool emptyInterior(const Cloud& board, const pcl::PointXYZ& center, double radius,
                          const Options& options = Options{})
{
    std::size_t interior = 0, surrounding = 0;
    std::array<std::size_t, 8> sectors{};
    for (const auto& point : board) {
        const double d = distance(point, center);
        if (d < 0.8 * radius)
            ++interior;
        if (d > 1.1 * radius && d < 1.6 * radius) {
            ++surrounding;
            ++sectors[angularSector(point.x - center.x, point.y - center.y)];
        }
    }
    const auto covered = std::count_if(sectors.begin(), sectors.end(),
        [](std::size_t n) { return n >= 2; });
    if (surrounding < static_cast<std::size_t>(options.min_inliers) || covered < options.min_surround_sectors)
        return false;
    // Areas divided by pi * radius^2: inner disk 0.64, annulus 1.35.
    const double density_ratio = interior * 1.35 / (surrounding * 0.64);
    return interior <= static_cast<std::size_t>(options.max_interior_points) ||
           density_ratio <= options.max_interior_density;
}

inline bool distinctCenter(const Cloud& centers, const pcl::PointXYZ& center, double radius)
{
    for (const auto& previous : centers) {
        if (distance(previous, center) < 2.0 * radius)
            return false;
    }
    return true;
}

inline double angularGap(const Cloud& support, const pcl::PointXYZ& center)
{
    std::vector<double> angles;
    for (const auto& point : support)
        angles.push_back(std::atan2(point.y - center.y, point.x - center.x));
    if (angles.empty()) return 360.0;
    std::sort(angles.begin(), angles.end());
    const double pi = std::acos(-1.0);
    double max_gap = angles.front() + 2.0 * pi - angles.back();
    for (std::size_t i = 1; i < angles.size(); ++i)
        max_gap = std::max(max_gap, angles[i] - angles[i - 1]);
    return max_gap * 180.0 / pi;
}

// Fit the known physical radius. Balance angular sectors so a dense scan arc
// cannot outweigh the opposite rim. The support is fixed during each fit.
inline bool fitCenter(const Cloud& support, double radius, double scale,
                      pcl::PointXYZ& center)
{
    if (support.size() < 3) return false;
    std::array<std::size_t, 8> counts{};
    std::vector<std::size_t> sectors;
    for (const auto& p : support) {
        sectors.push_back(angularSector(p.x - center.x, p.y - center.y));
        ++counts[sectors.back()];
    }
    Eigen::Vector2d c(center.x, center.y);
    for (int iteration = 0; iteration < 100; ++iteration) {
        Eigen::Matrix2d h = Eigen::Matrix2d::Zero();
        Eigen::Vector2d g = Eigen::Vector2d::Zero();
        for (std::size_t i = 0; i < support.size(); ++i) {
            const Eigen::Vector2d v = c - Eigen::Vector2d(support[i].x, support[i].y);
            const double d = v.norm();
            if (!std::isfinite(d) || d < 1e-9) return false;
            const double residual = d - radius;
            const double weight = 1.0 / (counts[sectors[i]] *
                (1.0 + residual * residual / (scale * scale)));
            const Eigen::Vector2d j = v / d;
            h += weight * j * j.transpose();
            g += weight * j * residual;
        }
        if (h.determinant() < 1e-10) return false;
        Eigen::Vector2d step = h.ldlt().solve(g);
        if (!step.allFinite()) return false;
        if (step.norm() > scale) step *= scale / step.norm();
        c -= step;
        if (step.norm() < 1e-7) {
            center = pcl::PointXYZ(c.x(), c.y(), 0.0f);
            return true;
        }
    }
    return false;
}

enum class Refinement { valid, fit, coverage, stability };

inline Refinement refineCandidate(const Cloud& edges, pcl::PointXYZ seed, double radius,
                                  const Options& options, CircleCandidate& result)
{
    Cloud pool;
    for (const auto& p : edges)
        if (std::abs(distance(p, seed) - radius) < options.radius_tolerance + options.fit_distance)
            pool.push_back(p);
    pcl::PointXYZ center = seed;
    if (pool.size() < static_cast<std::size_t>(options.min_inliers) ||
        !fitCenter(pool, radius, options.fit_distance, center)) return Refinement::fit;
    Cloud support;
    for (const auto& p : pool)
        if (std::abs(distance(p, center) - radius) < options.fit_distance)
            support.push_back(p);
    // This final support, rather than arbitrary board material, determines
    // the center, angular coverage and leave-one-sector-out stability.
    if (support.size() < static_cast<std::size_t>(options.min_inliers) ||
        !fitCenter(support, radius, options.fit_distance, center) ||
        distance(center, seed) > 2.0 * options.radius_tolerance) return Refinement::fit;
    const double gap = angularGap(support, center);
    if (gap >= options.max_angular_gap_deg) return Refinement::coverage;
    double squared_error = 0.0;
    for (const auto& p : support) {
        const double error = distance(p, center) - radius;
        squared_error += error * error;
    }
    const double error = std::sqrt(squared_error / support.size());
    if (error > options.fit_distance) return Refinement::fit;
    double max_shift = 0.0;
    for (std::size_t sector = 0; sector < 8; ++sector) {
        Cloud subset;
        for (const auto& p : support)
            if (angularSector(p.x - center.x, p.y - center.y) != sector)
                subset.push_back(p);
        if (subset.size() == support.size()) continue;
        pcl::PointXYZ trial = center;
        if (!fitCenter(subset, radius, options.fit_distance, trial)) return Refinement::stability;
        max_shift = std::max(max_shift, distance(trial, center));
    }
    if (max_shift > options.max_center_shift) return Refinement::stability;
    result = {center, error, support.size(), gap, max_shift};
    return Refinement::valid;
}

inline void extractCandidates(Cloud::Ptr remaining, const Cloud& board,
                              double radius, std::vector<CircleCandidate>& candidates,
                              SearchStats* stats = nullptr, const Options& options = Options{},
                              const Cloud* all_edges = nullptr)
{
    if (!std::isfinite(radius) || radius <= 0.0)
        return;
    // A cluster may contain several circles. Always remove the fitted inliers,
    // including rejected scan rings, so they cannot hide subsequent holes.
    while (remaining->size() >= static_cast<std::size_t>(options.min_inliers)) {
        pcl::ModelCoefficients coefficients;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CIRCLE2D);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(options.fit_distance);
        seg.setMaxIterations(1000);
        seg.setRadiusLimits(std::max(0.0, radius - options.radius_tolerance), radius + options.radius_tolerance);
        seg.setInputCloud(remaining);
        seg.segment(*inliers, coefficients);
        if (inliers->indices.size() < static_cast<std::size_t>(options.min_inliers))
            break;
        if (stats)
            ++stats->fits;

        if (coefficients.values.size() >= 3 &&
            std::isfinite(coefficients.values[0]) &&
            std::isfinite(coefficients.values[1]) &&
            std::isfinite(coefficients.values[2]) &&
            std::abs(coefficients.values[2] - radius) < options.radius_tolerance) {
            const pcl::PointXYZ seed(coefficients.values[0], coefficients.values[1], 0.0f);
            CircleCandidate candidate;
            const auto refined = refineCandidate(all_edges ? *all_edges : *remaining,
                seed, radius, options, candidate);
            bool accepted = false;
            if (refined == Refinement::fit) {
                if (stats) ++stats->rejected_radius;
            } else if (refined == Refinement::coverage) {
                if (stats) ++stats->rejected_coverage;
            } else if (refined == Refinement::stability) {
                if (stats) ++stats->rejected_stability;
            } else if (!emptyInterior(board, candidate.center, radius, options)) {
                if (stats) ++stats->rejected_interior;
            } else {
                candidates.push_back(candidate);
                accepted = true;
            }
            if (stats && !accepted) stats->rejected_centers.push_back(seed);
        } else if (stats) {
            ++stats->rejected_radius;
        }

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(remaining);
        extract.setIndices(inliers);
        extract.setNegative(true);
        Cloud::Ptr next(new Cloud);
        extract.filter(*next);
        remaining = next;
    }
}

// Summarize nearby fits for visualization only. Geometry selection must use
// all validated hypotheses: fit error alone cannot identify the true center.
inline Cloud distinctCandidates(std::vector<CircleCandidate> candidates, double radius,
                                SearchStats* stats = nullptr)
{
    std::stable_sort(candidates.begin(), candidates.end(),
        [](const CircleCandidate& a, const CircleCandidate& b) {
            if (a.error != b.error) return a.error < b.error;
            return a.support > b.support;
        });
    Cloud centers;
    for (const auto& candidate : candidates) {
        if (distinctCenter(centers, candidate.center, radius))
            centers.push_back(candidate.center);
        else if (stats)
            ++stats->rejected_duplicate;
    }
    return centers;
}

// Test all assignments of the four corners. Checking all six distances with
// their corner identities enforces both opposite sides and both diagonals.
// Keep searching after an invalid quartet and choose the lowest-error set.
inline bool selectTarget(const Cloud& candidates, double width, double height,
                         double radius, Cloud& selected, const Options& options = Options{})
{
    selected.clear();
    if (!std::isfinite(width) || !std::isfinite(height) ||
        !std::isfinite(radius) || radius <= 0.0 ||
        width <= 2 * radius || height <= 2 * radius)
        return false;
    const double diagonal = std::hypot(width, height);
    double best = std::numeric_limits<double>::infinity();
    std::array<std::size_t, 4> best_indices{};
    for (std::size_t a = 0; a < candidates.size(); ++a)
    for (std::size_t b = 0; b < candidates.size(); ++b) {
        if (b == a || std::abs(distance(candidates[a], candidates[b]) - width) > options.geometry_tolerance)
            continue;
        for (std::size_t c = 0; c < candidates.size(); ++c) {
            if (c == a || c == b ||
                std::abs(distance(candidates[a], candidates[c]) - height) > options.geometry_tolerance ||
                std::abs(distance(candidates[b], candidates[c]) - diagonal) > options.geometry_tolerance)
                continue;
            for (std::size_t d = a + 1; d < candidates.size(); ++d) {
                if (d == b || d == c)
                    continue;
                const std::array<std::size_t, 4> indices{{a, b, c, d}};
                const std::array<double, 6> expected{{width, height, diagonal, diagonal, height, width}};
                double score = 0.0;
                std::size_t pair = 0;
                bool valid = true;
                for (std::size_t i = 0; i < 4; ++i)
                for (std::size_t j = i + 1; j < 4; ++j) {
                    const double actual = distance(candidates[indices[i]], candidates[indices[j]]);
                    const double error = std::abs(actual - expected[pair++]);
                    if (!std::isfinite(actual) || actual < 2 * radius || error > options.geometry_tolerance)
                        valid = false;
                    score += error * error;
                }
                if (valid && score < best) {
                    best = score;
                    best_indices = indices;
                }
            }
        }
    }
    if (!std::isfinite(best)) {
        if (options.min_holes > 3) return false;
        const std::array<double, 3> expected{{std::min(width, height), std::max(width, height), diagonal}};
        for (std::size_t a = 0; a < candidates.size(); ++a)
        for (std::size_t b = a + 1; b < candidates.size(); ++b)
        for (std::size_t c = b + 1; c < candidates.size(); ++c) {
            std::array<double, 3> lengths{{distance(candidates[a], candidates[b]),
                distance(candidates[a], candidates[c]), distance(candidates[b], candidates[c])}};
            std::sort(lengths.begin(), lengths.end());
            bool valid = lengths[0] >= 2 * radius;
            double score = 0.0;
            for (std::size_t i = 0; i < 3; ++i) {
                const double error = std::abs(lengths[i] - expected[i]);
                valid = valid && std::isfinite(lengths[i]) && error <= options.geometry_tolerance;
                score += error * error;
            }
            if (valid && score < best) {
                best = score;
                best_indices = {{a, b, c, 0}};
            }
        }
        if (!std::isfinite(best)) return false;
        for (std::size_t i = 0; i < 3; ++i) selected.push_back(candidates[best_indices[i]]);
        return true;
    }
    for (const auto index : best_indices)
        selected.push_back(candidates[index]);
    return true;
}
}  // namespace lidar_holes
#endif
