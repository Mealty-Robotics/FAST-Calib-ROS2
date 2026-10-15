#ifndef FAST_CALIB_LIDAR_HOLES_H
#define FAST_CALIB_LIDAR_HOLES_H

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace lidar_holes {
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
constexpr std::size_t min_circle_inliers = 20;
constexpr double fit_tolerance = 0.02;
constexpr double geometry_tolerance = 0.03;

struct SearchStats {
    std::size_t fits = 0;
    std::size_t rejected_radius = 0;
    std::size_t rejected_interior = 0;
    std::size_t rejected_duplicate = 0;
};

inline double distance(const pcl::PointXYZ& a, const pcl::PointXYZ& b)
{
    return std::hypot(a.x - b.x, a.y - b.y);
}

// Use the entire board, not just boundary points: scan rings can fit circles
// even though there are board returns well inside them. Ignore the outer 20%
// of the radius for edge noise, and allow at most two isolated interior returns.
inline bool emptyInterior(const Cloud& board, const pcl::PointXYZ& center, double radius)
{
    std::size_t interior = 0;
    for (const auto& point : board) {
        if (distance(point, center) < 0.8 * radius && ++interior > 2)
            return false;
    }
    return true;
}

inline bool distinctCenter(const Cloud& centers, const pcl::PointXYZ& center, double radius)
{
    for (const auto& previous : centers) {
        if (distance(previous, center) < 2.0 * radius)
            return false;
    }
    return true;
}

inline void extractCandidates(Cloud::Ptr remaining, const Cloud& board,
                              double radius, Cloud& centers, SearchStats* stats = nullptr)
{
    if (!std::isfinite(radius) || radius <= 0.0)
        return;
    // A cluster may contain several circles. Always remove the fitted inliers,
    // including rejected scan rings, so they cannot hide subsequent holes.
    while (remaining->size() >= min_circle_inliers) {
        pcl::ModelCoefficients coefficients;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CIRCLE2D);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01);
        seg.setMaxIterations(1000);
        seg.setRadiusLimits(std::max(0.0, radius - fit_tolerance), radius + fit_tolerance);
        seg.setInputCloud(remaining);
        seg.segment(*inliers, coefficients);
        if (inliers->indices.size() < min_circle_inliers)
            break;
        if (stats)
            ++stats->fits;

        if (coefficients.values.size() >= 3 &&
            std::isfinite(coefficients.values[0]) &&
            std::isfinite(coefficients.values[1]) &&
            std::isfinite(coefficients.values[2]) &&
            std::abs(coefficients.values[2] - radius) < fit_tolerance) {
            const pcl::PointXYZ center(coefficients.values[0], coefficients.values[1], 0.0f);
            double error = 0.0;
            for (const auto index : inliers->indices)
                error += std::abs(distance((*remaining)[index], center) - radius);
            error /= inliers->indices.size();
            if (error >= fit_tolerance) {
                if (stats) ++stats->rejected_radius;
            } else if (!emptyInterior(board, center, radius)) {
                if (stats) ++stats->rejected_interior;
            } else if (!distinctCenter(centers, center, radius)) {
                if (stats) ++stats->rejected_duplicate;
            } else {
                centers.push_back(center);
            }
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

// Test all assignments of the four corners. Checking all six distances with
// their corner identities enforces both opposite sides and both diagonals.
// Keep searching after an invalid quartet and choose the lowest-error set.
inline bool selectTarget(const Cloud& candidates, double width, double height,
                         double radius, Cloud& selected)
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
        if (b == a || std::abs(distance(candidates[a], candidates[b]) - width) > geometry_tolerance)
            continue;
        for (std::size_t c = 0; c < candidates.size(); ++c) {
            if (c == a || c == b ||
                std::abs(distance(candidates[a], candidates[c]) - height) > geometry_tolerance ||
                std::abs(distance(candidates[b], candidates[c]) - diagonal) > geometry_tolerance)
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
                    if (!std::isfinite(actual) || actual < 2 * radius || error > geometry_tolerance)
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
    if (!std::isfinite(best))
        return false;
    for (const auto index : best_indices)
        selected.push_back(candidates[index]);
    return true;
}
}  // namespace lidar_holes
#endif
