#ifndef FAST_CALIB_PLANE_FIT_H
#define FAST_CALIB_PLANE_FIT_H

#include <algorithm>
#include <cmath>
#include <vector>
#include <Eigen/Eigenvalues>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace board_plane {

struct Fit {
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    double offset = 0.0;
    double initial_normal_change_deg = 0.0;
    int iterations = 0;
    std::vector<int> inliers;
};

// RANSAC identifies the board; a smooth robust objective determines its plane.
// Refit ALL cropped/voxelized points, not a seed-dependent hard inlier subset.
// Cauchy weights suppress distant returns without cutting points in and out
// of the optimization as the normal moves. The final hard mask is computed
// only after convergence and never used to move the fitted plane again.
inline bool refine(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                   Eigen::Vector4d initial, double distance_threshold, Fit& result)
{
    result = Fit{};
    if (cloud.size() < 3 || !initial.allFinite() || initial.head<3>().norm() < 1e-9 ||
        !std::isfinite(distance_threshold) || distance_threshold <= 0.0)
        return false;
    initial /= initial.head<3>().norm();
    if (initial.z() < 0.0) initial = -initial;
    Eigen::Vector3d normal = initial.head<3>();
    double offset = initial.w();
    bool converged = false;
    for (int iteration = 0; iteration < 100; ++iteration) {
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
        double weight_sum = 0.0;
        std::size_t supported = 0;
        for (const auto& point : cloud) {
            const Eigen::Vector3d p(point.x, point.y, point.z);
            if (!p.allFinite()) continue;
            const double residual = (normal.dot(p) + offset) / distance_threshold;
            const double weight = 1.0 / (1.0 + residual * residual);
            if (!std::isfinite(weight) || weight <= 0.0) continue;
            ++supported;
            weight_sum += weight;
            const Eigen::Vector3d delta = p - center;
            center += (weight / weight_sum) * delta;
            scatter += weight * delta * (p - center).transpose();
        }
        if (supported < 3 || weight_sum < 1e-12) return false;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(scatter / weight_sum);
        if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite() ||
            solver.eigenvalues()[1] < 1e-10)
            return false;
        Eigen::Vector3d next_normal = solver.eigenvectors().col(0);
        if (next_normal.dot(normal) < 0.0) next_normal = -next_normal;
        const double next_offset = -next_normal.dot(center);
        converged = (next_normal - normal).norm() < 1e-9 &&
                    std::abs(next_offset - offset) < 1e-9;
        normal = next_normal;
        offset = next_offset;
        result.iterations = iteration + 1;
        if (converged) break;
    }
    if (!converged || !normal.allFinite() || !std::isfinite(offset)) return false;
    if (normal.z() < 0.0) { normal = -normal; offset = -offset; }
    result.normal = normal;
    result.offset = offset;
    result.initial_normal_change_deg = std::acos(std::clamp(
        initial.head<3>().dot(normal), -1.0, 1.0)) * 180.0 / std::acos(-1.0);
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        const auto& p = cloud[i];
        const double residual = normal.dot(Eigen::Vector3d(p.x, p.y, p.z)) + offset;
        if (std::isfinite(residual) && std::abs(residual) <= distance_threshold)
            result.inliers.push_back(static_cast<int>(i));
    }
    return result.inliers.size() >= 3;
}

}  // namespace board_plane
#endif
