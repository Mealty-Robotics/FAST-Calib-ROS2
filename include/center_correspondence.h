#ifndef FAST_CALIB_CENTER_CORRESPONDENCE_H
#define FAST_CALIB_CENTER_CORRESPONDENCE_H

#include "lidar_holes.h"
#include <pcl/registration/transformation_estimation_svd.h>
#include <Eigen/Geometry>

// Match only observed centers. Three corners have distances width, height,
// diagonal; no fourth measurement is invented. Rectangle symmetries are
// resolved using the known mounting (+Z forward, +X down, +Y left).
inline bool matchPatternCenters(const lidar_holes::Cloud& lidar,
    const lidar_holes::Cloud& camera, double tolerance, double max_mounting_degrees,
    lidar_holes::Cloud& matched_camera, Eigen::Matrix4f& camera_from_lidar,
    double max_rmse = 0.005, double* best_rmse = nullptr)
{
    matched_camera.clear();
    camera_from_lidar.setIdentity();
    if (best_rmse) *best_rmse = std::numeric_limits<double>::infinity();
    if (!std::isfinite(max_rmse) || max_rmse <= 0.0) return false;
    if ((lidar.size() != 3 && lidar.size() != 4) || camera.size() != 4)
        return false;
    Eigen::Matrix3f nominal;
    nominal << 0, -1, 0, 1, 0, 0, 0, 0, 1;
    std::array<int, 4> assignment{{0, 1, 2, 3}};
    double best = std::numeric_limits<double>::infinity();
    do {
        lidar_holes::Cloud target;
        for (std::size_t i = 0; i < lidar.size(); ++i)
            target.push_back(camera[assignment[i]]);
        bool geometry_valid = true;
        for (std::size_t i = 0; i < lidar.size(); ++i)
        for (std::size_t j = i + 1; j < lidar.size(); ++j) {
            const double a = (lidar[i].getVector3fMap() - lidar[j].getVector3fMap()).norm();
            const double b = (target[i].getVector3fMap() - target[j].getVector3fMap()).norm();
            if (!std::isfinite(a) || !std::isfinite(b) || std::abs(a - b) > tolerance)
                geometry_valid = false;
        }
        if (!geometry_valid) continue;
        Eigen::Matrix4f transform;
        pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;
        svd.estimateRigidTransformation(lidar, target, transform);
        if (!transform.allFinite()) continue;
        const Eigen::Matrix3f delta = transform.block<3, 3>(0, 0) * nominal.transpose();
        const double angle = Eigen::AngleAxisf(delta).angle();
        if (angle > max_mounting_degrees * std::acos(-1.0) / 180.0) continue;
        double squared_error = 0.0;
        for (std::size_t i = 0; i < lidar.size(); ++i) {
            const Eigen::Vector4f p = transform * Eigen::Vector4f(lidar[i].x, lidar[i].y, lidar[i].z, 1);
            squared_error += (p.head<3>() - target[i].getVector3fMap()).squaredNorm();
        }
        const double score = squared_error / lidar.size();
        if (score < best) {
            best = score;
            matched_camera = target;
            camera_from_lidar = transform;
        }
    } while (std::next_permutation(assignment.begin(), assignment.end()));
    if (best_rmse) *best_rmse = std::sqrt(best);
    if (!std::isfinite(best) || best > max_rmse * max_rmse) {
        matched_camera.clear();
        camera_from_lidar.setIdentity();
        return false;
    }
    return true;
}

#endif
