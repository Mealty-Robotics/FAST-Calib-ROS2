/* 
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIDAR_DETECT_HPP
#define LIDAR_DETECT_HPP

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pcl/filters/voxel_grid.h>
#include "common_lib.h"
#include "lidar_holes.h"

class LidarDetect
{
private:
    double x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;
    double circle_radius_;
    double target_width_, target_height_;
    lidar_holes::Options options_;
    double boundary_search_radius_, cluster_tolerance_;
    bool has_board_transform_ = false;
    pcl::PointCloud<pcl::PointXYZ>::Ptr candidates_{new pcl::PointCloud<pcl::PointXYZ>};
    pcl::PointCloud<pcl::PointXYZ>::Ptr rejected_{new pcl::PointCloud<pcl::PointXYZ>};
    Eigen::Matrix4d board_transform_ = Eigen::Matrix4d::Identity();
    int min_points_per_voxel_;
    double plane_distance_threshold_;
    std::shared_ptr<rclcpp::Node> node_;

    // 存储中间结果的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr edge_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr center_z0_cloud_;

public:
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> filtered_pub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> plane_pub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> aligned_pub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> edge_pub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> center_z0_pub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> center_pub_;

    LidarDetect(std::shared_ptr<rclcpp::Node> node, Params &params)
        : node_(node),
          filtered_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
          plane_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
          aligned_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
          edge_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
          center_z0_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    {
        x_min_ = params.x_min;
        x_max_ = params.x_max;
        y_min_ = params.y_min;
        y_max_ = params.y_max;
        z_min_ = params.z_min;
        z_max_ = params.z_max;
        circle_radius_ = params.circle_radius;
        options_ = params.hole_options;
        boundary_search_radius_ = params.boundary_search_radius;
        cluster_tolerance_ = params.edge_cluster_tolerance;
        target_width_ = params.delta_width_circles;
        target_height_ = params.delta_height_circles;
        min_points_per_voxel_ = params.min_points_per_voxel;
        plane_distance_threshold_ = params.plane_distance_threshold;

        filtered_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_cloud", rclcpp::QoS(1).reliable().transient_local());
        plane_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("plane_cloud", rclcpp::QoS(1).reliable().transient_local());
        aligned_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("aligned_cloud", rclcpp::QoS(1).reliable().transient_local());
        edge_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("edge_cloud", rclcpp::QoS(1).reliable().transient_local());
        center_z0_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("center_z0_cloud", rclcpp::QoS(1).reliable().transient_local());
        center_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("center_cloud", rclcpp::QoS(1).reliable().transient_local());
    }

    bool detect_lidar(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr center_cloud)
    {
        center_cloud->clear();
        candidates_->clear();
        rejected_->clear();
        has_board_transform_ = false;
        center_z0_cloud_->clear();
        plane_cloud_->clear();
        aligned_cloud_->clear();
        edge_cloud_->clear();

        if (min_points_per_voxel_ < 1 || !std::isfinite(plane_distance_threshold_) ||
            plane_distance_threshold_ <= 0.0) {
            RCLCPP_ERROR(node_->get_logger(), "Invalid voxel support or plane distance threshold.");
            return false;
        }

        // 1. X、Y、Z方向滤波
        filtered_cloud_->clear();
        filtered_cloud_->reserve(cloud->size());

        pcl::PassThrough<pcl::PointXYZ> pass_x;
        pass_x.setInputCloud(cloud);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(x_min_, x_max_);
        pass_x.filter(*filtered_cloud_);
    
        pcl::PassThrough<pcl::PointXYZ> pass_y;
        pass_y.setInputCloud(filtered_cloud_);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(y_min_, y_max_);
        pass_y.filter(*filtered_cloud_);
    
        pcl::PassThrough<pcl::PointXYZ> pass_z;
        pass_z.setInputCloud(filtered_cloud_);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(z_min_, z_max_);
        pass_z.filter(*filtered_cloud_);
    
        RCLCPP_INFO(node_->get_logger(), "Filtered cloud size: %ld", filtered_cloud_->size());
        
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(filtered_cloud_);
        voxel_filter.setLeafSize(0.005f, 0.005f, 0.005f);
        voxel_filter.setMinimumPointsNumberPerVoxel(min_points_per_voxel_);
        voxel_filter.filter(*filtered_cloud_);
        RCLCPP_INFO(node_->get_logger(), "Filtered cloud size: %ld", filtered_cloud_->size());

        if (filtered_cloud_->size() < 3) {
            RCLCPP_ERROR(node_->get_logger(), "Not enough LiDAR points for plane detection.");
            return false;
        }

        // 2. 平面分割
        plane_cloud_->clear();
        plane_cloud_->reserve(filtered_cloud_->size());

        pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> plane_segmentation;
        plane_segmentation.setModelType(pcl::SACMODEL_PLANE);
        plane_segmentation.setMethodType(pcl::SAC_RANSAC);
        plane_segmentation.setDistanceThreshold(plane_distance_threshold_);
        plane_segmentation.setInputCloud(filtered_cloud_);
        plane_segmentation.segment(*plane_inliers, *plane_coefficients);
    
        if (plane_inliers->indices.size() < 3 || plane_coefficients->values.size() < 4) {
            RCLCPP_ERROR(node_->get_logger(), "No calibration board plane found.");
            return false;
        }

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(filtered_cloud_);
        extract.setIndices(plane_inliers);
        extract.filter(*plane_cloud_);
        RCLCPP_INFO(node_->get_logger(), "Plane cloud size: %ld", plane_cloud_->size());
    
        // 3. 平面点云对齐
        aligned_cloud_->clear();
        aligned_cloud_->reserve(plane_cloud_->size());

        Eigen::Vector3d normal(plane_coefficients->values[0],
            plane_coefficients->values[1],
            plane_coefficients->values[2]);
        if (!normal.allFinite() || normal.norm() < 1e-6) {
            RCLCPP_ERROR(node_->get_logger(), "Invalid calibration board normal.");
            return false;
        }
        normal.normalize();
        logPlaneOrientation(node_->get_logger(), "LiDAR", "calib_lidar", normal);
        const Eigen::Matrix3d R = Eigen::Quaterniond::FromTwoVectors(
            normal, Eigen::Vector3d::UnitZ()).toRotationMatrix();

        // 应用旋转矩阵，将平面对齐到 Z=0 平面
        double average_z = 0.0;
        int cnt = 0;
        for (const auto& pt : *plane_cloud_) {
            Eigen::Vector3d point(pt.x, pt.y, pt.z);
            Eigen::Vector3d aligned_point = R * point;
            aligned_cloud_->push_back(pcl::PointXYZ(aligned_point.x(), aligned_point.y(), 0.0));
            average_z += aligned_point.z();
            cnt++;
        }
        average_z /= cnt;
        board_transform_.setIdentity();
        board_transform_.block<3, 3>(0, 0) = R.transpose();
        board_transform_.block<3, 1>(0, 3) = R.transpose() * Eigen::Vector3d(0.0, 0.0, average_z);
        has_board_transform_ = true;

        // 4. 提取边缘点
        edge_cloud_->clear();
        edge_cloud_->reserve(aligned_cloud_->size());

        // The cloud is already planar and aligned with Z. Estimating normals
        // on individual sparse scan rings is ill-conditioned; use the known
        // plane normal and span adjacent rings when looking for boundaries.
        const double boundary_radius = boundary_search_radius_ > 0 ? boundary_search_radius_ :
            std::max(0.03, circle_radius_ * 2.0 / 3.0);
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        normals->assign(aligned_cloud_->size(), pcl::Normal(0.0f, 0.0f, 1.0f));
    
        pcl::PointCloud<pcl::Boundary> boundaries;
        pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundary_estimator;
        boundary_estimator.setInputCloud(aligned_cloud_);
        boundary_estimator.setInputNormals(normals);
        boundary_estimator.setRadiusSearch(boundary_radius);
        boundary_estimator.setAngleThreshold(M_PI / 2);
        boundary_estimator.compute(boundaries);
    
        for (size_t i = 0; i < aligned_cloud_->size(); ++i) {
            if (boundaries.points[i].boundary_point > 0) {
                edge_cloud_->push_back(aligned_cloud_->points[i]);
            }
        }
        RCLCPP_INFO(node_->get_logger(), "Extracted %ld edge points.", edge_cloud_->size());

        // 5. 对边缘点进行聚类
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(edge_cloud_);
    
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(cluster_tolerance_ > 0 ? cluster_tolerance_ : boundary_radius / 2.0);
        ec.setMinClusterSize(options_.min_inliers);
        ec.setMaxClusterSize(std::numeric_limits<int>::max());
        ec.setSearchMethod(tree);
        ec.setInputCloud(edge_cloud_);
        ec.extract(cluster_indices);
    
        RCLCPP_INFO(node_->get_logger(), "Number of edge clusters: %ld", cluster_indices.size());
    
        // Fit and validate all candidates before choosing a target. The full
        // aligned board is needed to distinguish holes from concentric scans.
        std::vector<lidar_holes::CircleCandidate> circle_candidates;
        lidar_holes::SearchStats search_stats;
        for (const auto& indices : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto index : indices.indices)
                cluster->push_back(edge_cloud_->points[index]);
            lidar_holes::extractCandidates(cluster, *aligned_cloud_, circle_radius_, circle_candidates, &search_stats, options_);
        }
        // Keep every validated fit for geometry selection. Nearby fits are
        // alternative center hypotheses, not interchangeable measurements:
        // radius error alone cannot decide which belongs to the rectangle.
        lidar_holes::Cloud hypotheses;
        for (const auto& candidate : circle_candidates)
            hypotheses.push_back(candidate.center);
        // Deduplicate only the RViz summary; selectTarget enforces distinct
        // holes by their separation while evaluating the full hypothesis set.
        const auto candidates = lidar_holes::distinctCandidates(circle_candidates, circle_radius_, &search_stats);
        *candidates_ = candidates;
        *rejected_ = search_stats.rejected_centers;
        RCLCPP_INFO(node_->get_logger(),
            "Circle search: %zu fits, %zu rejected for radius/error, %zu for interior/surround support, "
            "%zu for partial arcs, %zu nearby alternatives, %zu distinct candidates (radius %.3f m).",
            search_stats.fits, search_stats.rejected_radius, search_stats.rejected_interior,
            search_stats.rejected_coverage, search_stats.rejected_duplicate, candidates.size(), circle_radius_);

        if (!lidar_holes::selectTarget(hypotheses, target_width_, target_height_,
                                      circle_radius_, *center_z0_cloud_, options_)) {
            RCLCPP_ERROR(node_->get_logger(),
                "No valid set of at least %d LiDAR holes (target %.3f x %.3f m, "
                "%zu distinct candidates, %zu hypotheses, distance tolerance %.4f m).",
                options_.min_holes, target_width_, target_height_, candidates.size(),
                hypotheses.size(), options_.geometry_tolerance);
            for (std::size_t i = 0; i < circle_candidates.size(); ++i) {
                const auto& candidate = circle_candidates[i];
                RCLCPP_INFO(node_->get_logger(),
                    "Hole hypothesis %zu: board XY=(%.4f, %.4f) m, radius error=%.4f m, support=%zu.",
                    i + 1, candidate.center.x, candidate.center.y, candidate.error, candidate.support);
            }
            const std::array<double, 3> expected{{std::min(target_width_, target_height_),
                std::max(target_width_, target_height_), std::hypot(target_width_, target_height_)}};
            double best_error = std::numeric_limits<double>::infinity();
            std::array<std::size_t, 3> best_indices{};
            std::array<double, 3> best_lengths{};
            for (std::size_t a = 0; a < hypotheses.size(); ++a)
            for (std::size_t b = a + 1; b < hypotheses.size(); ++b)
            for (std::size_t c = b + 1; c < hypotheses.size(); ++c) {
                std::array<double, 3> lengths{{lidar_holes::distance(hypotheses[a], hypotheses[b]),
                    lidar_holes::distance(hypotheses[a], hypotheses[c]),
                    lidar_holes::distance(hypotheses[b], hypotheses[c])}};
                std::sort(lengths.begin(), lengths.end());
                if (lengths[0] < 2 * circle_radius_) continue;
                double error = 0.0;
                for (std::size_t i = 0; i < 3; ++i)
                    error = std::max(error, std::abs(lengths[i] - expected[i]));
                if (error < best_error) {
                    best_error = error;
                    best_indices = {{a, b, c}};
                    best_lengths = lengths;
                }
            }
            if (std::isfinite(best_error))
                RCLCPP_WARN(node_->get_logger(),
                    "Closest triangle hypotheses %zu/%zu/%zu: sorted distances %.4f/%.4f/%.4f m; "
                    "expected %.4f/%.4f/%.4f m; largest distance error %.4f m (tolerance %.4f m).",
                    best_indices[0] + 1, best_indices[1] + 1, best_indices[2] + 1,
                    best_lengths[0], best_lengths[1], best_lengths[2], expected[0], expected[1], expected[2],
                    best_error, options_.geometry_tolerance);
            return false;
        }

        // Commit only a complete, validated target to the output cloud.
        const Eigen::Matrix3d R_inv = R.transpose();
        board_transform_.setIdentity();
        board_transform_.block<3, 3>(0, 0) = R_inv;
        board_transform_.block<3, 1>(0, 3) = R_inv * Eigen::Vector3d(0.0, 0.0, average_z);
        for (const auto& center : *center_z0_cloud_) {
            const Eigen::Vector3d original = R_inv * Eigen::Vector3d(center.x, center.y, average_z);
            center_cloud->push_back(pcl::PointXYZ(original.x(), original.y(), original.z()));
        }
        return true;
    }

    bool hasBoardTransform() const { return has_board_transform_; }
    pcl::PointCloud<pcl::PointXYZ>::Ptr getCandidates() const { return candidates_; }
    pcl::PointCloud<pcl::PointXYZ>::Ptr getRejected() const { return rejected_; }
    Eigen::Matrix4d getBoardTransform() const { return board_transform_; }

    // 获取中间结果的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr getFilteredCloud() const { return filtered_cloud_; }
    pcl::PointCloud<pcl::PointXYZ>::Ptr getPlaneCloud() const { return plane_cloud_; }
    pcl::PointCloud<pcl::PointXYZ>::Ptr getAlignedCloud() const { return aligned_cloud_; }
    pcl::PointCloud<pcl::PointXYZ>::Ptr getEdgeCloud() const { return edge_cloud_; }
    pcl::PointCloud<pcl::PointXYZ>::Ptr getCenterZ0Cloud() const { return center_z0_cloud_; }
};

typedef std::shared_ptr<LidarDetect> LidarDetectPtr;

#endif
