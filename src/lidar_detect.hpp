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
        const Eigen::Matrix3d R = Eigen::Quaterniond::FromTwoVectors(
            normal, Eigen::Vector3d::UnitZ()).toRotationMatrix();

        // 应用旋转矩阵，将平面对齐到 Z=0 平面
        float average_z = 0.0;
        int cnt = 0;
        for (const auto& pt : *plane_cloud_) {
            Eigen::Vector3d point(pt.x, pt.y, pt.z);
            Eigen::Vector3d aligned_point = R * point;
            aligned_cloud_->push_back(pcl::PointXYZ(aligned_point.x(), aligned_point.y(), 0.0));
            average_z += aligned_point.z();
            cnt++;
        }
        average_z /= cnt;

        // 4. 提取边缘点
        edge_cloud_->clear();
        edge_cloud_->reserve(aligned_cloud_->size());

        // The cloud is already planar and aligned with Z. Estimating normals
        // on individual sparse scan rings is ill-conditioned; use the known
        // plane normal and span adjacent rings when looking for boundaries.
        const double boundary_radius = std::max(0.03, circle_radius_ * 2.0 / 3.0);
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
        ec.setClusterTolerance(boundary_radius / 2.0);
        ec.setMinClusterSize(lidar_holes::min_circle_inliers);
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
            lidar_holes::extractCandidates(cluster, *aligned_cloud_, circle_radius_, circle_candidates, &search_stats);
        }
        const auto candidates = lidar_holes::distinctCandidates(circle_candidates, circle_radius_, &search_stats);
        RCLCPP_INFO(node_->get_logger(),
            "Circle search: %zu fits, %zu rejected for radius/error, %zu for occupied interiors, "
            "%zu for partial arcs, %zu duplicates, %zu candidates (radius %.3f m).",
            search_stats.fits, search_stats.rejected_radius, search_stats.rejected_interior,
            search_stats.rejected_coverage, search_stats.rejected_duplicate, candidates.size(), circle_radius_);

        if (!lidar_holes::selectTarget(candidates, target_width_, target_height_,
                                      circle_radius_, *center_z0_cloud_)) {
            RCLCPP_ERROR(node_->get_logger(),
                "No valid set of four LiDAR holes (target %.3f x %.3f m, %zu candidates).",
                target_width_, target_height_, candidates.size());
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
