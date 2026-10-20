/* 
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "qr_detect.hpp"
#include "lidar_detect.hpp"
#include "data_preprocess.hpp"
#include "calibration_view.hpp"
#include "center_correspondence.h"
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <functional>

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("mono_qr_pattern");

    // Load parameters
    Params params = loadParameters(node);

    // Initialize QR detection and LiDAR detection
    QRDetectPtr qrDetectPtr;
    qrDetectPtr.reset(new QRDetect(node, params));

    LidarDetectPtr lidarDetectPtr;
    lidarDetectPtr.reset(new LidarDetect(node, params));

    DataPreprocessPtr dataPreprocessPtr;
    dataPreprocessPtr.reset(new DataPreprocess(params));

    // Read image and point cloud
    cv::Mat img_input = dataPreprocessPtr->img_input_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input = dataPreprocessPtr->cloud_input_;
    
    // Detect QR codes
    pcl::PointCloud<pcl::PointXYZ>::Ptr qr_center_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    qr_center_cloud->reserve(4);
    if (!img_input.empty()) qrDetectPtr->detect_qr(img_input, qr_center_cloud);

    // Detect LiDAR data
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_center_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    lidar_center_cloud->reserve(4);
    const bool lidar_valid = lidarDetectPtr->detect_lidar(cloud_input, lidar_center_cloud);
    auto qr_centers = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto lidar_centers = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    if (lidar_valid) *lidar_centers = *lidar_center_cloud;
    double best_center_rmse = std::numeric_limits<double>::infinity();
    const bool calibration_valid = lidar_valid && matchPatternCenters(*lidar_centers, *qr_center_cloud,
        params.hole_options.geometry_tolerance, params.max_mounting_rotation_deg,
        *qr_centers, transformation, params.max_calibration_rmse, &best_center_rmse);
    if (!calibration_valid && std::isfinite(best_center_rmse))
        RCLCPP_ERROR(node->get_logger(), "Calibration rejected: best center RMSE %.2f mm exceeds limit %.2f mm.",
            best_center_rmse * 1000.0, params.max_calibration_rmse * 1000.0);
    auto aligned_lidar_centers = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    const Eigen::Matrix4f raw_from_standard = standardToRawLidar();
    Eigen::Matrix4f camera_from_standard = Eigen::Matrix4f::Identity();
    const auto rectified_pixel = [&](const pcl::PointXYZ& p) {
        return cv::Point2d(params.fx * p.x / p.z + params.cx,
                           params.fy * p.y / p.z + params.cy);
    };
    if (calibration_valid) {
        alignPointCloud(lidar_centers, aligned_lidar_centers, transformation);
        camera_from_standard = transformation * raw_from_standard;
        RCLCPP_INFO(node->get_logger(), "Calibration uses %zu observed LiDAR centers; no missing center is synthesized.",
            lidar_centers->size());
        const double rmse = computeRMSE(qr_centers, aligned_lidar_centers);
        RCLCPP_INFO(node->get_logger(), "Center RMSE: %.6f m", rmse);
        std::filesystem::create_directories(params.output_path);
        std::ofstream report(std::filesystem::path(params.output_path) / "center_residuals.csv");
        report << "hole,lidar_x,lidar_y,lidar_z,camera_x,camera_y,camera_z,"
            "error_x_m,error_y_m,error_z_m,error_m,error_u_px,error_v_px,error_px\n";
        report << std::setprecision(10);
        for (std::size_t i = 0; i < qr_centers->size(); ++i) {
            const auto& raw = (*lidar_centers)[i];
            const auto& expected = (*qr_centers)[i];
            const auto& measured = (*aligned_lidar_centers)[i];
            const Eigen::Vector3f delta = measured.getVector3fMap() - expected.getVector3fMap();
            const cv::Point2d pixels = rectified_pixel(measured) - rectified_pixel(expected);
            RCLCPP_INFO(node->get_logger(), "Center %zu residual: %.2f mm, %.2f px (du=%.2f, dv=%.2f).",
                i + 1, delta.norm() * 1000.0, cv::norm(pixels), pixels.x, pixels.y);
            report << i + 1 << ',' << raw.x << ',' << raw.y << ',' << raw.z << ','
                << expected.x << ',' << expected.y << ',' << expected.z << ','
                << delta.x() << ',' << delta.y() << ',' << delta.z() << ',' << delta.norm() << ','
                << pixels.x << ',' << pixels.y << ',' << cv::norm(pixels) << '\n';
        }
        std::cout << "T_cam_lidar:\n" << transformation << "\nT_cam_lidar_standard:\n"
                  << camera_from_standard << std::endl;
    } else {
        RCLCPP_ERROR(node->get_logger(),
            "Calibration failed: need %d or more rectangle corners and unambiguous camera correspondences "
            "within %.1f degrees of the mounting, center RMSE <= %.2f mm and camera RMSE <= %.2f px. "
            "No extrinsics will be saved. Diagnostics stay active.",
            params.hole_options.min_holes, params.max_mounting_rotation_deg,
            params.max_calibration_rmse * 1000.0, params.max_camera_reprojection_rmse);
    }

    // Display each cloud in its actual frame. T_cam_lidar maps original
    // LiDAR coordinates into camera optical coordinates; no mounting rotation
    // is applied to the SVD inputs or to the recorded point cloud.
    auto tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
    const auto make_tf = [&](const Eigen::Matrix4d& matrix, const std::string& parent,
                             const std::string& child) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = node->now();
        tf.header.frame_id = parent;
        tf.child_frame_id = child;
        tf.transform.translation.x = matrix(0, 3);
        tf.transform.translation.y = matrix(1, 3);
        tf.transform.translation.z = matrix(2, 3);
        const Eigen::Quaterniond q(matrix.block<3, 3>(0, 0));
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();
        return tf;
    };
    std::vector<geometry_msgs::msg::TransformStamped> transforms{
        make_tf(raw_from_standard.cast<double>(), "calib_lidar", "calib_lidar_standard")};
    if (lidarDetectPtr->hasBoardTransform())
        transforms.push_back(make_tf(lidarDetectPtr->getBoardTransform(), "calib_lidar", "calib_board"));
    if (calibration_valid)
        transforms.push_back(make_tf(transformation.cast<double>(), "calib_camera_optical", "calib_lidar"));
    // With no calibration the camera remains a separate frame. Never publish
    // an identity camera/LiDAR transform that would look like a valid result.
    tf_broadcaster->sendTransform(transforms);

    // Latched snapshots let RViz connect before or after computation. Publish
    // the board and centers before the expensive full-cloud projection/export.
    const auto snapshot_qos = rclcpp::QoS(1).reliable().transient_local();
    auto colored_cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "colored_cloud", snapshot_qos);
    auto aligned_lidar_centers_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "aligned_lidar_centers", snapshot_qos);
    const auto publish_cloud = [&](const auto& cloud, const auto& publisher,
                                   const std::string& frame) {
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header.stamp = node->now();
        msg.header.frame_id = frame;
        publisher->publish(msg);
    };
    if (DEBUG) {
        publish_cloud(qr_center_cloud, qrDetectPtr->qr_pub_, "calib_camera_optical");
        publish_cloud(lidar_centers, lidarDetectPtr->center_pub_, "calib_lidar");
        publish_cloud(lidarDetectPtr->getFilteredCloud(), lidarDetectPtr->filtered_pub_, "calib_lidar");
        publish_cloud(lidarDetectPtr->getPlaneCloud(), lidarDetectPtr->plane_pub_, "calib_lidar");
        publish_cloud(lidarDetectPtr->getAlignedCloud(), lidarDetectPtr->aligned_pub_, "calib_board");
        publish_cloud(lidarDetectPtr->getEdgeCloud(), lidarDetectPtr->edge_pub_, "calib_board");
        publish_cloud(lidarDetectPtr->getCenterZ0Cloud(), lidarDetectPtr->center_z0_pub_, "calib_board");
        publish_cloud(aligned_lidar_centers, aligned_lidar_centers_pub, "calib_camera_optical");
    }
    auto candidates_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("hole_candidates", snapshot_qos);
    auto rejected_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("rejected_circles", snapshot_qos);
    if (lidarDetectPtr->hasBoardTransform()) {
        publish_cloud(lidarDetectPtr->getCandidates(), candidates_pub, "calib_board");
        publish_cloud(lidarDetectPtr->getRejected(), rejected_pub, "calib_board");
    }
    RCLCPP_INFO(node->get_logger(), "Diagnostic snapshots ready for RViz (fixed frame calib_lidar).");

    // Full-scene verification, independent of the target ROI and image colors.
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> scene_filter;
    scene_filter.setInputCloud(cloud_input);
    scene_filter.setLeafSize(0.01f, 0.01f, 0.01f);
    if (!cloud_input->empty()) scene_filter.filter(*scene);
    auto scene_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "calibration_scene", snapshot_qos);
    publish_cloud(scene, scene_pub, "calib_lidar");
    CalibrationView view;
    if (!img_input.empty()) {
        const pcl::PointCloud<pcl::PointXYZ> empty_scene;
        view = makeCalibrationView(img_input, qrDetectPtr->cameraMatrix_,
            qrDetectPtr->distCoeffs_, calibration_valid ? *scene : empty_scene, transformation);
        if (!calibration_valid)
            cv::putText(view.overlay, "NO CALIBRATION - IMAGE ONLY", cv::Point(25, 40),
                cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 0, 255), 2);
    }
    for (std::size_t i = 0; i < qr_centers->size(); ++i) {
        const cv::Point expected = rectified_pixel((*qr_centers)[i]);
        const cv::Point measured = rectified_pixel((*aligned_lidar_centers)[i]);
        cv::arrowedLine(view.overlay, measured, expected, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
        cv::drawMarker(view.overlay, expected, cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 14, 2);
        cv::drawMarker(view.overlay, measured, cv::Scalar(0, 0, 255), cv::MARKER_TILTED_CROSS, 14, 2);
    }
    auto image_pub = node->create_publisher<sensor_msgs::msg::Image>(
        "calibration_camera/image_rect", snapshot_qos);
    auto info_pub = node->create_publisher<sensor_msgs::msg::CameraInfo>(
        "calibration_camera/camera_info", snapshot_qos);
    auto overlay_pub = node->create_publisher<sensor_msgs::msg::Image>(
        "calibration_camera/overlay", snapshot_qos);
    std::function<void()> publish_camera = [image_pub, info_pub, overlay_pub, clock = node->get_clock(),
        image = view.rectified.empty() ? sensor_msgs::msg::Image() :
            *cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", view.rectified).toImageMsg(),
        overlay = view.overlay.empty() ? sensor_msgs::msg::Image() :
            *cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", view.overlay).toImageMsg(),
        info = view.info]() mutable {
        info.header.stamp = clock->now();
        info.header.frame_id = "calib_camera_optical";
        image.header = info.header;
        overlay.header = info.header;
        info_pub->publish(info);
        image_pub->publish(image);
        overlay_pub->publish(overlay);
    };
    if (!img_input.empty()) publish_camera();
    // CameraInfo subscribers may be volatile; repeat snapshots for late viewers.
    auto camera_timer = node->create_wall_timer(std::chrono::seconds(1), [&]() {
        if (!img_input.empty()) publish_camera();
    });
    RCLCPP_INFO(node->get_logger(), "Camera verification ready: %zu projected pixels.", view.projected_pixels);
    if (!calibration_valid) {
        RCLCPP_WARN(node->get_logger(), "Image shown separately; camera overlay unavailable without extrinsics. Ctrl+C to exit diagnostics.");
        if (rclcpp::ok()) rclcpp::spin(node);
        rclcpp::shutdown();
        return 1;
    }
    std::filesystem::create_directories(params.output_path);
    cv::imwrite((std::filesystem::path(params.output_path) / "projection_overlay.png").string(), view.overlay);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    projectPointCloudToImage(cloud_input, transformation, qrDetectPtr->cameraMatrix_, qrDetectPtr->distCoeffs_, img_input, colored_cloud);
    // Keep the RViz preview small even when the bag contains millions of points.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr preview(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> preview_filter;
    preview_filter.setInputCloud(colored_cloud);
    preview_filter.setLeafSize(0.01f, 0.01f, 0.01f);
    preview_filter.filter(*preview);
    if (DEBUG)
        publish_cloud(preview, colored_cloud_pub, "calib_camera_optical");
    saveCalibrationResults(params, transformation, colored_cloud, qrDetectPtr->imageCopy_, camera_from_standard);

    if (rclcpp::ok())
        rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
