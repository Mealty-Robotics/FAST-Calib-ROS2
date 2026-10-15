#ifndef FAST_CALIB_CALIBRATION_VIEW_HPP
#define FAST_CALIB_CALIBRATION_VIEW_HPP

#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>

struct CalibrationView
{
    cv::Mat rectified;
    cv::Mat overlay;
    sensor_msgs::msg::CameraInfo info;
    std::size_t projected_pixels = 0;
};

inline CalibrationView makeCalibrationView(const cv::Mat& image, const cv::Mat& intrinsics,
    const cv::Mat& distortion, const pcl::PointCloud<pcl::PointXYZ>& raw_cloud,
    const Eigen::Matrix4f& camera_from_raw)
{
    CalibrationView view;
    cv::Mat k;
    intrinsics.convertTo(k, CV_64F);
    cv::undistort(image, view.rectified, k, distortion, k);
    view.info.width = image.cols;
    view.info.height = image.rows;
    view.info.distortion_model = "plumb_bob";
    view.info.d.assign(5, 0.0);  // Published image is already rectified.
    for (int row = 0; row < 3; ++row) {
        view.info.r[row * 3 + row] = 1.0;
        for (int col = 0; col < 3; ++col) {
            view.info.k[row * 3 + col] = k.at<double>(row, col);
            view.info.p[row * 4 + col] = k.at<double>(row, col);
        }
    }

    // Retain the nearest return at each image pixel. Depth colors are
    // independent of the photograph so misalignment remains visible.
    cv::Mat depth(image.rows, image.cols, CV_32F,
                  cv::Scalar(std::numeric_limits<float>::infinity()));
    for (const auto& point : raw_cloud) {
        const Eigen::Vector4f p = camera_from_raw * Eigen::Vector4f(point.x, point.y, point.z, 1.0f);
        if (!p.allFinite() || p.z() <= 0.001f)
            continue;
        const double u = k.at<double>(0, 0) * p.x() / p.z() + k.at<double>(0, 2);
        const double v = k.at<double>(1, 1) * p.y() / p.z() + k.at<double>(1, 2);
        if (u < 0 || v < 0 || u >= image.cols || v >= image.rows)
            continue;
        float& nearest = depth.at<float>(static_cast<int>(v), static_cast<int>(u));
        nearest = std::min(nearest, p.z());
    }
    cv::Mat ramp(1, 256, CV_8U), colors;
    for (int i = 0; i < 256; ++i)
        ramp.at<unsigned char>(0, i) = i;
    cv::applyColorMap(ramp, colors, cv::COLORMAP_TURBO);
    cv::Mat dots = view.rectified.clone();
    for (int y = 0; y < image.rows; ++y)
    for (int x = 0; x < image.cols; ++x) {
        const float z = depth.at<float>(y, x);
        if (!std::isfinite(z))
            continue;
        ++view.projected_pixels;
        const int index = std::clamp(static_cast<int>((z - 0.5f) / 7.5f * 255), 0, 255);
        const cv::Vec3b color = colors.at<cv::Vec3b>(0, index);
        cv::circle(dots, cv::Point(x, y), 1, cv::Scalar(color[0], color[1], color[2]), -1);
    }
    cv::addWeighted(dots, 0.75, view.rectified, 0.25, 0, view.overlay);
    return view;
}

#endif
